import sys

import numpy as np
from PyQt5.QtCore import QThread, pyqtSignal
from PyQt5.QtWidgets import (
    QApplication, QFormLayout, QHBoxLayout, QLabel, QLineEdit,
    QPlainTextEdit, QPushButton, QVBoxLayout, QWidget,
)

from .kinematics import forward_kinematics, inverse_kinematics
from .protocol import encode_joint_command


class SolverWorker(QThread):
    solved = pyqtSignal(object)
    failed = pyqtSignal(str)

    def __init__(self, target, parent):
        super().__init__(parent)
        self.target = target

    def run(self):
        try:
            self.solved.emit(inverse_kinematics(self.target))
        except ValueError as error:
            self.failed.emit(str(error))


class IKTestWindow(QWidget):
    def __init__(self):
        super().__init__()
        self.setWindowTitle('三自由度逆运动学 · 底层控制参数预览')
        self.resize(820, 720)
        self.worker = None
        self.frame_hex = ''
        layout = QVBoxLayout(self)
        title = QLabel('XYZ → 三关节逆解 → 0x12 控制帧（仅预览）')
        font = title.font()
        font.setPointSize(15)
        font.setBold(True)
        title.setFont(font)
        layout.addWidget(title)
        coordinates = QLabel(
            '坐标系：底层末端上报坐标系（底盘固定，chassis / world），单位：米。\n'
            '包含安装偏置 (0.085, 0, 0.240) m；不是机械臂 base_link 坐标。\n'
            '仅求末端位置，不控制姿态；误差阈值 0.00001 m 仅代表模型计算精度。'
        )
        coordinates.setWordWrap(True)
        layout.addWidget(coordinates)
        form = QFormLayout()
        self.inputs = []
        for axis in 'XYZ':
            field = QLineEdit()
            field.setAccessibleName(f'{axis}（米）')
            field.setObjectName(f'input_{axis.lower()}')
            field.setPlaceholderText('米，例如 0.349')
            field.textChanged.connect(self.clear_result)
            field.returnPressed.connect(self.solve)
            form.addRow(f'{axis}（m）', field)
            self.inputs.append(field)
        layout.addLayout(form)
        buttons = QHBoxLayout()
        self.solve_button = QPushButton('求解')
        self.solve_button.clicked.connect(self.solve)
        self.zero_button = QPushButton('加载零位示例')
        self.zero_button.clicked.connect(self.load_zero)
        self.copy_button = QPushButton('复制控制帧')
        self.copy_button.clicked.connect(self.copy_frame)
        for button in (self.solve_button, self.zero_button, self.copy_button):
            buttons.addWidget(button)
        layout.addLayout(buttons)
        self.status = QLabel()
        self.status.setWordWrap(True)
        layout.addWidget(self.status)
        self.output = QPlainTextEdit()
        self.output.setReadOnly(True)
        self.output.setAccessibleName('逆解与底层控制参数')
        layout.addWidget(self.output, 1)
        warning = QLabel(
            '接口说明：发送的是逻辑关节角（度），不要乘减速比。\n'
            '0x12 不包含速度/加速度，沿用固件当前设置；它还会调用腕部/夹爪舵机的零参数，'
            '并非保持其他关节不动。\n'
            '仅离线测试：不会打开串口或发送指令。未校验碰撞、回零和实机标定，'
            '生成参数不代表实机运动安全。'
        )
        warning.setWordWrap(True)
        layout.addWidget(warning)
        self.load_zero()

    def clear_result(self):
        self.frame_hex = ''
        self.output.clear()
        self.copy_button.setEnabled(False)
        self.status.setText('输入目标后点击“求解”。')

    def load_zero(self):
        for field, value in zip(self.inputs, forward_kinematics([0, 0, 0])):
            field.setText(f'{value:.12g}')
        self.clear_result()
        self.status.setText('已加载底层零位的 XYZ；逆解可能返回位置等价的另一组关节角。')

    def set_busy(self, busy):
        for widget in (*self.inputs, self.solve_button, self.zero_button):
            widget.setEnabled(not busy)

    def solve(self):
        if self.worker is not None and self.worker.isRunning():
            return
        self.clear_result()
        try:
            target = np.array([float(field.text()) for field in self.inputs])
        except ValueError:
            self.status.setText('输入无效：X、Y、Z 都必须是数字（单位：米）。')
            return
        if not np.all(np.isfinite(target)):
            self.status.setText('输入无效：不接受 NaN 或 Inf。')
            return
        self.target = target
        self.set_busy(True)
        self.status.setText('正在求解并检查限位、float32 控制参数和末端误差…')
        self.worker = SolverWorker(target, self)
        self.worker.solved.connect(self.show_result)
        self.worker.failed.connect(self.show_error)
        self.worker.finished.connect(self.solver_finished)
        self.worker.start()

    def show_error(self, message):
        self.clear_result()
        self.status.setText(message)

    def solver_finished(self):
        self.set_busy(False)
        self.worker.deleteLater()
        self.worker = None

    def show_result(self, angles):
        frame = encode_joint_command(angles)
        self.frame_hex = frame.hex(' ').upper()
        actual = forward_kinematics(angles)
        error = np.linalg.norm(actual - self.target)
        lines = ['控制参数（float32 负载对应的关节角）', '']
        for index, (angle, motors, ratio, limits) in enumerate(zip(
            angles, ('1', '2 + 3（镜像驱动）', '4'), ('3', '6', '4.8'),
            ('[-90, 90]', '[0, 90]', '[-90, 0]'),
        ), start=1):
            lines.append(f'q{index} = {angle:.9g} 度  /  {np.deg2rad(angle):.9g} rad')
            lines.append(f'  电机 {motors}；底层减速比 {ratio}；限位 {limits} 度')
        lines.extend([
            '',
            '目标 XYZ（m）：' + ', '.join(f'{value:.10g}' for value in self.target),
            '回算 XYZ（m）：' + ', '.join(f'{value:.10g}' for value in actual),
            f'位置残差：{error:.6g} m（{error * 1000:.6g} mm）',
            '',
            '底层调用（角度单位：度）：',
            'Joint_Control(' + ', '.join(f'{angle:.9g}' for angle in angles) + ', 0, 0);',
            '',
            '0x12 串口帧：AA 55 | 12 | 0C | 3 × float32 小端 | 校验和',
            self.frame_hex,
        ])
        self.output.setPlainText('\n'.join(lines))
        self.copy_button.setEnabled(True)
        self.status.setText('求解通过：符合底层软件限位与模型精度要求。未发送任何指令。')

    def copy_frame(self):
        if self.frame_hex:
            QApplication.clipboard().setText(self.frame_hex)

    def closeEvent(self, event):
        if self.worker is not None:
            self.worker.wait()
        super().closeEvent(event)


def main():
    app = QApplication(sys.argv)
    window = IKTestWindow()
    window.show()
    return app.exec_()


if __name__ == '__main__':
    sys.exit(main())
