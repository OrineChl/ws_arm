import os
import time

import pytest

if not os.environ.get('DISPLAY') and not os.environ.get('WAYLAND_DISPLAY'):
    os.environ.setdefault('QT_QPA_PLATFORM', 'offscreen')

from PyQt5.QtCore import Qt
from PyQt5.QtTest import QTest
from PyQt5.QtWidgets import QApplication

from custom_3dof_kinematics.gui import IKTestWindow


@pytest.fixture(scope='module')
def app():
    return QApplication.instance() or QApplication([])


@pytest.fixture
def window(app):
    widget = IKTestWindow()
    widget.show()
    QTest.qWait(50)
    yield widget
    widget.close()
    app.processEvents()


def wait_for_solver(window):
    deadline = time.monotonic() + 30
    while window.worker is not None and time.monotonic() < deadline:
        QTest.qWait(20)
    assert window.worker is None, 'Solver did not finish within 30 seconds'


def test_zero_solve_copy_and_invalidation(window, app):
    assert not window.copy_button.isEnabled()
    QTest.mouseClick(window.solve_button, Qt.LeftButton)
    wait_for_solver(window)
    assert '求解通过' in window.status.text()
    assert 'Joint_Control(' in window.output.toPlainText()
    assert len(bytes.fromhex(window.frame_hex)) == 17
    QTest.mouseClick(window.copy_button, Qt.LeftButton)
    assert app.clipboard().text() == window.frame_hex
    window.inputs[0].selectAll()
    QTest.keyClicks(window.inputs[0], '10')
    assert not window.copy_button.isEnabled()
    assert window.output.toPlainText() == ''
    assert window.frame_hex == ''
    QTest.mouseClick(window.solve_button, Qt.LeftButton)
    wait_for_solver(window)
    assert '最大伸展范围' in window.status.text()
    assert not window.copy_button.isEnabled()
    QTest.mouseClick(window.zero_button, Qt.LeftButton)
    QTest.mouseClick(window.solve_button, Qt.LeftButton)
    wait_for_solver(window)
    assert window.copy_button.isEnabled()


@pytest.mark.parametrize('text', ['', 'abc', 'NaN', 'Inf', '-Inf'])
def test_invalid_input(window, text):
    window.inputs[0].setText(text)
    QTest.mouseClick(window.solve_button, Qt.LeftButton)
    wait_for_solver(window)
    assert '输入无效' in window.status.text()
    assert not window.copy_button.isEnabled()
    assert not window.frame_hex


def test_limit_unreachable_and_close_while_solving(window):
    for field, value in zip(window.inputs, ('0.085', '0', '0.401')):
        field.setText(value)
    QTest.mouseClick(window.solve_button, Qt.LeftButton)
    assert not window.solve_button.isEnabled()
    wait_for_solver(window)
    assert '未找到' in window.status.text()
    assert window.output.toPlainText() == ''
    assert not window.copy_button.isEnabled()
    QTest.mouseClick(window.solve_button, Qt.LeftButton)
    window.close()
    QTest.qWait(20)
    assert not window.isVisible()
