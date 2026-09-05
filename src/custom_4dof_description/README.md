# 自制四自由度机械臂描述包

目标环境：Ubuntu 22.04 / ROS 2 Humble。模型为 **visualization-only**，
不能用于动力学仿真或真实控制。没有质量、质心、惯量或真实关节限位。

四个独立旋转关节 joint1～joint4 临时使用 continuous；取得机械限位后改为
revolute 并填入实测限位。GUI 的显示角度区间不是机械限位或速度限制。
未实现 ros2_control、电机通信、MoveIt、相机、标定、逆运动学或正式 DH 表。

## 构建与运行

运行依赖：xacro、robot_state_publisher、joint_state_publisher、
joint_state_publisher_gui、rviz2，以及标准 ROS launch 包。
检查器 check_urdf 用于验收。缺失软件须由用户自行决定安装，本工程不自动安装。

在工作区根目录执行：

```bash
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
mkdir -p build/custom_4dof_description/validation
xacro src/custom_4dof_description/urdf/custom_4dof.urdf.xacro \
  -o build/custom_4dof_description/validation/custom_4dof.urdf
check_urdf build/custom_4dof_description/validation/custom_4dof.urdf
ros2 run custom_4dof_description verify_zero_pose.py \
  build/custom_4dof_description/validation/custom_4dof.urdf
ros2 launch custom_4dof_description display.launch.py
```

验证脚本也可直接执行，默认展开包内 Xacro；失败返回非零退出码。
它读取实际 URDF，累乘 joint origin（含 rpy），检查链、轴和零位位置，
打印实际与期望位置，容差为 1e-9 m。缺少 xacro 返回错误，不视为验证通过。

启动参数：

- `use_gui:=true`：默认启用四个关节滑块；false 使用非 GUI joint_state_publisher 发布零位。
- `model:=/absolute/path/model.urdf.xacro`：替换描述文件。
- `rviz_config:=/absolute/path/config.rviz`：替换 RViz 配置。

`use_gui:=false` 仍启动 RViz，因此并非纯无头运行模式。
桌面验收时依次拖动四个滑块，确认各关节可独立运动、子链跟随、TF 连通、无 RobotModel 错误。
连续关节的 GUI 初值为零。RViz Fixed Frame 为 base_link，RobotModel 订阅 /robot_description。
无图形界面时先完成构建、Xacro/URDF 和零位验证，RViz 留待有桌面的 Ubuntu 22.04 主机验证。

## 参数与模型约定

几何长度集中在 `urdf/custom_4dof.urdf.xacro` 的 property 中；
`config/robot_parameters.yaml` 是工程参数记录，不是传给 ROS 节点的参数文件，修改尺寸时同步更新记录。
零位验证的期望值保持独立，防止模型尺寸被误改却仍然通过。
简单几何体的宽度、底座半径等为示意尺寸，visual/collision 相同，不表示精确实物包络。

坐标转换见 [coordinate_conventions.md](docs/coordinate_conventions.md)。
关节轴依次为 (0,0,1)、(0,-1,0)、(0,-1,0)、(0,-1,0)。
tool0 与 J4/link4 重合，tcp 相对 tool0 平移 (0.092,0,0.020) m。
这两个 TCP 尺寸均为 **provisional**，不是最终标定值。

下一阶段须提供各关节机械上下限、允许速度/力矩、实物零位与电机正方向、
精确 TCP 位置与朝向。动力学工作还需要逐连杆质量、质心和惯量，精确碰撞建模需要外形/CAD。
若进入真实控制阶段，另需电机/驱动器型号、协议、减速比、编码器换算、回零和安全停机要求。

当前环境验收记录见 [validation_results.md](docs/validation_results.md)。
