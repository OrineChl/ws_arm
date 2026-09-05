# 当前环境验收记录

日期：2026-09-05。工作区：`/home/sweeper/ws_arm`。

环境：Ubuntu 22.04.5，ROS_DISTRO=humble，与目标一致。
开始时及 source Humble 后，colcon、rviz2、check_urdf 均可用；xacro 命令和 Python 模块均不存在。
robot_state_publisher 已安装；joint_state_publisher 和 joint_state_publisher_gui 未在 ROS share 中找到。
未安装任何系统软件，未修改系统 ROS 环境，未提交 Git commit。

| 顺序 | 验收步骤 | 实际结果 |
| --- | --- | --- |
| 1 | source /opt/ros/humble/setup.bash | 成功，ROS_DISTRO=humble |
| 2 | colcon build --symlink-install | 最终成功，1 package finished，退出码 0 |
| 3 | source install/setup.bash | 成功，ros2 run/launch 可以找到本包 |
| 4 | xacro 展开后 check_urdf | xacro 不存在，退出码 127；未生成 URDF，check_urdf 验收阻塞 |
| 5 | ros2 run custom_4dof_description verify_zero_pose.py | 退出码 2，明确报告找不到 xacro；实际模型零位尚未验证 |
| 6 | ros2 launch custom_4dof_description display.launch.py | 退出码 1；launch 的 FindExecutable 无法找到 xacro，节点尚未启动 |

首次构建发生在 README 写入完成前，安装步骤报告 README 不存在；补齐全部要求文件后重新构建成功。
启动日志在工作区 `log/ros/` 下；运行时设置 ROS_LOG_DIR 到该目录。

补充检查：

- package.xml、两个 Xacro 的 XML 格式检查通过（不能替代 Xacro 展开和 URDF 语义检查）。
- launch 和验证脚本 Python 语法检查通过；参数文件及 RViz 配置 YAML 解析通过。
- 验证器使用独立合成 URDF 测试：正确零位返回 0；TCP 偏移 1 mm 返回 1；
  将肩部 origin 旋转 90° 后返回 1，均打印实际值和期望值。
  这些测试仅检查验证器，不是对实际 Xacro 模型的验收。

环境设置了 DISPLAY=:0 和 WAYLAND_DISPLAY=wayland-0，但本次因缺少依赖尚未到达图形启动阶段，
不能据此断言 RViz 可显示或模型失败。需要补齐依赖后在有桌面环境的 Ubuntu 22.04 / Humble
主机上重新执行 README 的完整验收命令，验证四个滑块、RobotModel 和 TF 树。

## 安装依赖后的复验（2026-09-05）

xacro、joint_state_publisher 和 joint_state_publisher_gui 现已可用。
真实 Xacro 展开成功，check_urdf 成功解析完整七连杆链，实际模型零位脚本全部 PASS。
在线检查发现 robot_state_publisher、joint_state_publisher、rviz 和 TF listener 均存在；
robot_description、tf、tf_static 的发布与 RViz 订阅 QoS 匹配。
/joint_states 正常提供四个零位关节，tf2_echo 连续读出 base_link → tcp 平移
(0.264, 0, 0.372) m、单位旋转。诊断用 tf2_echo 在 8 秒后由 timeout 停止。
用户截图中 RViz 仍报 No tf data，模型不可见，说明桌面显示验收尚未完成；
在线 TF 检查成功不能替代 RViz 显示验收。尚未确认 RViz 未接收数据的具体原因。
