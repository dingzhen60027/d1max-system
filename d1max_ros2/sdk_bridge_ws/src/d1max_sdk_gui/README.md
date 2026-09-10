# D1 Max SDK Qt control center

The GUI is a ROS 2 client of d1max_sdk_bridge. It never links to or calls the
proprietary SDK directly, so all commands remain subject to the bridge behavior
state machine and safety interlocks.

Features:

- SDK connection, RobotState, behavior FSM, fault and navigation-ready status
- guarded behavior services for control ownership, posture and motion modes
- explicit lock and unlock-to-stand workflow
- transition audit log that distinguishes service acceptance from completion
- prominent soft emergency stop and zero-velocity controls
- hold-to-run manual Twist publisher, disabled unless navigation is ready

Build:

    source /opt/ros/humble/setup.bash
    cd /home/dndx/智元四足机器人D1\ Max二次开发文档资料包v0.1.0/d1max_ros2/sdk_bridge_ws
    colcon build --symlink-install --packages-select d1max_sdk_bridge d1max_sdk_gui
    source install/setup.bash

Start bridge and GUI over wired Ethernet:

    ros2 launch d1max_sdk_gui sdk_control_center.launch.py

Wireless SDK endpoint:

    ros2 launch d1max_sdk_gui sdk_control_center.launch.py robot_ip:=192.168.234.1

Start only the GUI when the bridge is already running:

    ros2 launch d1max_sdk_gui sdk_gui.launch.py
