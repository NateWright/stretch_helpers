# stretch_moveit_shim
Moveit support is very limited with stretch and is not supported on the real robot. In order to overcome this I have made a package that acts as a shim or interface that can be communicated with to make your code independent of the platform. One node is for moving the real robot and the other node is for gazebo. 

# Installation
```bash
cd ~/catkin_ws/src
git clone https://github.com/NateWright/stretch_helpers.git
sudo apt install ros-noetic-moveit-commander
cd ..
catkin build
source devel/setup.bash
```
# Launching
```bash
# Gazebo
rosrun stretch_moveit_shim stretch_interface_gazebo
# Real Robot
rosrun stretch_moveit_shim stretch_interface_real
```

# Example code
```python
#!/usr/bin/env python3

import sys
import rospy
from stretch_moveit_shim.srv import SetJoints, SetJointsRequest, SetBodyResponse
from stretch_moveit_shim.msg import Joint

rospy.wait_for_service('/stretch_interface/set_joints')

try:
    service = rospy.ServiceProxy('/stretch_interface/set_joints', SetJoints)
    msg = Joint(joint_name='gripper_aperture', val = 0.01)
    service([msg])
except rospy.ServiceException as e:
    print("Service call failed: %s"%e)
```
Available joints are
- joint_head_pan
- joint_head_tilt
- joint_lift
- wrist_extension
- joint_wrist_roll
- joint_wrist_pitch
- joint_wrist_yaw
- gripper_aperture

### TODO
- Test interface on real robot