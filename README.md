# Robotiq 2F URCap Adapter

This is a direct control interface to the Robotiq *2F gripper URCap* using a socket connection and string commands.
The used string interface is defined
[here](https://dof.robotiq.com/discussion/2420/control-robotiq-gripper-mounted-on-ur-robot-via-socket-communication-python).
For communicating with that interface we use work from SDU's `ur_rtde`
[repository](https://gitlab.com/sdurobotics/ur_rtde/blob/master/doc/_static/robotiq_gripper.py)
licensed under the MIT license.

The overall idea is as follows:

1) Mount the gripper to the UR robot of your choice and connect it via
Robotiq's recommended way. There seem to be two options: Directly over the
end-effector for URe versions, and over UR's control box for the other
versions.

2) Use Robotiq's URCap to interface the gripper from UR's Polyscope. This
URCap seems to be better maintained by Robotiq than ROS packages and should cover all of the
gripper's features. You should be able to get the URCap
[here](https://robotiq.com/products/2f85-140-adaptive-robot-gripper?ref=nav_product_new_button). The
[quick start
guide](https://blog.robotiq.com/hubfs/Support%20Documents/QSG/Quick_start_2Finger_e-Series_nocropmarks_EN.pdf)
should help you getting started quickly.

3) To allow the ROS2 package to properly control the gripper, some of the hardware specs need to specified via ROS parameters.
These include the grip width (in Meters; `max_gripper_width`, `min_gripper_width`), the speed (in Meters / s; `max_gripper_speed`, `min_gripper_speed`) and the effort (in Newton; `max_gripper_force` `min_gripper_force`).
The user also has to set the IP address (`robot_ip`) of the robot.

4) When starting the `robotiq_2f_urcap_adapter` node the gripper will activate, if not already activated, and auto-calibrate its movement range.

5) The gripper can then be controlled using ROS actions using the `robotiq_2f_urcap_adapter/GripperControl` action.

## Parameters
The node currently has the following parameters:

- **`robot_ip`**:
  The robot's IP address
  (*default: "192.168.0.104"*)
- **`robot_port`**:
  The port where the URCap interface is running on the robot. This should usually not be changed.
  (*default: 63352*)
- **`max_gripper_width`**:
  The maximum opening width of a gripper in meters. Change this if your have a 2F-140 gripper.
  (*default: 0.085*)
- **`min_gripper_width`**:
  The minimum opening width of a gripper in meters.
  (*default: 0.0*)
- **`max_gripper_speed`**:
  The maximum speed of a gripper in meters/sec. See the
  [specification](https://robotiq.com/products/2f85-140-adaptive-robot-gripper?ref=nav_product_new_button)
  for details on the value range. Requested speeds higher than this will be rejected.
  (*default: 0.15*)
- **`min_gripper_speed`**:
  The minimum speed of a gripper in meters/sec. See the
  [specification](https://robotiq.com/products/2f85-140-adaptive-robot-gripper?ref=nav_product_new_button)
  for details on the value range. Requested speeds lower than this will be rejected.
  (*default: 0.02*)
- **`max_gripper_force`**:
  The maximum force of a gripper in Newtons. See the
  [specification](https://robotiq.com/products/2f85-140-adaptive-robot-gripper?ref=nav_product_new_button)
  for details on the value range. Requested forces higher than this will be rejected.
  (*default: 235*)
- **`min_gripper_force`**:
  The minimum force of a gripper in Newtons. See the
  [specification](https://robotiq.com/products/2f85-140-adaptive-robot-gripper?ref=nav_product_new_button)
  for details on the value range. Requested forces lower than this will be rejected.
  (*default: 20*)

## Example
Start the node:

```
ros2 run robotiq_2f_urcap_adapter robotiq_2f_adapter_node.py --ros-args -p robot_ip:=192.168.0.4
```

Close the gripper:
```
ros2 action send_goal -f /robotiq_2f_urcap_adapter/gripper_command robotiq_2f_urcap_adapter/GripperCommand '{ command: { position: 0.0, max_effort: 140, max_speed: 0.15 }}
```

Open the gripper (assuming it's a 2F-85):
```
ros2 action send_goal -f /robotiq_2f_urcap_adapter/gripper_command robotiq_2f_urcap_adapter/GripperCommand '{ command: { position: 0.085, max_effort: 140, max_speed: 0.15 }}'
```
