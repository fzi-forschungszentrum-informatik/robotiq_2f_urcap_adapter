# Robotiq 2F-85 URCap Adapter

This is a direct control interface to the Robotiq *2F-85 gripper URCap* via Universal Robot's Modbus.

---

**Note**: The primary motivation for this package is a fast way of interfacing Robotiq's gripper in ROS2 until
a decent driver becomes available.

---

The overall idea is as follows:

1) Mount the gripper to the UR robot of your choice and connect it via
Robotiq's recommended way. There seem to be two options: Directly over the
end-effector for URe versions, and over UR's control box for the other
versions.

2) Use Robotiq's URCap to interface the gripper from UR's Polyscope. This
URCap seems to be better maintained by Robotiq than ROS packages and should cover all of the gripper's features.

3) UR robots expose a Modbus server, that is accessible on port `502` by default. You can write to their general
purpose registers over a TCP-based connection to the UR control box.
Use a lightweight ROS2 node to write to these registers to set values.
You choose both suitable registers and values by yourself.

3) Build a custom control program in UR's Polyscope that reads the Modbus
registers of your choice and reacts upon events with triggering Robotiq's URCap
functionalities, for which you design suitable presets.



## Prepare the URCap side

## Starting the ROS2 adapter
