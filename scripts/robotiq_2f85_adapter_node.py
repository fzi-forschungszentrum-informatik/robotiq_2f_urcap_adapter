#!/usr/bin/env python3
"""Module containing adapters for the Robotq_2f85 offering direct gripper <-> ROS interface."""

# Copyright (c) 2022 FZI Forschungszentrum Informatik
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.


import rclpy
from rclpy.action import ActionServer
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from sensor_msgs.msg import JointState

from robotiq_2f85_urcap_adapter.action import MoveGripper

from robotiq_2f85_urcap_adapter.robotiq_2f85_socket_adapter import ObjectStatus
from robotiq_2f85_urcap_adapter.robotiq_2f85_socket_adapter import Robotiq2f85SocketAdapter


class Robotiq2f85AdapterNode(Node):
    """ROS node offering ROS actions to control a Robotq2f85 gripper using string commands."""

    def __init__(self):
        super().__init__("robotiq_2f86_urcap_adapter")

        self.get_logger().info("Gripper control via urcap setting up!")

        self._action_server = ActionServer(
            self, MoveGripper, "~/move_gripper", self.execute_callback
        )

        self.declare_parameter("robot_ip", "192.168.0.104")
        self.declare_parameter("robot_port", 63352)
        self.declare_parameter("publish_fake_joint_values", False)

        robot_ip: str = self.get_parameter("robot_ip").value
        robot_port: int = self.get_parameter("robot_port").value

        self.get_logger().info(f"Connecting to URCAP on {robot_ip}:{robot_port}!")

        self.gripper_adapter: Robotiq2f85SocketAdapter = Robotiq2f85SocketAdapter()
        self.gripper_adapter.connect(hostname=robot_ip, port=robot_port)

        self.get_logger().info(f"Connected to URCAP on {robot_ip}:{robot_port}!")

        publish_fake_joint_values = self.get_parameter("publish_fake_joint_values").value

        self.joint_value = 0.0
        self.velocity = 0.5
        if publish_fake_joint_values:
            self.joint_val_pub = self.create_publisher(JointState, "joint_states", 10)
            self.timer_period = 1.0 / 50
            self.timer = self.create_timer(self.timer_period, self.publish_joint_values)

    def disconnect(self):
        """Disconnect from the URCAP socket."""
        self.get_logger().info("Disconnecting from URCAP!")
        self.gripper_adapter.disconnect()

    def __move_gripper_to_position(self, goal_handle, position: int) -> MoveGripper.Result:
        if self.gripper_adapter.current_position == position:
            goal_handle.succeed()
            return MoveGripper.Result(
                success=True
            )

        last_position, object_status = self.gripper_adapter.move_and_wait_for_pos(
            position=position,
            speed=self.gripper_adapter.max_speed,
            force=self.gripper_adapter.max_force
        )

        if object_status == ObjectStatus.AT_DEST:
            goal_handle.succeed()
            return MoveGripper.Result(
                success=True
            )

        if object_status in {
                ObjectStatus.MOVING,
                ObjectStatus.STOPPED_INNER_OBJECT,
                ObjectStatus.STOPPED_OUTER_OBJECT
        }:
            goal_handle.abort()
            return MoveGripper.Result(
                success=False,
                message=f"Failed to close gripper at position: {last_position} {object_status}"
            )
        goal_handle.abort()
        return MoveGripper.Result(
            success=False, message=f"Invalid status returned by gripper {object_status}"
        )

    def execute_callback(self, goal_handle):
        """
        Run the callback to move the robotiq gripper with ros actions.

        :param goal_handle: ROS action server goal handle.
        :return MoveGripper.Result containing the result of the execution.
        """
        if goal_handle.request.mode is MoveGripper.Goal.OPEN_GRIPPER:
            self.get_logger().info("Opening gripper via modbus")
            return self.__move_gripper_to_position(
                goal_handle=goal_handle,
                position=self.gripper_adapter.open_position
            )

        if goal_handle.request.mode is MoveGripper.Goal.CLOSE_GRIPPER:
            self.get_logger().info("Closing gripper via modbus")
            return self.__move_gripper_to_position(
                goal_handle=goal_handle,
                position=self.gripper_adapter.closed_position
            )

        goal_handle.abort()
        return MoveGripper.Result(
            success=False, message="Invalid mode {}".format(goal_handle.request.mode)
        )

    def publish_joint_values(self):
        """Publish the current joint values of the finger_joint at regular intervals."""
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = ["finger_joint"]
        msg.position = [self.gripper_adapter.current_position]
        msg.velocity = [0.1]
        msg.effort = [0.0]
        self.joint_val_pub.publish(msg)


def main(args=None):
    """
    Run the main method for the Robotiq2f85AdapterNode standalone node.

    :param: Arguments passed to the rclpy init call.
    """
    rclpy.init(args=args)

    gripper_control_adapter = Robotiq2f85AdapterNode()

    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(gripper_control_adapter)

    try:
        executor.spin()
    finally:
        executor.shutdown()
        gripper_control_adapter.disconnect()
        gripper_control_adapter.destroy_node()

    rclpy.shutdown()


if __name__ == "__main__":
    main()
