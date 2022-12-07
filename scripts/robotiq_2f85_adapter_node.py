#!/usr/bin/env python3
"""Module containing adapters for the Robotq_2f85 offering direct gripper <-> ROS interface."""

# Copyright (c) 2022 FZI Forschungszentrum Informatik
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.

import socket
import threading
import time
from turtle import speed
import typing
from enum import Enum
from collections import OrderedDict
from typing import Optional, Union, Tuple
from unicodedata import name

import rclpy
from rclpy.action import ActionServer
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node, ParameterDescriptor, ParameterType,\
    ParameterNotDeclaredException, ParameterUninitializedException
from sensor_msgs.msg import JointState

from robotiq_2f85_urcap_adapter.action import GripperCommand as GripperCommandAction
from robotiq_2f85_urcap_adapter.msg import GripperCommand as GripperCommandMsg

from robotiq_2f85_urcap_adapter.robotiq_2f85_socket_adapter import ObjectStatus
from robotiq_2f85_urcap_adapter.robotiq_2f85_socket_adapter import Robotiq2f85SocketAdapter

M_S_TO_MM_S = 1000
MM_S_TO_M_S = 0.001
M_TO_MM = 1000
MM_TO_M = 0.001

class Robotiq2f85AdapterNode(Node):
    """ROS node offering ROS actions to control a Robotq2f85 gripper using string commands."""

    def __init__(self):
        super().__init__("robotiq_2f86_urcap_adapter")

        self.get_logger().info("Gripper control via urcap setting up!")

        self._action_server = ActionServer(
            self,
            GripperCommandAction,
            "~/gripper_command",
            self.execute_callback
        )

        self.declare_parameter(
            name="robot_ip",
            value="192.168.0.104",
            descriptor=ParameterDescriptor(
                name="robot_ip",
                type=ParameterType.PARAMETER_STRING,
                description="IP address of the URCAP server controlling the gripper."
            )
        )
        self.declare_parameter(
            name="robot_port",
            value=63352,
            descriptor=ParameterDescriptor(
                name="robot_port",
                type=ParameterType.PARAMETER_INTEGER,
                description="Port for the URCAP server controlling the gripper."
            )
        )
        self.declare_parameter(
            name="robot_max_stroke",
            value=85.0,
            descriptor=ParameterDescriptor(
                name="robot_max_stroke",
                type=ParameterType.PARAMETER_DOUBLE,
                description="Maximum stroke length (mm) the gripper can open to. Used for normalization."
            )
        )
        self.declare_parameter(
            name="robot_min_stroke",
            value=0.0,
            descriptor=ParameterDescriptor(
                name="robot_min_stroke",
                type=ParameterType.PARAMETER_DOUBLE,
                description="Minimum stroke length (mm) the gripper can close to. Used for normalization."
            )
        )
        self.declare_parameter(
            name="robot_max_speed",
            value=150.0,
            descriptor=ParameterDescriptor(
                name="robot_max_speed",
                type=ParameterType.PARAMETER_DOUBLE,
                description="Maximum speed (mm/2) at which the gripper can move. Used for normalization."
            )
        )
        self.declare_parameter(
            name="robot_min_speed",
            value=20.0,
            descriptor=ParameterDescriptor(
                name="robot_min_speed",
                type=ParameterType.PARAMETER_DOUBLE,
                description="Minimum speed (mm/2) at which the gripper can move. Used for normalization."
            )
        )
        self.declare_parameter(
            name="robot_max_effort",
            value=235.0,
            descriptor=ParameterDescriptor(
                name="robot_max_effort",
                type=ParameterType.PARAMETER_DOUBLE,
                description="Maximum effort (N) which the gripper excert. Used for normalization."
            )
        )
        self.declare_parameter(
            name="robot_min_effort",
            value=20.0,
            descriptor=ParameterDescriptor(
                name="robot_max_effort",
                type=ParameterType.PARAMETER_DOUBLE,
                description="Minimum effort (N) which the gripper excert. Used for normalization."
            )
        )

        try:

            robot_ip: Optional[str] = self.get_parameter("robot_ip").value
            if robot_ip is None:
                raise ParameterUninitializedException(parameter_name="robot_ip")

            robot_port: Optional[int] = self.get_parameter("robot_port").value
            if robot_port is None:
                raise ParameterUninitializedException(parameter_name="robot_port")

            robot_max_stroke_mm: Optional[float] = self.get_parameter("robot_max_stroke").value
            if robot_max_stroke_mm is None:
                raise ParameterUninitializedException(parameter_name="robot_max_stroke")

            robot_min_stroke_mm: Optional[float] = self.get_parameter("robot_min_stroke").value
            if robot_min_stroke_mm is None:
                raise ParameterUninitializedException(parameter_name="robot_min_stroke")

            robot_max_speed_mm_s: Optional[float] = self.get_parameter("robot_max_speed").value
            if robot_max_speed_mm_s is None:
                raise ParameterUninitializedException(parameter_name="robot_max_speed")

            robot_min_speed_mm_s: Optional[float] = self.get_parameter("robot_min_speed").value
            if robot_min_speed_mm_s is None:
                raise ParameterUninitializedException(parameter_name="robot_min_speed")

            robot_max_effort_N: Optional[float] = self.get_parameter("robot_max_effort").value
            if robot_max_effort_N is None:
                raise ParameterUninitializedException(parameter_name="robot_max_effort")

            robot_min_effort_N: Optional[float] = self.get_parameter("robot_min_effort").value
            if robot_min_effort_N is None:
                raise ParameterUninitializedException(parameter_name="robot_min_effort")

        except ParameterNotDeclaredException as exc:
            self.get_logger().error(f"Parameter not declated: {exc}")
            raise RuntimeError() from exc
        except ParameterUninitializedException as exc:
            self.get_logger().error(f"Parameter uninitialized: {exc}")
            raise RuntimeError() from exc\

        self.get_logger().info(f"Connecting to URCAP on {robot_ip}:{robot_port}!")

        self.gripper_adapter: Robotiq2f85SocketAdapter = Robotiq2f85SocketAdapter()
        self.gripper_adapter.connect(hostname=robot_ip, port=robot_port)

        self.get_logger().info(f"Connected to URCAP on {robot_ip}:{robot_port}!")

        self._normalized_stroke_factor = (robot_max_stroke_mm - robot_min_stroke_mm) / 255
        self._normalized_stroke_baseline = robot_min_stroke_mm

        self._normalized_effort_factor = (robot_max_effort_N - robot_min_effort_N) / 255
        self._normalized_effort_baseline = robot_min_effort_N

        self._normalized_speed_factor = (robot_max_speed_mm_s - robot_min_speed_mm_s) / 255
        self._normalized_speed_baseline = robot_min_speed_mm_s

        self.joint_value = 0.0
        self.velocity = 0.5

    def __newton_from_normalized_effort_value(self, normalized_value: int) -> float:
        """Calculate a effort value in Newton for a normalized effort value [0-255].

        The calculation is dependent on the effort limits of the robot.

        :param normalized_value: The normalized effort value that should be transfered into a
        Newton value.
        :type normalized_value: int
        :raises ValueError: If the provided normalized_value is not within the bounds of 0-255,
        this error is raised.
        :return: The effort in Newton.
        :rtype: float
        """
        if normalized_value < 0 or normalized_value > 255:
            raise ValueError("Normalized value should be in the range of [0, 255]")
        return (normalized_value * self._normalized_effort_factor) \
            + self._normalized_effort_baseline

    def __normalized_effort_value_from_newton(self, newton_effort: float) -> int:
        """Calculate a normalized effort value [0-255] from a effort specification in Newton.

        This calculation is dependent on the effort limits of the robot.

        :param newton_effort: The newton effort that should be normalized into the range of 0-255.
        :type newton_effort: float
        :raises ValueError: If the specified newton_effort exceeds the limits of the robot, this
        error is raised.
        :return: The provided effort value normalized to the range of 0-255.
        :rtype: int
        """
        if newton_effort < self.__newton_from_normalized_effort_value(0) \
            or newton_effort > self.__newton_from_normalized_effort_value(255):
            raise ValueError(
                "Provided effort in newton exceeds limits of the gripper."
                "Validate that the provided force in within the specs of the gripper!"
                )

        return int(
            (newton_effort - self._normalized_effort_baseline) / self._normalized_effort_factor
            )

    def __mm_s_from_normalized_speed_value(self, normalized_value: int) -> float:
        """Calculate a mm/s speed value from the normalized speed values.

        This calculation is dependent on the maximum and minimum speed values of the robot.

        :param normalized_value: The speed value normalized to the range of 0-255 dependent on the
        robots speed limits.
        :type normalized_value: int
        :raises ValueError: If the provided normalized speed value is out of the range of 0-255,
        this error is raised.
        :return: The provided speed value in mm/s depending on the robot speed limits.
        :rtype: float
        """        """"""
        if normalized_value < 0 or normalized_value > 255:
            raise ValueError("Normalized value should be in the range of [0, 255]")
        return (normalized_value * self._normalized_speed_factor) \
            + self._normalized_speed_baseline

    def __normalized_speed_value_from_mm_s(self, mm_s_speed: float) -> int:
        """Calculate a normalized speed value [0-255] from mm/s speed values.

        This calculation is dependent on the maximum and minimum speed values of the robot.

        :param mm_s_speed: The speed value to be transformed into a normalized format.
        This value should be within the maximum and minimum values for the gripper.
        :type mm_s_speed: float
        :raises ValueError: If the provided mm/s speed value is not within the limits of the gripper
        this error is raised.
        :return: The normalized speed value [0-255] calculated from the mm/s input parameter.
        :rtype: int
        """
        if mm_s_speed < self.__mm_s_from_normalized_speed_value(0) \
            or mm_s_speed > self.__mm_s_from_normalized_speed_value(255):
            raise ValueError(
                "Provided speed in mm/s exceeds limits of the gripper."
                "Validate that the provided speed in within the specs of the gripper!"
                )

        return int(
            (mm_s_speed - self._normalized_speed_baseline) / self._normalized_speed_factor
            )


    def __mm_position_value_from_normalized_position(self, normalized_position: int) -> float:
        if normalized_position < 0 or normalized_position > 255:
            raise ValueError("Normalized position should be in the range of [0, 255]")
        return int((normalized_position * self._normalized_stroke_factor) \
            + self._normalized_stroke_baseline)

    def __normalized_position_value_from_mm(self, mm: float) -> int:
        if mm < self.__mm_position_value_from_normalized_position(0) \
            or mm > self.__mm_position_value_from_normalized_position(255):
            raise ValueError(
                "Provided speed in mm exceeds limits of the gripper."
                "Validate that the provided speed in within the specs of the gripper!"
                )

        return int(
            (mm - self._normalized_stroke_baseline) / self._normalized_stroke_factor
            )

    def disconnect(self):
        """Disconnect from the URCAP socket."""
        self.get_logger().info("Disconnecting from URCAP!")
        self.gripper_adapter.disconnect()

    def __move_gripper_to_position(self, goal_handle,
                                   position_mm: float,
                                   max_effort_N: float,
                                   max_speed_mm_s: float
                                   ) -> GripperCommandAction.Result:
        target_position_normalized = self.__normalized_position_value_from_mm(position_mm)
        if self.gripper_adapter.position == target_position_normalized:
            goal_handle.succeed()
            return GripperCommandAction.Result(
                position=position_mm,
                effort=0,
                speed=0,
                stalled=False,
                reached_goal=True
            )

        set_ok, cmd_pos  = self.gripper_adapter.move(
            position=target_position_normalized,
            speed=self.__normalized_speed_value_from_mm_s(max_speed_mm_s),
            force=self.__normalized_effort_value_from_newton(max_effort_N)
        )
        if not set_ok:
            return GripperCommandAction.Result(
                position=self.gripper_adapter.position,
                effort=0,
                speed=0,
                stalled=False,
                reached_goal=False
            )

        while self.gripper_adapter.__get_gripper_variable(self.gripper_adapter.PRE) != cmd_pos:
            goal_handle.publish_feedback(
                GripperCommandAction.Feedback(
                    position=self.gripper_adapter.position,
                    effort=0,
                    speed=0,
                    stalled=False,
                    reached_goal=False
                )
            )
            time.sleep(0.001)

        # wait until not moving
        object_status = ObjectStatus(
                self.gripper_adapter.__get_gripper_variable(self.gripper_adapter.OBJ)
                )
        while object_status == ObjectStatus.MOVING:
            object_status = ObjectStatus(
                self.gripper_adapter.__get_gripper_variable(self.gripper_adapter.OBJ)
                )
            goal_handle.publish_feedback(
                GripperCommandAction.Feedback(
                    position=self.gripper_adapter.position,
                    effort=self.gripper_adapter.force,
                    speed=self.gripper_adapter.speed,
                    stalled=False,
                    reached_goal=False
                )
            )

        if object_status == ObjectStatus.AT_DEST:
            goal_handle.succeed()
            return GripperCommandAction.Result(
                position=self.gripper_adapter.position,
                effort=self.gripper_adapter.force,
                speed=self.gripper_adapter.speed,
                stalled=False,
                reached_goal=True
            )

        if object_status in {
                ObjectStatus.MOVING,
                ObjectStatus.STOPPED_INNER_OBJECT,
                ObjectStatus.STOPPED_OUTER_OBJECT
        }:
            goal_handle.abort()

            return GripperCommandAction.Result(
                position=self.gripper_adapter.position,
                effort=self.gripper_adapter.force,
                speed=self.gripper_adapter.speed,
                stalled=True,
                reached_goal=False
            )
        goal_handle.abort()
        return GripperCommandAction.Result(
                position=self.gripper_adapter.position,
                effort=0,
                speed=0,
                stalled=False,
                reached_goal=False
            )

    def execute_callback(self, goal_handle):
        """
        Run the callback to move the robotiq gripper with ros actions.

        :param goal_handle: ROS action server goal handle.
        :return MoveGripper.Result containing the result of the execution.
        """
        goal: GripperCommandAction.Goal = goal_handle.request


        return self.__move_gripper_to_position(
            goal_handle=goal_handle,
            position_mm=goal.command.position * M_TO_MM,
            max_effort_N=goal.command.max_effort,
            max_speed_mm_s=goal.command.max_speed * M_S_TO_MM_S
        )

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
