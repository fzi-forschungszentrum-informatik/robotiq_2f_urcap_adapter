#!/usr/bin/env python3
"""Module containing a ROS node for Robotiq 2f grippers <-> ROS action communication."""

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

import time
from typing import Optional

import rclpy
from rclpy.action import ActionServer
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node, ParameterDescriptor, ParameterType,\
    ParameterNotDeclaredException, ParameterUninitializedException

from robotiq_2f_urcap_adapter.action import GripperCommand as GripperCommandAction

from robotiq_2f_urcap_adapter_socket.robotiq_2f_socket_adapter import ObjectStatus
from robotiq_2f_urcap_adapter_socket.robotiq_2f_socket_adapter import Robotiq2fSocketAdapter


class Robotiq2fAdapterNode(Node):
    """ROS node offering ROS actions to control a Robotq2f gripper using string commands."""

    def __init__(self):
        super().__init__("robotiq_2f_urcap_adapter")

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
            name="max_gripper_width",
            value=0.085,
            descriptor=ParameterDescriptor(
                name="max_gripper_width",
                type=ParameterType.PARAMETER_DOUBLE,
                description="Maximum gripper opening width (m)."
            )
        )
        self.declare_parameter(
            name="min_gripper_width",
            value=0.0,
            descriptor=ParameterDescriptor(
                name="min_gripper_width",
                type=ParameterType.PARAMETER_DOUBLE,
                description="Minimum gripper opening width (m)."
            )
        )
        self.declare_parameter(
            name="max_gripper_speed",
            value=0.15,
            descriptor=ParameterDescriptor(
                name="max_gripper_speed",
                type=ParameterType.PARAMETER_DOUBLE,
                description="Maximum speed (m/s) at which the gripper can move."
            )
        )
        self.declare_parameter(
            name="min_gripper_speed",
            value=0.02,
            descriptor=ParameterDescriptor(
                name="min_gripper_speed",
                type=ParameterType.PARAMETER_DOUBLE,
                description="Minimum speed (mm/s) at which the gripper can move."
            )
        )
        self.declare_parameter(
            name="max_gripper_force",
            value=235.0,
            descriptor=ParameterDescriptor(
                name="max_gripper_force",
                type=ParameterType.PARAMETER_DOUBLE,
                description="Maximum effort (N) which the gripper excert."
            )
        )
        self.declare_parameter(
            name="min_gripper_force",
            value=20.0,
            descriptor=ParameterDescriptor(
                name="max_gripper_force",
                type=ParameterType.PARAMETER_DOUBLE,
                description="Minimum effort (N) which the gripper excert."
            )
        )

        try:

            robot_ip: Optional[str] = self.get_parameter("robot_ip").value
            if robot_ip is None:
                raise ParameterUninitializedException(parameter_name="robot_ip")

            robot_port: Optional[int] = self.get_parameter("robot_port").value
            if robot_port is None:
                raise ParameterUninitializedException(parameter_name="robot_port")

            max_gripper_width_m: Optional[float] = \
                self.get_parameter("max_gripper_width").value
            if max_gripper_width_m is None:
                raise ParameterUninitializedException(parameter_name="max_gripper_width")

            min_gripper_width_m: Optional[float] = \
                self.get_parameter("min_gripper_width").value
            if min_gripper_width_m is None:
                raise ParameterUninitializedException(parameter_name="min_gripper_width")

            max_gripper_speed_m_s: Optional[float] = self.get_parameter("max_gripper_speed").value
            if max_gripper_speed_m_s is None:
                raise ParameterUninitializedException(parameter_name="max_gripper_speed")

            min_gripper_speed_m_s: Optional[float] = self.get_parameter("min_gripper_speed").value
            if min_gripper_speed_m_s is None:
                raise ParameterUninitializedException(parameter_name="min_gripper_speed")

            max_gripper_force_N: Optional[float] = self.get_parameter("max_gripper_force").value
            if max_gripper_force_N is None:
                raise ParameterUninitializedException(parameter_name="max_gripper_force")

            min_gripper_force_N: Optional[float] = self.get_parameter("min_gripper_force").value
            if min_gripper_force_N is None:
                raise ParameterUninitializedException(parameter_name="min_gripper_force")

        except ParameterNotDeclaredException as exc:
            self.get_logger().error(f"Parameter not declated: {exc}")
            raise RuntimeError() from exc
        except ParameterUninitializedException as exc:
            self.get_logger().error(f"Parameter uninitialized: {exc}")
            raise RuntimeError() from exc\

        self.get_logger().info(f"Connecting to URCAP on {robot_ip}:{robot_port}!")

        self.gripper_adapter: Robotiq2fSocketAdapter = Robotiq2fSocketAdapter()
        self.gripper_adapter.connect(hostname=robot_ip, port=robot_port)

        self.get_logger().info(f"Connected to URCAP on {robot_ip}:{robot_port}!")

        self.get_logger().info(f"Activate Gripper on {robot_ip}:{robot_port}!")

        self.gripper_adapter.activate(auto_calibrate=True)

        self.get_logger().info(f"Activated Gripper on {robot_ip}:{robot_port}!")

        self._normalized_grip_width_factor = \
            (max_gripper_width_m - min_gripper_width_m) / 255
        self._normalized_grip_width_baseline = max_gripper_width_m

        min_position_m = \
            self.__m_value_from_normalized_grip_width(self.gripper_adapter.min_position)
        max_position_m = \
            self.__m_value_from_normalized_grip_width(self.gripper_adapter.max_position)

        self.get_logger().info(
            f"Grip range reported by autocalibration: [{min_position_m}, {max_position_m}]"
            )

        self._normalized_effort_factor = (max_gripper_force_N - min_gripper_force_N) / 255
        self._normalized_effort_baseline = min_gripper_force_N

        self._normalized_speed_factor = (max_gripper_speed_m_s - min_gripper_speed_m_s) / 255
        self._normalized_speed_baseline = min_gripper_speed_m_s

    def __newton_value_from_normalized_effort(self, normalized_value: int) -> float:
        """
        Calculate a effort value in Newton for a normalized effort value [0-255].

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
        """
        Calculate a normalized effort value [0-255] from a effort specification in Newton.

        This calculation is dependent on the effort limits of the robot.

        :param newton_effort: The newton effort that should be normalized into the range of 0-255.
        :type newton_effort: float
        :raises ValueError: If the specified newton_effort exceeds the limits of the robot, this
        error is raised.
        :return: The provided effort value normalized to the range of 0-255.
        :rtype: int
        """
        if newton_effort < self.__newton_value_from_normalized_effort(0) \
                or newton_effort > self.__newton_value_from_normalized_effort(255):
            raise ValueError(
                "Provided effort in newton exceeds limits of the gripper."
                "Validate that the provided force in within the specs of the gripper!"
                )

        return int(
            (newton_effort - self._normalized_effort_baseline) / self._normalized_effort_factor
            )

    def __m_s_Value_from_normalized_speed(self, normalized_value: int) -> float:
        """
        Calculate a m/s speed value from the normalized speed values.

        This calculation is dependent on the maximum and minimum speed values of the robot.

        :param normalized_value: The speed value normalized to the range of 0-255 dependent on the
        robots speed limits.
        :type normalized_value: int
        :raises ValueError: If the provided normalized speed value is out of the range of 0-255,
        this error is raised.
        :return: The provided speed value in m/s depending on the robot speed limits.
        :rtype: float
        """
        if normalized_value < 0 or normalized_value > 255:
            raise ValueError("Normalized value should be in the range of [0, 255]")
        return (normalized_value * self._normalized_speed_factor) \
            + self._normalized_speed_baseline

    def __normalized_speed_value_from_m_s(self, m_s_speed: float) -> int:
        """
        Calculate a normalized speed value [0-255] from m/s speed values.

        This calculation is dependent on the maximum and minimum speed values of the robot.

        :param mm_s_speed: The speed value to be transformed into a normalized format.
        This value should be within the maximum and minimum values for the gripper.
        :type mm_s_speed: float
        :raises ValueError: If the provided m/s speed value is not within the limits of the gripper
        this error is raised.
        :return: The normalized speed value [0-255] calculated from the m/s input parameter.
        :rtype: int
        """
        if m_s_speed < self.__m_s_Value_from_normalized_speed(0) \
                or m_s_speed > self.__m_s_Value_from_normalized_speed(255):
            raise ValueError(
                f"Provided speed {m_s_speed} in m/s exceeds limits ["
                f"{self.__m_s_Value_from_normalized_speed(0)}, "
                f"{self.__m_s_Value_from_normalized_speed(255)}] "
                f"of the gripper. Validate that the provided speed"
                f"within the specs of the gripper!"
                )
        return int(
            (m_s_speed - self._normalized_speed_baseline) / self._normalized_speed_factor
            )

    def __m_value_from_normalized_grip_width(self, normalized_position: int) -> float:
        """
        Calculate the grip width in m from the normalized grip width values.

        This calculation is dependent on the maximum and minimum grip width values of the robot.

        :param normalized_value: The grip width normalized to the range of 0-255 dependent on the
        robots limits.
        :type normalized_value: int
        :raises ValueError: If the provided normalized grip width is out of the range of 0-255,
        this error is raised.
        :return: The grip width value in m.
        :rtype: float
        """
        if normalized_position < 0 or normalized_position > 255:
            raise ValueError("Normalized grip width should be in the range of [0, 255]")

        return float(
            self._normalized_grip_width_baseline -
            (normalized_position * self._normalized_grip_width_factor)
            )

    def __normalized_grip_width_value_from_m(self, m: float) -> int:
        """
        Calculate a normalized grip width [0-255] from grip width in m.

        This calculation is dependent on the maximum and minimum grip width of the robot.

        :param m: The grip width value to be transformed into a normalized format.
        This value should be within the maximum and minimum values for the gripper.
        :type m: float
        :raises ValueError: If the provided m grip width is not within the limits of the gripper
        this error is raised.
        :return: The normalized grip width value [0-255] calculated from the m input parameter.
        :rtype: int
        """
        if m < self.__m_value_from_normalized_grip_width(255) \
                or m > self.__m_value_from_normalized_grip_width(0):
            raise ValueError(
                f"Provided grip width {m} in m exceeds limits ["
                f"{self.__m_value_from_normalized_grip_width(255)}, "
                f"{self.__m_value_from_normalized_grip_width(0)}] "
                f"of the gripper. Validate that the provided grip width is"
                f"within the specs of the gripper!"
                )

        return int(
            (self._normalized_grip_width_baseline - m) / self._normalized_grip_width_factor
            )

    def disconnect(self):
        """Disconnect from the URCAP socket."""
        # self.get_logger().info("Disconnecting from URCAP!")
        self.gripper_adapter.disconnect()
        time.sleep(0.01)

    def __move_gripper_to_grip_width(self, goal_handle,
                                     grip_width_m: float,
                                     max_effort_N: float,
                                     max_speed_m_s: float
                                     ) -> GripperCommandAction.Result:
        try:
            target_position_normalized = self.__normalized_grip_width_value_from_m(grip_width_m)
            normalized_speed = self.__normalized_speed_value_from_m_s(max_speed_m_s)
            normalized_effort = self.__normalized_effort_value_from_newton(max_effort_N)
        except ValueError as exc:
            self.get_logger().error(
                f"Failed to convert goal values into normalized values for the gripper: {exc}"
                )
            goal_handle.abort()
            return GripperCommandAction.Result(
                position=self.__m_value_from_normalized_grip_width(self.gripper_adapter.position),
                effort=0.0,
                speed=0.0,
                stalled=False,
                reached_goal=False
            )

        if self.gripper_adapter.position == target_position_normalized:
            goal_handle.succeed()
            return GripperCommandAction.Result(
                position=grip_width_m,
                effort=0,
                speed=0,
                stalled=False,
                reached_goal=True
            )

        set_ok, cmd_pos = self.gripper_adapter.move(
            position=target_position_normalized,
            speed=normalized_speed,
            force=normalized_effort
        )
        if not set_ok:
            goal_handle.abort()
            return GripperCommandAction.Result(
                position=self.__m_value_from_normalized_grip_width(
                    self.gripper_adapter.position
                    ),
                effort=0.0,
                speed=0.0,
                stalled=False,
                reached_goal=False
            )

        prev_position = self.gripper_adapter.position
        while self.gripper_adapter.get_gripper_variable(self.gripper_adapter.PRE) != cmd_pos:
            position = self.gripper_adapter.position
            if prev_position != position:
                prev_position = position

                goal_handle.publish_feedback(
                    GripperCommandAction.Feedback(
                        position=self.__m_value_from_normalized_grip_width(prev_position),
                        effort=self.__newton_value_from_normalized_effort(
                            self.gripper_adapter.force
                            ),
                        speed=self.__m_s_Value_from_normalized_speed(self.gripper_adapter.speed),
                        stalled=False,
                        reached_goal=False
                    )
                )
            time.sleep(0.001)

        # wait until not moving
        object_status = ObjectStatus(
                self.gripper_adapter.get_gripper_variable(self.gripper_adapter.OBJ)
                )

        prev_position = self.gripper_adapter.position
        prev_effort = self.gripper_adapter.force
        prev_speed = self.gripper_adapter.speed

        while object_status == ObjectStatus.MOVING:
            object_status = ObjectStatus(
                self.gripper_adapter.get_gripper_variable(self.gripper_adapter.OBJ)
                )

            position = self.gripper_adapter.position
            effort = self.gripper_adapter.force
            speed = self.gripper_adapter.speed

            if (position != prev_position) or (effort != prev_effort) or (speed != prev_speed):
                prev_position = position
                prev_effort = effort
                prev_speed = speed

                goal_handle.publish_feedback(
                    GripperCommandAction.Feedback(
                        position=self.__m_value_from_normalized_grip_width(prev_position),
                        effort=self.__newton_value_from_normalized_effort(prev_effort),
                        speed=self.__m_s_Value_from_normalized_speed(prev_speed),
                        stalled=False,
                        reached_goal=False
                    )
                )

        if object_status == ObjectStatus.AT_DEST:
            goal_handle.succeed()
            return GripperCommandAction.Result(
                position=self.__m_value_from_normalized_grip_width(self.gripper_adapter.position),
                effort=self.__newton_value_from_normalized_effort(self.gripper_adapter.force),
                speed=self.__m_s_Value_from_normalized_speed(self.gripper_adapter.speed),
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
                position=self.__m_value_from_normalized_grip_width(self.gripper_adapter.position),
                effort=self.__newton_value_from_normalized_effort(self.gripper_adapter.force),
                speed=self.__m_s_Value_from_normalized_speed(self.gripper_adapter.speed),
                stalled=True,
                reached_goal=False
            )
        goal_handle.abort()
        return GripperCommandAction.Result(
                position=self.__m_value_from_normalized_grip_width(self.gripper_adapter.position),
                effort=self.__newton_value_from_normalized_effort(prev_effort),
                speed=self.__m_s_Value_from_normalized_speed(prev_speed),
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

        return self.__move_gripper_to_grip_width(
            goal_handle=goal_handle,
            grip_width_m=goal.command.position,
            max_effort_N=goal.command.max_effort,
            max_speed_m_s=goal.command.max_speed
        )


def main(args=None):
    """
    Run the main method for the Robotiq2fAdapterNode standalone node.

    :param: Arguments passed to the rclpy init call.
    """
    rclpy.init(args=args)

    gripper_control_adapter = Robotiq2fAdapterNode()

    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(gripper_control_adapter)

    try:
        executor.spin()
    finally:
        gripper_control_adapter.disconnect()
        gripper_control_adapter.destroy_node()
        executor.shutdown()

    rclpy.shutdown()


if __name__ == "__main__":
    main()
