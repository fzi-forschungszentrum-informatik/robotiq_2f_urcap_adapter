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

import socket
import threading
import time
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

from robotiq_2f85_urcap_adapter.action import MoveGripper


class GripperStatus(Enum):
    """Gripper status reported by the gripper."""

    RESET = 0
    ACTIVATING = 1
    # UNUSED = 2  # This value is currently not used by the gripper firmware
    ACTIVE = 3


class ObjectStatus(Enum):
    """Object status reported by the gripper."""

    MOVING = 0
    STOPPED_OUTER_OBJECT = 1
    STOPPED_INNER_OBJECT = 2
    AT_DEST = 3


def clip_val(min_value: int, value: int, max_value: int) -> int:
    """
    Clips a value to the bounds specified by both arguments.

    :param min_value: Lowest value returned
    :param value: Value that should be clipped to the bounds.
    :param max_value: Maximum value returned

    :return Clipped value
    """
    return max(min_value, min(value, max_value))


class Robotiq2f85AdapterNode(Node):
    """ROS node offering ROS actions to control a Robotq2f85 gripper using string commands."""

    def __init__(self):
        super().__init__("robotiq_2f86_urcap_adapter")

        self.get_logger().info("Gripper control via urcap setting up!")

        self._action_server = ActionServer(
            self, MoveGripper, "~/move_gripper", self.execute_callback
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
            value=502,
            descriptor=ParameterDescriptor(
                name="robot_port",
                type=ParameterType.PARAMETER_INTEGER,
                description="Port for the URCAP server controlling the gripper."
            )
        )
        self.declare_parameter(
            name="publish_fake_joint_values",
            value=True,
            descriptor=ParameterDescriptor(
                name="publish_fake_joint_values",
                type=ParameterType.PARAMETER_BOOL,
                description="Publish joint value estimates depending on the current gripper pose."
            )
        )
        self.declare_parameter(
            name="robot_max_speed",
            value=150.0,
            descriptor=ParameterDescriptor(
                name="robot_max_speed",
                type=ParameterType.PARAMETER_DOUBLE,
                description="Maximum speed at which the gripper can move. Used for normalization."
            )
        )
        self.declare_parameter(
            name="robot_min_speed",
            value=20.0,
            descriptor=ParameterDescriptor(
                name="robot_min_speed",
                type=ParameterType.PARAMETER_DOUBLE,
                description="Minimum speed at which the gripper can move. Used for normalization."
            )
        )
        self.declare_parameter(
            name="robot_max_effort",
            value=235.0,
            descriptor=ParameterDescriptor(
                name="robot_max_effort",
                type=ParameterType.PARAMETER_DOUBLE,
                description="Maximum effort which the gripper excert. Used for normalization."
            )
        )
        self.declare_parameter(
            name="robot_min_effort",
            value=20.0,
            descriptor=ParameterDescriptor(
                name="robot_max_effort",
                type=ParameterType.PARAMETER_DOUBLE,
                description="Minimum effort which the gripper excert. Used for normalization."
            )
        )

        try:

            robot_ip: Optional[str] = self.get_parameter("robot_ip").value
            if robot_ip is None:
                raise ParameterUninitializedException(parameter_name="robot_ip")

            robot_port: Optional[int] = self.get_parameter("robot_port").value
            if robot_port is None:
                raise ParameterUninitializedException(parameter_name="robot_port")

            publish_fake_joint_values: Optional[bool] = self.get_parameter(
                "publish_fake_joint_values"
                ).value
            if publish_fake_joint_values is None:
                raise ParameterUninitializedException(parameter_name="publish_fake_joint_values")

            robot_max_speed: Optional[float] = self.get_parameter("robot_max_speed").value
            if robot_max_speed is None:
                raise ParameterUninitializedException(parameter_name="robot_max_speed")

            robot_min_speed: Optional[float] = self.get_parameter("robot_min_speed").value
            if robot_min_speed is None:
                raise ParameterUninitializedException(parameter_name="robot_min_speed")

            robot_max_effort: Optional[float] = self.get_parameter("robot_max_effort").value
            if robot_max_effort is None:
                raise ParameterUninitializedException(parameter_name="robot_max_effort")

            robot_min_effort: Optional[float] = self.get_parameter("robot_min_effort").value
            if robot_min_effort is None:
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

        self._normalized_effort_factor = (robot_max_effort - robot_min_effort) / 255
        self._normalized_effort_baseline = robot_min_effort

        self._normalized_speed_factor = (robot_max_speed - robot_min_speed) / 255
        self._normalized_speed_baseline = robot_min_speed

        self.joint_value = 0.0
        self.velocity = 0.5
        if publish_fake_joint_values:
            self.joint_val_pub = self.create_publisher(JointState, "joint_states", 10)
            self.timer_period = 1.0 / 50
            self.timer = self.create_timer(self.timer_period, self.publish_joint_values)

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


class Robotiq2f85SocketAdapter:
    """Communicates with the gripper directly via socket with string commands."""

    __SET_COMMAND = 'SET'
    __GET_COMMAND = 'GET'
    __ACK_MESSAGE = 'ack'

    ACT = 'ACT'
    GTO = 'GTO'
    ATR = 'ATR'
    ADR = 'ADR'
    FOR = 'FOR'
    SPE = 'SPE'
    POS = 'POS'
    STA = 'STA'
    PRE = 'PRE'
    OBJ = 'OBJ'
    FLT = 'FLT'

    ENCODING = 'UTF-8'

    def __init__(self):
        self.socket: typing.Optional[socket.socket] = None
        self.socket_lock = threading.Lock()
        self._min_position: int = 0
        self._max_position: int = 255
        self._min_speed: int = 0
        self._max_speed: int = 255
        self._min_force: int = 0
        self._max_force: int = 255

    def connect(self, hostname: str, port: int, socket_timeout: float = 2.0) -> None:
        """
        Connect to the Robotiq2f85 gripper at the given address.

        :param hostname: Hostname or ip.
        :param port: Port.
        :param socket_timeout: Timeout for blocking socket operations.
        """
        with self.socket_lock:
            self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.socket.connect((hostname, port))
            self.socket.settimeout(socket_timeout)

    def disconnect(self) -> None:
        """Close the connection with the gripper."""
        with self.socket_lock:
            if self.socket is not None:
                self.socket.close()

    def __set_gripper_variables(self, variable_dict: typing.OrderedDict[str, Union[int, float]]):
        """
        Set the variables specified in the dict on the gripper via the socket connection.

        This command waits for the 'ack' response form the gripper.

        :param variable_dict: Dictionary of variables to set (variable_name, value).
        :return: True on successful reception of ack, false if no ack was received
        """
        cmd = f'{self.__SET_COMMAND}'
        for variable, value in variable_dict.items():
            cmd += f" {variable} {str(value)}"
        cmd += '\n'  # new line is required for the command to finish
        # atomic commands send/rcv
        with self.socket_lock:
            if self.socket is not None:
                self.socket.sendall(cmd.encode(self.ENCODING))
                data = self.socket.recv(1024)
                return self._is_ack(data)
            else:
                raise RuntimeError("Socket is none!")

    def __set_gripper_variable(self, variable: str, value: Union[int, float]):
        """
        Set the specified variable on the gripper via the socket connection.

        This command waits for the 'ack' response form the gripper.

        :param variable: Variable to set.
        :param value: Value to set for the variable.

        :return: True on successful reception of ack, false if no ack was received
        """
        return self.__set_gripper_variables(OrderedDict([(variable, value)]))

    def __get_gripper_variable(self, variable: str):
        """
        Get the specified variable value from the gripper using the socket connection.

        This command blocks until the response has been received or a timeout occurs.

        :param variable: Name of the variable to retrieve.
        :return: Value of the variable as integer.
        """
        # atomic commands send/rcv
        cmd = f"{self.__GET_COMMAND} {variable}\n"
        with self.socket_lock:
            if self.socket is not None:
                self.socket.sendall(cmd.encode(self.ENCODING))
                data = self.socket.recv(1024)
            else:
                raise RuntimeError("Socket is none!")

        var_name, value_str = data.decode(self.ENCODING).split()
        if var_name != variable:
            raise ValueError(
                f"Unexpected response {data} ({data.decode(self.ENCODING)}): "
                f"does not match '{variable}'"
                )
        value = int(value_str)
        return value

    def _is_ack(self, data: bytes) -> bool:
        """
        Check if received data is an acknowledgement message.

        :param data: bytes data received from the socket.

        :return if the data is an acknowledgement message.
        """
        return data.decode(self.ENCODING) == self.__ACK_MESSAGE

    def _reset(self):
        """Reset the gripper."""
        self.__set_gripper_variable(self.ACT, 0)
        self.__set_gripper_variable(self.ATR, 0)
        while (
            self.__get_gripper_variable(self.ACT) != 0 or
            self.__get_gripper_variable(self.STA) != 0
        ):
            self.__set_gripper_variable(self.ACT, 0)
            self.__set_gripper_variable(self.ATR, 0)
        time.sleep(0.5)

    def activate(self, auto_calibrate: bool = True):
        """
        Reset the activation flag in the gripper and clear previous fault flags.

        :param auto_calibrate: Whether to calibrate the
        minimum and maximum positions based on actual motion.
        """
        if not self.is_active:
            self._reset()
            while (
                self.__get_gripper_variable(self.ACT) != 0 or
                self.__get_gripper_variable(self.STA) != 0
            ):
                time.sleep(0.01)

            self.__set_gripper_variable(self.ACT, 1)
            time.sleep(1.0)
            while(
                self.__get_gripper_variable(self.ACT) != 1 or
                self.__get_gripper_variable(self.STA) != 3
            ):
                time.sleep(0.01)

        if auto_calibrate:
            self.auto_calibrate()

    @property
    def is_active(self):
        """Return whether the gripper is active."""
        status = self.__get_gripper_variable(self.STA)
        return GripperStatus(status) == GripperStatus.ACTIVE

    @property
    def max_force(self):
        """Return the maximum force the gripper can provide."""
        return self._max_force

    @property
    def min_force(self):
        """Return the minimum force the gripper can provide."""
        return self._min_force

    @property
    def max_speed(self):
        """Return the maximum force the gripper can move with."""
        return self._max_speed

    @property
    def min_speed(self):
        """Return the minimum force the gripper can move with."""
        return self._min_speed

    @property
    def min_position(self) -> int:
        """Return the minimum position the gripper can reach (open position)."""
        return self._min_position

    @property
    def max_position(self) -> int:
        """Return the maximum position the gripper can reach (closed position)."""
        return self._max_position

    @property
    def open_position(self) -> int:
        """Return what is considered the open position for gripper (minimum position value)."""
        return self.min_position

    @property
    def closed_position(self) -> int:
        """Return what is considered the closed position for gripper (maximum position value)."""
        return self.max_position

    @property
    def is_open(self):
        """Return whether the current position is considered as being fully open."""
        return self.current_position <= self.open_position

    @property
    def is_closed(self):
        """Return whether the current position is considered as being fully closed."""
        return self.current_position >= self.closed_position

    @property
    def current_position(self) -> int:
        """Return the current position as returned by the physical hardware."""
        return self.__get_gripper_variable(self.POS)

    def auto_calibrate(self, log: bool = True) -> None:
        """
        Attempt to calibrate the open and closed positions.

        This slowly closes and opens the gripper.

        :param log: Whether to print the results to log.
        """
        # first try to open in case we are holding an object
        (position, status) = self.move_and_wait_for_pos(self.open_position, 64, 1)
        if ObjectStatus(status) != ObjectStatus.AT_DEST:
            raise RuntimeError(f"Calibration failed opening to start: {str(status)}")

        # try to close as far as possible, and record the number
        (position, status) = self.move_and_wait_for_pos(self.closed_position, 64, 1)
        if ObjectStatus(status) != ObjectStatus.AT_DEST:
            raise RuntimeError(f"Calibration failed because of an object: {str(status)}")
        assert position <= self._max_position
        self._max_position = position

        # try to open as far as possible, and record the number
        (position, status) = self.move_and_wait_for_pos(self.open_position, 64, 1)
        if ObjectStatus(status) != ObjectStatus.AT_DEST:
            raise RuntimeError(f"Calibration failed because of an object: {str(status)}")
        assert position >= self._min_position
        self._min_position = position

        if log:
            print(f"Gripper auto-calibrated to [{self.min_position}, {self.max_position}]")

    def move(self, position: int, speed: int, force: int) -> Tuple[bool, int]:
        """
        Send commands to start moving towards the given position.

        :param position: Position to move to [min_position, max_position]
        :param speed: Speed to move at [min_speed, max_speed]
        :param force: Force to use [min_force, max_force]

        :return: A tuple with a bool indicating whether the action it was successfully sent,
        and an integer with the actual position that was requested,
        after being adjusted to the min/max calibrated range.
        """
        clip_pos = clip_val(self._min_position, position, self._max_position)
        clip_spe = clip_val(self._min_speed, speed, self._max_speed)
        clip_for = clip_val(self._min_force, force, self._max_force)

        # moves to the given position with the given speed and force
        var_dict = OrderedDict(
            [
                (self.POS, clip_pos),
                (self.SPE, clip_spe),
                (self.FOR, clip_for),
                (self.GTO, 1)
            ]
        )
        return self.__set_gripper_variables(var_dict), clip_pos

    def move_and_wait_for_pos(
        self,
        position: int,
        speed: int,
        force: int
    ) -> Tuple[int, ObjectStatus]:
        """
        Send commands to start moving towards the given position waiting for the move to complete.

        :param position: Position to move to [min_position, max_position]
        :param speed: Speed to move at [min_speed, max_speed]
        :param force: Force to use [min_force, max_force]

        :return: A tuple with an integer representing the last position returned by the gripper
        after it notified that the move had completed, a status indicating how the move ended.
        """
        set_ok, cmd_pos = self.move(position, speed, force)
        if not set_ok:
            raise RuntimeError("Failed to set variables for move.")

        # wait until the gripper acknowledges that it will try to go to the requested position
        while self.__get_gripper_variable(self.PRE) != cmd_pos:
            time.sleep(0.001)

        # wait until not moving
        cur_obj = self.__get_gripper_variable(self.OBJ)
        while ObjectStatus(cur_obj) == ObjectStatus.MOVING:
            cur_obj = self.__get_gripper_variable(self.OBJ)

        # report the actual position and the object status
        final_pos = self.__get_gripper_variable(self.POS)
        final_obj = cur_obj
        return final_pos, ObjectStatus(final_obj)


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
