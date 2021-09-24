"""
Module containing adapters for the Robotq_2f85 offering a direct interface between the gripper and ROS.
"""

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
from collections import OrderedDict
from enum import Enum
from typing import Union, Tuple

import rclpy
from rclpy.action import ActionServer
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from sensor_msgs.msg import JointState

from robotiq_2f85_urcap_adapter.action import MoveGripper


class GripperStatus(Enum):
    """Gripper status reported by the gripper. The integer values have to match what the gripper sends."""
    RESET = 0
    ACTIVATING = 1
    # UNUSED = 2  # This value is currently not used by the gripper firmware
    ACTIVE = 3


class ObjectStatus(Enum):
    """Object status reported by the gripper. The integer values have to match what the gripper sends."""
    MOVING = 0
    STOPPED_OUTER_OBJECT = 1
    STOPPED_INNER_OBJECT = 2
    AT_DEST = 3

class Robotiq2f85AdapterNode(Node):

    def __init__(self):
        super().__init__("robotiq_2f86_urcap_adapter")

        self._action_server = ActionServer(
            self, MoveGripper, "~/move_gripper", self.execute_callback
        )

        self.get_logger().info("Gripper control via modbus ready")

        self.declare_parameter("robot_ip", "192.168.0.104")
        self.declare_parameter("robot_port", 502)
        self.declare_parameter("publish_fake_joint_values", True)

        robot_ip = self.get_parameter("robot_ip").value
        robot_port = self.get_parameter("robot_port").value

        self.gripper_adapter = GripperAdapter()
        self.gripper_adapter.connect(hostname=robot_ip, port=robot_port)

        self.publish_fake_joint_values = self.get_parameter("publish_fake_joint_values").value

        self.joint_value = 0.0
        self.velocity = 0.5
        if self.publish_fake_joint_values:
            self.joint_val_pub = self.create_publisher(JointState, "joint_states", 10)
            self.timer_period = 1.0 / 50
            self.timer = self.create_timer(self.timer_period, self.publish_joint_values)

    def __del__(self):
        self.gripper_adapter.disconnect()

    def execute_callback(self, goal_handle):

        self.goal_handle = goal_handle
        if goal_handle.request.mode is MoveGripper.Goal.OPEN_GRIPPER:
            self.get_logger().info("Opening gripper via modbus")
            if self.gripper_adapter.is_open():
                goal_handle.succeed()
                return MoveGripper.Result(
                    success=True
                )

            last_position, object_status = self.gripper_adapter.move_and_wait_for_pos(
                position=self.gripper_adapter.get_open_position(),
                speed=self.gripper_adapter._max_speed,
                force=self.gripper_adapter._max_force
            )

            if object_status == ObjectStatus.AT_DEST:
                goal_handle.succeed()
                return MoveGripper.Result(
                    success=True
                )
            elif object_status in {
                    ObjectStatus.MOVING,
                    ObjectStatus.STOPPED_INNER_OBJECT,
                    ObjectStatus.STOPPED_OUTER_OBJECT
            }:
                goal_handle.abort()
                return MoveGripper.Result(
                    success=False,
                    message=f"Failed to close gripper at position: {last_position} {object_status}"
                )
            else:
                goal_handle.abort()
                return MoveGripper.Result(
                    success=False, message=f"Invalid status returned by gripper {object_status}"
                )

        elif goal_handle.request.mode is MoveGripper.Goal.CLOSE_GRIPPER:
            self.get_logger().info("Closing gripper via modbus")
            if self.gripper_adapter.is_closed():
                goal_handle.succeed()
                return MoveGripper.Result(
                    success=True
                )

            last_position, object_status = self.gripper_adapter.move_and_wait_for_pos(
                position=self.gripper_adapter.get_closed_position(),
                speed=self.gripper_adapter._max_speed,
                force=self.gripper_adapter._max_force
            )

            if object_status == ObjectStatus.AT_DEST:
                goal_handle.succeed()
                return MoveGripper.Result(
                    success=True
                )

            elif object_status in {
                    ObjectStatus.MOVING,
                    ObjectStatus.STOPPED_INNER_OBJECT,
                    ObjectStatus.STOPPED_OUTER_OBJECT
            }:
                goal_handle.abort()
                return MoveGripper.Result(
                    success=False,
                    message=f"Failed to close gripper at position: {last_position} {object_status}"
                )
            else:
                goal_handle.abort()
                return MoveGripper.Result(
                    success=False, message=f"Invalid status returned by gripper {object_status}"
                )


        else:
            goal_handle.abort()
            return MoveGripper.Result(
                success=False, message="Invalid mode {}".format(goal_handle.request.mode)
            )

    def publish_joint_values(self):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = ["finger_joint"]
        msg.position = [self.gripper_adapter.get_current_position()]
        msg.velocity = [0.1]
        msg.effort = [0.0]
        self.joint_val_pub.publish(msg)

class GripperAdapter:
    """
    Communicates with the gripper directly, via socket with string commands, leveraging string names for variables.
    """
    # WRITE VARIABLES (CAN ALSO READ)
    ACT = 'ACT'  # act : activate (1 while activated, can be reset to clear fault status)
    GTO = 'GTO'  # gto : go to (will perform go to with the actions set in pos, for, spe)
    ATR = 'ATR'  # atr : auto-release (emergency slow move)
    ADR = 'ADR'  # adr : auto-release direction (open(1) or close(0) during auto-release)
    FOR = 'FOR'  # for : force (0-255)
    SPE = 'SPE'  # spe : speed (0-255)
    POS = 'POS'  # pos : position (0-255), 0 = open
    # READ VARIABLES
    STA = 'STA'  # status (0 = is reset, 1 = activating, 3 = active)
    PRE = 'PRE'  # position request (echo of last commanded position)
    OBJ = 'OBJ'  # object detection (0 = moving, 1 = outer grip, 2 = inner grip, 3 = no object at rest)
    FLT = 'FLT'  # fault (0=ok, see manual for errors if not zero)

    ENCODING = 'UTF-8'  # ASCII and UTF-8 both seem to work

    def __init__(self):
        """Constructor."""
        self.socket = None
        self.command_lock = threading.Lock()
        self._min_position = 0
        self._max_position = 255
        self._min_speed = 0
        self._max_speed = 255
        self._min_force = 0
        self._max_force = 255

    def connect(self, hostname: str, port: int, socket_timeout: float = 2.0) -> None:
        """Connects to a gripper at the given address.
        :param hostname: Hostname or ip.
        :param port: Port.
        :param socket_timeout: Timeout for blocking socket operations.
        """
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.socket.connect((hostname, port))
        self.socket.settimeout(socket_timeout)

    def disconnect(self) -> None:
        """Closes the connection with the gripper."""
        self.socket.close()

    def _set_vars(self, var_dict: OrderedDict[str, Union[int, float]]):
        """Sends the appropriate command via socket to set the value of n variables, and waits for its 'ack' response.
        :param var_dict: Dictionary of variables to set (variable_name, value).
        :return: True on successful reception of ack, false if no ack was received, indicating the set may not
        have been effective.
        """
        # construct unique command
        cmd = "SET"
        for variable, value in var_dict.items():
            cmd += f" {variable} {str(value)}"
        cmd += '\n'  # new line is required for the command to finish
        # atomic commands send/rcv
        with self.command_lock:
            self.socket.sendall(cmd.encode(self.ENCODING))
            data = self.socket.recv(1024)
        return self._is_ack(data)

    def _set_var(self, variable: str, value: Union[int, float]):
        """Sends the appropriate command via socket to set the value of a variable, and waits for its 'ack' response.
        :param variable: Variable to set.
        :param value: Value to set for the variable.
        :return: True on successful reception of ack, false if no ack was received, indicating the set may not
        have been effective.
        """
        return self._set_vars(OrderedDict([(variable, value)]))

    def _get_var(self, variable: str):
        """Sends the appropriate command to retrieve the value of a variable from the gripper, blocking until the
        response is received or the socket times out.
        :param variable: Name of the variable to retrieve.
        :return: Value of the variable as integer.
        """
        # atomic commands send/rcv
        with self.command_lock:
            cmd = f"GET {variable}\n"
            self.socket.sendall(cmd.encode(self.ENCODING))
            data = self.socket.recv(1024)

        # expect data of the form 'VAR x', where VAR is an echo of the variable name, and X the value
        # note some special variables (like FLT) may send 2 bytes, instead of an integer. We assume integer here
        var_name, value_str = data.decode(self.ENCODING).split()
        if var_name != variable:
            raise ValueError(f"Unexpected response {data} ({data.decode(self.ENCODING)}): does not match '{variable}'")
        value = int(value_str)
        return value

    @staticmethod
    def _is_ack(data: str):
        return data == b'ack'


    def _reset(self):
        """
        Reset the gripper.
        The following code is executed in the corresponding script function
        def rq_reset(gripper_socket="1"):
            rq_set_var("ACT", 0, gripper_socket)
            rq_set_var("ATR", 0, gripper_socket)

            while(not rq_get_var("ACT", 1, gripper_socket) == 0 or not rq_get_var("STA", 1, gripper_socket) == 0):
                rq_set_var("ACT", 0, gripper_socket)
                rq_set_var("ATR", 0, gripper_socket)
                sync()
            end

            sleep(0.5)
        end
        """
        self._set_var(self.ACT, 0)
        self._set_var(self.ATR, 0)
        while self._get_var(self.ACT) != 0 or self._get_var(self.STA) != 0:
            self._set_var(self.ACT, 0)
            self._set_var(self.ATR, 0)
        time.sleep(0.5)

    def activate(self, auto_calibrate: bool = True):
        """Resets the activation flag in the gripper, and sets it back to one, clearing previous fault flags.
        :param auto_calibrate: Whether to calibrate the minimum and maximum positions based on actual motion.
        """
        if not self.is_active():
            self._reset()
            while self._get_var(self.ACT) != 0 or self._get_var(self.STA) != 0:
                time.sleep(0.01)

            self._set_var(self.ACT, 1)
            time.sleep(1.0)
            while self._get_var(self.ACT) != 1 or self._get_var(self.STA) != 3:
                time.sleep(0.01)

        # auto-calibrate position range if desired
        if auto_calibrate:
            self.auto_calibrate()

    def is_active(self):
        """Returns whether the gripper is active."""
        status = self._get_var(self.STA)
        return GripperStatus(status) == GripperStatus.ACTIVE

    def get_min_position(self) -> int:
        """Returns the minimum position the gripper can reach (open position)."""
        return self._min_position

    def get_max_position(self) -> int:
        """Returns the maximum position the gripper can reach (closed position)."""
        return self._max_position

    def get_open_position(self) -> int:
        """Returns what is considered the open position for gripper (minimum position value)."""
        return self.get_min_position()

    def get_closed_position(self) -> int:
        """Returns what is considered the closed position for gripper (maximum position value)."""
        return self.get_max_position()

    def is_open(self):
        """Returns whether the current position is considered as being fully open."""
        return self.get_current_position() <= self.get_open_position()

    def is_closed(self):
        """Returns whether the current position is considered as being fully closed."""
        return self.get_current_position() >= self.get_closed_position()

    def get_current_position(self) -> int:
        """Returns the current position as returned by the physical hardware."""
        return self._get_var(self.POS)

    def auto_calibrate(self, log: bool = True) -> None:
        """Attempts to calibrate the open and closed positions, by slowly closing and opening the gripper.
        :param log: Whether to print the results to log.
        """
        # first try to open in case we are holding an object
        (position, status) = self.move_and_wait_for_pos(self.get_open_position(), 64, 1)
        if ObjectStatus(status) != ObjectStatus.AT_DEST:
            raise RuntimeError(f"Calibration failed opening to start: {str(status)}")

        # try to close as far as possible, and record the number
        (position, status) = self.move_and_wait_for_pos(self.get_closed_position(), 64, 1)
        if ObjectStatus(status) != ObjectStatus.AT_DEST:
            raise RuntimeError(f"Calibration failed because of an object: {str(status)}")
        assert position <= self._max_position
        self._max_position = position

        # try to open as far as possible, and record the number
        (position, status) = self.move_and_wait_for_pos(self.get_open_position(), 64, 1)
        if ObjectStatus(status) != ObjectStatus.AT_DEST:
            raise RuntimeError(f"Calibration failed because of an object: {str(status)}")
        assert position >= self._min_position
        self._min_position = position

        if log:
            print(f"Gripper auto-calibrated to [{self.get_min_position()}, {self.get_max_position()}]")

    def move(self, position: int, speed: int, force: int) -> Tuple[bool, int]:
        """Sends commands to start moving towards the given position, with the specified speed and force.
        :param position: Position to move to [min_position, max_position]
        :param speed: Speed to move at [min_speed, max_speed]
        :param force: Force to use [min_force, max_force]
        :return: A tuple with a bool indicating whether the action it was successfully sent, and an integer with
        the actual position that was requested, after being adjusted to the min/max calibrated range.
        """

        def clip_val(min_val, val, max_val):
            return max(min_val, min(val, max_val))

        clip_pos = clip_val(self._min_position, position, self._max_position)
        clip_spe = clip_val(self._min_speed, speed, self._max_speed)
        clip_for = clip_val(self._min_force, force, self._max_force)

        # moves to the given position with the given speed and force
        var_dict = OrderedDict([(self.POS, clip_pos), (self.SPE, clip_spe), (self.FOR, clip_for), (self.GTO, 1)])
        return self._set_vars(var_dict), clip_pos

    def move_and_wait_for_pos(self, position: int, speed: int, force: int) -> Tuple[int, ObjectStatus]:  # noqa
        """Sends commands to start moving towards the given position, with the specified speed and force, and
        then waits for the move to complete.
        :param position: Position to move to [min_position, max_position]
        :param speed: Speed to move at [min_speed, max_speed]
        :param force: Force to use [min_force, max_force]
        :return: A tuple with an integer representing the last position returned by the gripper after it notified
        that the move had completed, a status indicating how the move ended (see ObjectStatus enum for details). Note
        that it is possible that the position was not reached, if an object was detected during motion.
        """
        set_ok, cmd_pos = self.move(position, speed, force)
        if not set_ok:
            raise RuntimeError("Failed to set variables for move.")

        # wait until the gripper acknowledges that it will try to go to the requested position
        while self._get_var(self.PRE) != cmd_pos:
            time.sleep(0.001)

        # wait until not moving
        cur_obj = self._get_var(self.OBJ)
        while ObjectStatus(cur_obj) == ObjectStatus.MOVING:
            cur_obj = self._get_var(self.OBJ)

        # report the actual position and the object status
        final_pos = self._get_var(self.POS)
        final_obj = cur_obj
        return final_pos, ObjectStatus(final_obj)
