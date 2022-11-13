#!/usr/bin/env python3

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
import sys

from robotiq_2f85_urcap_adapter.action import MoveGripper
from robotiq_2f85_urcap_adapter import Robotiq2f85AdapterNode

import rclpy
from rclpy.executors import MultiThreadedExecutor


def main(args=None):
    rclpy.init(args=args)

    gripper_control_adapter = Robotiq2f85AdapterNode()

    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(gripper_control_adapter)

    try:
        executor.spin()
    finally:
        executor.shutdown()
        gripper_control_adapter.destroy_node()

    rclpy.shutdown()


if __name__ == "__main__":
    main()
