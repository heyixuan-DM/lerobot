#!/usr/bin/env python

# Copyright 2024 The HuggingFace Inc. team. All rights reserved.
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
from rclpy.node import Node
from geometry_msgs.msg import Twist

import logging
from .config_lekiwi import LeKiwiConfig
from .lekiwi import LeKiwi
import time
import math

class TwistSubscriber(Node):
    def __init__(self):
        super().__init__('twist_subscriber')

        logging.info("Configuring LeKiwi")
        self.robot_config = LeKiwiConfig()
        self.robot = LeKiwi(self.robot_config)
        logging.info("Connecting LeKiwi")
        self.robot.connect()

        # 创建订阅者，订阅/cmd_vel话题（Twist类型）
        self.subscription = self.create_subscription(
            Twist,
            '/lekiwi/cmd_vel',
            self.twist_callback,
            10
        )
    def twist_callback(self, msg):
        # 提取线速度和角速度
        data = {
            "x.vel": 0,
            "y.vel": 0,
            "theta.vel": 0,
        }

        data["x.vel"] = msg.linear.x # 前进方向线速度（单位：m/s）
        data["theta.vel"] = msg.angular.z * 180 / math.pi # 绕Z轴角速度（单位：degree/s）
        _action_sent = self.robot.send_action(data)
        
        self.get_logger().info(
            f'收到速度指令: 线速度={data["x.vel"]:.2f} m/s, 角速度={data["theta.vel"]:.2f} degree/s'
        )
    def destroy_node(self):
        self.get_logger().info("destroy_node...")
        self.robot.stop_base()
        self.robot.disconnect()
        super().destroy_node()  # 调用父类方法完成标准销毁流程
        
def main(args=None):
    rclpy.init(args=args)
    node = TwistSubscriber()
    try:
        rclpy.spin(node)  # 保持节点运行
    except KeyboardInterrupt:
        print("exit")
        node.destroy_node()
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()
