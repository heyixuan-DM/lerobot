import sys
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
import torch
import numpy as np
import logging

from dataclasses import asdict
from pprint import pformat
from lerobot.common.robot_devices.robots.utils import make_robot_from_config
from lerobot.common.robot_devices.control_configs import ControlPipelineConfig
from lerobot.common.utils.utils import init_logging
from lerobot.configs import parser
from functools import partial

class TeleOperateFollow(Node):

    def __init__(self, robot):
        super().__init__('follower_arm_subscriber')
        self.robot = robot
        if not self.robot.is_connected:
            self.robot.connect()
        self.subscription_left = self.create_subscription(
            Float32MultiArray,
            '/leader_left/state',
            partial(self.listener_callback, arm_name="left"),
            30)
        self.subscription_right = self.create_subscription(
            Float32MultiArray,
            '/leader_right/state',
            partial(self.listener_callback, arm_name="right"),
            30)
        self.subscription_left  # prevent unused variable warning
        self.subscription_right  # prevent unused variable warning

    def listener_callback(self, msg, arm_name):
        # todo: analyse msg
        goal_pos = np.array(msg.data, dtype=np.float32)
        print("numpy float32 ", goal_pos)
        goal_pos = goal_pos.astype(np.int32)
        print("numpy int32 ", goal_pos)
        self.robot.follower_arms[arm_name].write("Goal_Position", goal_pos)
        
    
    def shutdown(self):
        if self.robot.is_connected:
        # Disconnect manually to avoid a "Core dump" during process
        # termination due to camera threads not properly exiting.
            self.robot.disconnect()

@parser.wrap()
def parse_arg(cfg: ControlPipelineConfig):
    init_logging()
    logging.info(pformat(asdict(cfg)))

    robot = make_robot_from_config(cfg.robot)
    return robot

if __name__ == '__main__':
    rclpy.init()
    robot = parse_arg()
    follower_obj = TeleOperateFollow(robot)

    try:
        rclpy.spin(follower_obj)
    except KeyboardInterrupt:
        pass
    finally:
        follower_obj.shutdown()
        follower_obj.destroy_node()
        rclpy.shutdown()