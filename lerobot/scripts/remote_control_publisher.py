import sys
import rclpy
from rclpy.node import Node
import numpy as np
from std_msgs.msg import Float32MultiArray
import logging

from dataclasses import asdict
from pprint import pformat
from lerobot.common.robot_devices.robots.utils import make_robot_from_config
from lerobot.common.robot_devices.control_configs import ControlPipelineConfig
from lerobot.common.utils.utils import init_logging
from lerobot.configs import parser

class TeleOperateObj(Node):
    def __init__(self, robot, fps):
        super().__init__('leader_arm_publisher')
        self.robot = robot
        if not self.robot.is_connected:
            self.robot.connect()
        self.publisher_left = self.create_publisher(Float32MultiArray, '/leader_left/state', 30)
        self.publisher_right = self.create_publisher(Float32MultiArray, '/leader_right/state', 30)
        timer_period = 1.0 / fps  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
    def timer_callback(self):
        leader_pos_left=self.robot.leader_arms["left"].read("Present_Position")
        leader_pos_right=self.robot.leader_arms["right"].read("Present_Position")

        msg_left = Float32MultiArray()
        msg_right = Float32MultiArray()
        msg_left.data = leader_pos_left.tolist()
        msg_right.data = leader_pos_right.tolist()
        self.publisher_left.publish(msg_left)
        self.publisher_right.publish(msg_right)
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
    fps = cfg.control.fps
    return robot, fps

if __name__ == "__main__":
    rclpy.init()
    robot, fps = parse_arg()
    obj = TeleOperateObj(robot, fps)
    try:
        rclpy.spin(obj)
    except KeyboardInterrupt:
        pass
    finally:
        obj.shutdown()
        obj.destroy_node()
        rclpy.shutdown()