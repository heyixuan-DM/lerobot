import sys
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
import torch
import numpy as np
from lerobot.common.utils.utils import init_hydra_config
from lerobot.common.robot_devices.robots.factory import make_robot
from lerobot.common.utils.utils import init_hydra_config, init_logging

class TeleOperateFollow(Node):

    def __init__(self, robot_path):
        super().__init__('follower_arm_subscriber')
        self.robot_cfg = init_hydra_config(robot_path)
        self.robot = make_robot(self.robot_cfg)
        if not self.robot.is_connected:
            self.robot.connect()
        self.subscription = self.create_subscription(
            Float32MultiArray,
            '/leader_arm/state',
            self.listener_callback,
            30)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        # todo: analyse msg
        for name in self.robot.follower_arms:
            goal_pos = np.array(msg.data, dtype=np.float32)
            print("numpy float32 ", goal_pos)
            goal_pos = goal_pos.astype(np.int32)
            print("numpy int32 ", goal_pos)
            self.robot.follower_arms[name].write("Goal_Position", goal_pos)
            break
    
    def shutdown(self):
        if self.robot.is_connected:
        # Disconnect manually to avoid a "Core dump" during process
        # termination due to camera threads not properly exiting.
            self.robot.disconnect()


if __name__ == '__main__':
    rclpy.init()
    robot_path = sys.argv[1]
    follower_obj = TeleOperateFollow(robot_path)

    try:
        rclpy.spin(follower_obj)
    except KeyboardInterrupt:
        pass
    finally:
        follower_obj.shutdown()
        follower_obj.destroy_node()
        rclpy.shutdown()