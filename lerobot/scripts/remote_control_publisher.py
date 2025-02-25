import sys
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import numpy as np
from std_msgs.msg import Float32MultiArray
from lerobot.common.utils.utils import init_hydra_config
from lerobot.common.robot_devices.robots.factory import make_robot
from lerobot.common.utils.utils import init_hydra_config, init_logging

class TeleOperateObj(Node):
    def __init__(self, robot_path, fps):
        super().__init__('leader_arm_publisher')
        self.robot_cfg = init_hydra_config(robot_path)
        self.robot = make_robot(self.robot_cfg)
        if not self.robot.is_connected:
            self.robot.connect()
        self.publisher = self.create_publisher(Float32MultiArray, '/leader_arm/state', 30)
        timer_period = 1.0 / fps  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
    def timer_callback(self):
        leader_pos = None
        for name in self.robot.leader_arms:
            leader_pos=self.robot.leader_arms[name].read("Present_Position")
            break

        msg = Float32MultiArray()
        msg.data = leader_pos.tolist()
        self.publisher.publish(msg)
    def shutdown(self):
        if self.robot.is_connected:
        # Disconnect manually to avoid a "Core dump" during process
        # termination due to camera threads not properly exiting.
            self.robot.disconnect()

if __name__ == "__main__":
    rclpy.init()
    robot_path = sys.argv[1]
    fps = int(sys.argv[2])
    obj = TeleOperateObj(robot_path, fps)
    try:
        rclpy.spin(obj)
    except KeyboardInterrupt:
        pass
    finally:
        obj.shutdown()
        obj.destroy_node()
        rclpy.shutdown()