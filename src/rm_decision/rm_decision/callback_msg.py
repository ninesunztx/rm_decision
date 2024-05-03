from rclpy.node import Node
from rclpy import logging
from std_msgs.msg import UInt8
from rclpy.qos import QoSProfile

class CallBackMsg(Node):
    def __init__(self):
        self.friend_color = 0

        self.current_robot_hp = 600
        self.current_base_hp = 1500

    def callback_refree(self, msg):
        self.friend_color = msg.color
        self.current_robot_hp = msg.robot_hp
        self.current_base_hp = msg.base_hp

    