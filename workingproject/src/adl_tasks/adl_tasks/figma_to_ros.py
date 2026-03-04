import sys
import threading
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from PyQt5.Widgests import QApplication, QWidget, QPushButton, QVBoxLayout
from PyQt5.QtCore import Qt
from clear_table import *
from ex_move import *
from pick_dropped_bottle import *
from helper_moves import *


class RobotController(Node):
    def __init__(self):
        super().__init__('robot_controller')
        self.publisher = self.create_publisher(Twist, '/cmd_vel',10)

    btn_clear_table = clearTableNode(Node)

    btn_retrieve_fallen_object = PickDroppedBottle(Node)

    #btn_give_medication = 