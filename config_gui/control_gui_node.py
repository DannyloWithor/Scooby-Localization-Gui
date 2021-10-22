#!/usr/bin/env python3
import sys
sys.argv = [sys.argv[0]]

from kivy.config import Config
Config.set('graphics', 'width', '450')
Config.set('graphics', 'height', '450')
from typing import Text

import threading
import time
from kivymd.uix import textfield
import rclpy
from rclpy import publisher
from rclpy.node import Node
from kivy.lang import Builder
from kivymd.app import MDApp
from kivymd.uix.list import OneLineListItem
from kivymd.uix.textfield import MDTextField
from nav_msgs.msg import Odometry
from std_msgs.msg import Int32
from std_msgs.msg import Float32
import weakref
from localization_interfaces.msg import ControlMsgs


class controlNode(Node):
    def __init__(self):
        super().__init__('control_node')
        self.publisher_ = self.create_publisher(ControlMsgs, '/control_message', 1000)

        self.linear_subscription = self.create_subscription(
            Float32,
            '/scooby/linear_error',
            self.linear_callback,
            10)

        self.angular_subscription = self.create_subscription(
            Float32,
            '/scooby/linear_error',
            self.angular_callback,
            10)

        self.linear_error = 0.0
        self.angular_error = 0.0
        self.is_new_linear_error = False

    def linear_callback(self, msg):
        self.linear_error = msg.data
        self.is_new_linear_error = True

    def angular_callback(self, msg):
        self.angular_error = msg.data

    def get_linear_error(self):
        return self.linear_error

    def get_angular_error(self):
        return self.angular_error

    def publish_control_msg(self, msg):
        self.publisher_.publish(msg)

class GuiApp(MDApp):

    def __init__(self, node):
        super().__init__()
        self.screen = Builder.load_file('scooby_simulation/scooby/scooby_localization/config_gui/config_gui/kv/gui_control.kv')
        self.linear_msg = ControlMsgs()
        self.angular_msg = ControlMsgs()
        self.ros_node = node
        #self.errors_writer = threading.Thread(target=self.write_errors)
        #self.errors_writer.start()
        
    def build(self):
        return self.screen

    def write_errors(self):
        while(True):
            self.ros_get_logger().info('Im inside the thread')
            self.screen.ids.error_distance.text = str(self.ros_node.get_linear_error())
            self.screen.ids.error_angle.text = str(self.ros_node.get_angular_error())
            time.sleep(1)

    def on_distance_save(self):
        self.linear_msg.type = 0
        self.linear_msg.p = float(self.root.ids.distance_p.text)
        self.linear_msg.i = float(self.root.ids.distance_i.text) 
        self.linear_msg.d = float(self.root.ids.distance_d.text) 
        self.linear_msg.setpoint = float(self.root.ids.distance.text)

    def on_distance_send(self):
        self.publish(self.linear_msg)

    def on_angular_save(self):
        self.angular_msg.type = 1
        self.angular_msg.p = float(self.root.ids.angular_p.text)
        self.angular_msg.i = float(self.root.ids.angular_i.text) 
        self.angular_msg.d = float(self.root.ids.angular_d.text) 
        self.angular_msg.setpoint = float(self.root.ids.angular.text)

    def on_stop(self):
        msg = ControlMsgs()
        msg.type = 2
        self.publish(msg)

    def on_angular_send(self):
        self.publish(self.angular_msg)
        
    def stop_publishing(self):
        self.thread_event.clear()

    def start_publishing(self):
        self.thread_event.set()

    def publish(self, msg):
        self.ros_node.publish_control_msg(msg)
            

def main(args=None):
    rclpy.init(args=args)

    ros_node = controlNode()
    app = GuiApp(ros_node)
    
    app.run()
    rclpy.spin(ros_node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    #uwbPub.destroy_node()
    rclpy.shutdown()
    ros_node.destroy_node()


if __name__ == '__main__':
    main()