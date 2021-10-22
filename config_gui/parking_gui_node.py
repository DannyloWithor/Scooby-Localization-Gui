#!/usr/bin/env python3
from kivy.config import Config
Config.set('graphics', 'width', '400')
Config.set('graphics', 'height', '400')
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
import weakref


class markerNode(Node):
    def __init__(self):
        super().__init__('parking_node')
        self.publisher_ = self.create_publisher(Odometry, '/parking_message', 1000)

    def publish_parking_spot(self, parking_spot, spots):
        my_msg = Odometry()
        current_spot = spots[str(parking_spot)]

        
        my_msg.pose.pose.position.x = float(current_spot[0])     # Spot pose in x
        my_msg.pose.pose.position.y = float(current_spot[1])     # Spot pose in y
        my_msg.pose.pose.position.z = float(current_spot[2])     # Spot pose in th
        my_msg.twist.twist._linear.z = float(parking_spot)       # Spot id

        self.publisher_.publish(my_msg)

class GuiApp(MDApp):

    def __init__(self, **kwargs):
        super().__init__(**kwargs)
        self.screen = Builder.load_file('scooby_simulation/scooby/scooby_localization/config_gui/config_gui/kv/gui_original.kv')
        self.parking_state = 0
        self.pub_state = 0
        self.parking_spot = 0
        self.pub_thread = threading.Thread(target=self.publish)
        self.thread_event = threading.Event()
        self.spots = dict()

        self.pub_thread.start()

    def build(self):
        return self.screen

    def set_node(self, node):
        self.ros_node = node

    def on_clear(self):
        self.spots.clear()
        self.root.ids.number_of_spots_label.text = str(len(self.spots)) + "  vagas cadastradas!"

    def on_add(self):
        # Retrieve spot parameters
        text = self.root.ids.add_parking_spot.text
        splittedText = text.split()

        if(((len(splittedText) != 4))or(splittedText[0] == "0")):
            self.root.ids.spot_label.text = "Invalido!"
            return

        self.spots[splittedText[0]] = splittedText[1:4]
        self.root.ids.number_of_spots_label.text = str(len(self.spots)) + "  vagas cadastradas!"

    def on_press(self):
        # Change parking state
        if(self.parking_state == 0):
            self.parking_state = 1
        else:
            self.parking_state = 0

        if(self.parking_state == 0):
            self.root.ids.parking_button.background_color = (1.0, 0.0, 0.0, 1.0)
            self.root.ids.parking_button.text = "Não"
            self.parking_spot = 0
            self.update_spot_label()
            self.stop_publishing()
            self.on_press_pub()

        if(self.parking_state == 1):
            self.root.ids.parking_button.background_color = (0.0, 1.0, 0.0, 1.0)
            self.root.ids.parking_button.text = "Sim"

    def on_press_pub(self):
        # Change pub state
        if(self.pub_state == 0):
            self.pub_state = 1
        else:
            self.pub_state = 0

        if(self.pub_state == 0):
            self.root.ids.pub_button.background_color = (0.0, 1.0, 0.0, 1.0)
            self.root.ids.pub_button.text = "Começar"
            self.stop_publishing()

        if((self.pub_state == 1)and(self.parking_spot != 0)):
            self.root.ids.pub_button.background_color = (1.0, 0.0, 0.0, 1.0)
            self.root.ids.pub_button.text = "Parar" 
            self.start_publishing()
        
    def update_spot_label(self):
        if(self.parking_spot == 0):
            self.root.ids.spot_label.text = "Nenhuma vaga selecionada!"
            return

        self.root.ids.spot_label.text = "Scooby está na vaga " + str(self.parking_spot)

    def set_parking_spot(self):
        if(self.parking_state == 0):
            self.root.ids.spot_label.text = "O scooby está em movimento!"
            return
            
        text = self.screen.ids.parking_spot.text

        if((len(text) != 1)):
            self.root.ids.spot_label.text = "Inválido!"
            return

        self.parking_spot = int(text)
        self.update_spot_label()

    def stop_publishing(self):
        self.thread_event.clear()

    def start_publishing(self):
        self.thread_event.set()

    def publish(self):
        while(True):
            event_is_set = self.thread_event.wait()
            print("Publishing!")
            self.ros_node.publish_parking_spot(self.parking_spot, self.spots)
            time.sleep(1)

def main(args=None):
    rclpy.init(args=args)

    ros_node = markerNode()
    app = GuiApp()
    app.set_node(ros_node)
    app.run()

    ##rclpy.spin(ros_node)

    
    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    #uwbPub.destroy_node()
    rclpy.shutdown()
    ros_node.destroy_node()


if __name__ == '__main__':
    main()