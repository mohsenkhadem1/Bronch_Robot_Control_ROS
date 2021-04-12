#!/usr/bin/env python3

import roslib
import sys
import rospy
import numpy as np
from std_msgs.msg import Float64MultiArray
from pyPS4Controller.controller import Controller
import os
import struct
import time


class MyController(Controller):

    def __init__(self, **kwargs):
        # connect to joystick (it must be paired with the computer via bluetooth)
        Controller.__init__(self, **kwargs)
        # initialize the node named joystick
        rospy.init_node('joystick', anonymous=True)
        # initialize a publisher to send messages to a topic named pos_commands
        self.pub = rospy.Publisher("pos_commands", Float64MultiArray, queue_size=10)
        # initialize a publisher to send messages to a topic named image_rot
        self.pub_rot_angle = rospy.Publisher("image_rot", Float64MultiArray, queue_size=10)
        self.rate = rospy.Rate(30)  # 30hz
        self.R3_up_down, self.L3_up_down, self.L3_left_right, self.reset, self.reset_shaft, self.enable, self.home = 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0
        self.rot_angle = 0.0
        self.joints = Float64MultiArray()
        self.joints.data = [self.R3_up_down, self.L3_up_down, self.L3_left_right, self.reset, self.reset_shaft, self.enable, self.home]
        self.pub.publish(self.joints)
        self.angle = Float64MultiArray()
        self.angle.data = [self.rot_angle]
        self.pub.publish(self.angle)


    def on_R3_up(self, value):
        self.R3_up_down = 1.0

    def on_R3_down(self, value):
        self.R3_up_down = -1.0

    def on_R3_y_at_rest(self):
        self.R3_up_down = 0.0

    def on_L3_up(self, value):
        self.L3_up_down = 1.0

    def on_L3_down(self, value):
        self.L3_up_down = -1.0

    def on_L3_left(self, value):
        self.L3_left_right = 1.0

    def on_L3_right(self, value):
        self.L3_left_right = -1.0

    def on_L3_y_at_rest(self):
        self.L3_up_down = 0.0

    def on_L3_x_at_rest(self):
        self.L3_left_right = 0.0

    def on_L1_press(self):
        self.reset = 1.0

    def on_L1_release(self):
        self.reset = 0.0

    def on_L2_press(self):
        self.reset_shaft = 1.0

    def on_L2_release(self):
        self.reset_shaft = 0.0

    def on_R1_press(self):
        self.enable = 1.0

    def on_R1_release(self):
        self.enable = 0.0

    def on_x_press(self):
        if self.home == 0.0:
            self.home = 1.0
        else:
            self.home = 0.0

    def on_up_arrow_press(self):
        self.rot_angle -= 5.0

    def on_down_arrow_press(self):
        self.rot_angle += 5.0

    # Listen to joystick and Publish pos commands for motors
    def listener(self, timeout=30):

        def read_events():
            try:
                return _file.read(self.event_size)
            except IOError:
                print("Interface lost. Device disconnected?")
                exit(1)

        def unpack():
            __event = struct.unpack(self.event_format, event)
            return __event[3:], __event[2], __event[1], __event[0]

        try:
            _file = open(self.interface, "rb")
            event = read_events()
            while not rospy.is_shutdown() and event:
                (overflow, value, button_type, button_id) = unpack()
                if button_id not in self.black_listed_buttons:
                    self.__handle_event(button_id=button_id, button_type=button_type, value=value, overflow=overflow,
                                        debug=self.debug)
                    self.joints.data = [self.R3_up_down, self.L3_up_down, self.L3_left_right, self.reset, self.reset_shaft, self.enable, self.home]
                    print(self.joints.data)
                    # self.reset resets all cables of the tip, self.reset_shaft resets all cables of the backbone, self.enable enables movement of backbone
                    self.pub.publish(self.joints)
                    self.angle.data = [self.rot_angle]
                    self.pub_rot_angle.publish(self.angle)
                    # self.rate.sleep()
                event = read_events()
        except rospy.ROSInterruptException:
            print("\nExiting (Ctrl + C)")
            exit()

    def __handle_event(self, button_id, button_type, value, overflow, debug):

        event = self.event_definition(button_id=button_id,
                                      button_type=button_type,
                                      value=value,
                                      connecting_using_ds4drv=self.connecting_using_ds4drv,
                                      overflow=overflow,
                                      debug=debug)

        if event.R3_event():
            self.event_history.append("right_joystick")
            if event.R3_y_at_rest():
                self.on_R3_y_at_rest()
            elif event.R3_x_at_rest():
                self.on_R3_x_at_rest()
            elif event.R3_right():
                self.on_R3_right(value)
            elif event.R3_left():
                self.on_R3_left(value)
            elif event.R3_up():
                self.on_R3_up(value)
            elif event.R3_down():
                self.on_R3_down(value)
        elif event.L3_event():
            self.event_history.append("left_joystick")
            if event.L3_y_at_rest():
                self.on_L3_y_at_rest()
            elif event.L3_x_at_rest():
                self.on_L3_x_at_rest()
            elif event.L3_up():
                self.on_L3_up(value)
            elif event.L3_down():
                self.on_L3_down(value)
            elif event.L3_left():
                self.on_L3_left(value)
            elif event.L3_right():
                self.on_L3_right(value)
        elif event.circle_pressed():
            self.event_history.append("circle")
            self.on_circle_press()
        elif event.circle_released():
            self.on_circle_release()
        elif event.x_pressed():
            self.event_history.append("x")
            self.on_x_press()
        elif event.x_released():
            self.on_x_release()
        elif event.triangle_pressed():
            self.event_history.append("triangle")
            self.on_triangle_press()
        elif event.triangle_released():
            self.on_triangle_release()
        elif event.square_pressed():
            self.event_history.append("square")
            self.on_square_press()
        elif event.square_released():
            self.on_square_release()
        elif event.L1_pressed():
            self.event_history.append("L1")
            self.on_L1_press()
        elif event.L1_released():
            self.on_L1_release()
        elif event.L2_pressed():
            self.event_history.append("L2")
            self.on_L2_press()
        elif event.L2_released():
            self.on_L2_release()
        elif event.R1_pressed():
            self.event_history.append("R1")
            self.on_R1_press()
        elif event.R1_released():
            self.on_R1_release()
        elif event.R2_pressed():
            self.event_history.append("R2")
            self.on_R2_press()
        elif event.R2_released():
            self.on_R2_release()
        elif event.options_pressed():
            self.event_history.append("options")
            self.on_options_press()
        elif event.options_released():
            self.on_options_release()
        elif event.left_right_arrow_released():
            self.on_left_right_arrow_release()
        elif event.up_down_arrow_released():
            self.on_up_down_arrow_release()
        elif event.left_arrow_pressed():
            self.event_history.append("left")
            self.on_left_arrow_press()
        elif event.right_arrow_pressed():
            self.event_history.append("right")
            self.on_right_arrow_press()
        elif event.up_arrow_pressed():
            self.event_history.append("up")
            self.on_up_arrow_press()
        elif event.down_arrow_pressed():
            self.event_history.append("down")
            self.on_down_arrow_press()
        elif event.playstation_button_pressed():
            self.event_history.append("ps")
            self.on_playstation_button_press()
        elif event.playstation_button_released():
            self.on_playstation_button_release()
        elif event.share_pressed():
            self.event_history.append("share")
            self.on_share_press()
        elif event.share_released():
            self.on_share_release()
        elif event.R3_pressed():
            self.event_history.append("R3")
            self.on_R3_press()
        elif event.R3_released():
            self.on_R3_release()
        elif event.L3_pressed():
            self.event_history.append("L3")
            self.on_L3_press()
        elif event.L3_released():
            self.on_L3_release()


# run the code if the node is called
if __name__ == '__main__':
    controller = MyController(interface="/dev/input/js0", connecting_using_ds4drv=False)
    try:
        controller.listener()
    except KeyboardInterrupt:
        print("Shutting down")
