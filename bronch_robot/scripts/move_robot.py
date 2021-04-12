#!/usr/bin/env python3

# This code must be used after all the motors are enabled for homing

import roslib
import sys
import rospy
import cv2
import numpy as np
from std_msgs.msg import Float64MultiArray
import can
from can.interface import Bus
import struct
import ctypes
import time


class Motors:

    def __init__(self):
        self.sampling_freq = 30
        self.motor_conv_ratio = np.array(
            [614400 / 4, 614400 / 2, 614400 / 2, 614400 / 2, 614400 / 2, 614400 / 2, 614400 / 2])
        self.pos_motor = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
        self.pos = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
        rospy.init_node('move_robot', anonymous=True)
        self.rate = rospy.Rate(self.sampling_freq)
        self.time = rospy.get_time()
        self.velocity_insert = 20.0  # mm/sec
        self.velocity_pull = 5.0  # mm/sec
        self.velocity_pull_shaft = 6.0  # mm/sec
        self.left_right_shaft, self.up_down_shaft, self.left_right, self.up_down, self.reset, self.reset_shaft, self.enable, self.home = 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0
        self.counter = 1  # counter is used to stop retraction whenever X button pressed
        self.rot_angle = np.array([0.0])

    def move_motors(self):
        while not rospy.is_shutdown():
            # subscribe and read joystick command from publisher
            rospy.Subscriber("pos_commands", Float64MultiArray, self.callback)
            # subscribe and read desired rotation for image
            rospy.Subscriber("image_rot", Float64MultiArray, self.callback2)

            # set secondary control commands
            self.reset, self.reset_shaft, self.enable, self.home = self.pos[3], self.pos[4], self.pos[5], self.pos[6]

            # insertion/retraction
            self.pos_motor[0] += (rospy.get_time() - self.time) * self.pos[0] * self.velocity_insert
            # insertion cannot be negative (before where it started)
            if self.pos_motor[0] <= 0:
                self.pos_motor[0] = 0
                self.home = 0.0

            if self.enable == 0:
                # left/right and up/down for tip
                self.left_right += np.cos(-self.rot_angle) * (rospy.get_time() - self.time) * self.pos[
                    2] * -self.velocity_pull - np.sin(-self.rot_angle) * (rospy.get_time() - self.time) * self.pos[
                                       1] * self.velocity_pull
                self.up_down += np.sin(-self.rot_angle) * (rospy.get_time() - self.time) * self.pos[
                    2] * -self.velocity_pull + np.cos(-self.rot_angle) * (rospy.get_time() - self.time) * self.pos[
                                    1] * self.velocity_pull
            else:
                # left/right and up/down for shaft
                self.left_right_shaft += np.cos(self.rot_angle) * (rospy.get_time() - self.time) * self.pos[2] * self.velocity_pull_shaft - np.sin(self.rot_angle) * (rospy.get_time() - self.time) * self.pos[1] * self.velocity_pull_shaft
                self.up_down_shaft += np.sin(self.rot_angle) * (rospy.get_time() - self.time) * self.pos[2] * self.velocity_pull_shaft + np.cos(self.rot_angle) * (rospy.get_time() - self.time) * self.pos[1] * self.velocity_pull_shaft

            # convert tip commands to pull/push
            r = 2
            alpha = -np.pi / 20 # deviation
            delta = np.arctan2(self.up_down, self.left_right)
            delta1 = delta + alpha
            delta2 = delta + 2 * np.pi / 3 + alpha
            delta3 = delta + 4 * np.pi / 3 + alpha

            tetha = np.sqrt(self.left_right ** 2 + self.up_down ** 2) * 2 * np.pi / 20

            dl1 = r * np.cos(delta1) * tetha
            dl2 = r * np.cos(delta2) * tetha
            dl3 = r * np.cos(delta3) * tetha

            if np.sqrt(self.left_right ** 2 + self.up_down ** 2) < 55:
                self.pos_motor[3] = dl2
                self.pos_motor[1] = dl3
                self.pos_motor[4] = dl1
            else:
                print('WARNING Warning Robot is Breaking')

            # convert shaft commands to pull/push
            r = 2
            alpha = -np.pi / 20 - np.pi / 6
            delta = np.arctan2(self.up_down_shaft, self.left_right_shaft)
            delta1 = delta + alpha
            delta2 = delta + 2 * np.pi / 3 + alpha
            delta3 = delta + 4 * np.pi / 3 + alpha

            tetha = np.sqrt(self.left_right_shaft ** 2 + self.up_down_shaft ** 2) * 2 * np.pi / 20

            dl6 = r * np.cos(delta1) * tetha
            dl5 = r * np.cos(delta2) * tetha
            dl4 = r * np.cos(delta3) * tetha

            if np.sqrt(self.left_right_shaft ** 2 + self.up_down_shaft ** 2) < 25:
                self.pos_motor[6] = dl5
                self.pos_motor[2] = dl6
                self.pos_motor[5] = dl4
            else:
                print('WARNING Warning Robot is Breaking')

            # reset cables pos for tip if L1 is pressed
            if self.reset == 1:
                self.pos_motor[3] = 0
                self.pos_motor[1] = 0
                self.pos_motor[4] = 0
                self.left_right = 0
                self.up_down = 0
                print('Tip cables resetting')

            # reset cables pos for shaft if L2 is pressed
            if self.reset_shaft == 1:
                self.pos_motor[6] = 0
                self.pos_motor[2] = 0
                self.pos_motor[5] = 0
                self.left_right_shaft = 0
                self.up_down_shaft = 0
                print('Shaft cables resetting')

            # home robot if X is pressed
            if self.home == 1:
                self.pos_motor[1:] = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
                self.pos_motor[0] -= (rospy.get_time() - self.time) * 1 * self.velocity_insert  # starting retracting
                self.left_right_shaft, self.up_down_shaft, self.left_right, self.up_down, self.reset, self.reset_shaft, self.enable, self.home = 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0
                print('Shaft cables resetting')

            command = self.motor_conv_ratio * self.pos_motor
            print(self.pos_motor)

            self.send_message(command.reshape(7, ).astype('int64'))
            self.time = rospy.get_time()
            rospy.sleep(1 / self.sampling_freq)  # enforce sampling frequency

    def send_message(self, data):
        bus = can.Bus(channel='0', bustype="kvaser", bitrate=500000)
        # print(data)
        for i in range(0, len(data)):
            Data = self.pos2message(int(data[i]))
            # set position value
            msg = can.Message(arbitration_id=0x421 + i,
                              data=[0x3F, 0x00, int(Data[0:2], 16), int(Data[2:4], 16), int(Data[4:6], 16),
                                    int(Data[6:8], 16)],
                              is_extended_id=False)
            bus.send(msg)
            time.sleep(0.01)
            # toggle new position
            msg = can.Message(arbitration_id=0x421 + i,
                              data=[0x0F, 0x00, 0x00, 0x00, 0x00, 0x00],
                              is_extended_id=False)
            bus.send(msg)

        bus.shutdown()
        return

    # functions for swapping bytes and turning position data into hexadecimal
    def pos2message(self, i):
        num = ctypes.c_uint32(i).value  # convert int into uint
        num2 = struct.unpack("<I", struct.pack(">I", num))[0]  # swap bytes
        output = hex(num2)[2:]
        return output.zfill(8)

    # retrieve pos data from topic
    def callback(self, data):
        self.pos = np.asarray(data.data)

    # retrieve desired image rotation angle from topic
    def callback2(self, data):
        self.rot_angle = np.asarray(data.data)


if __name__ == '__main__':
    Motors = Motors()
    try:
        Motors.move_motors()
    except rospy.ROSInterruptException:
        pass
