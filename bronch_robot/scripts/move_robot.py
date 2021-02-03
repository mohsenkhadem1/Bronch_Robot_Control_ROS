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
        self.pos = np.array([0.0, 0.0, 0.0])
        rospy.init_node('move_robot', anonymous=True)
        self.rate = rospy.Rate(self.sampling_freq)
        self.time = rospy.get_time()
        self.velocity_insert = 10  # mm/sec
        self.velocity_pull = 1  # mm/sec

    def move_motors(self):
        while not rospy.is_shutdown():
            rospy.Subscriber("pos_commands", Float64MultiArray, self.callback)
            self.pos_motor[0] += (rospy.get_time() - self.time) * self.pos[0] * self.velocity_insert
            if self.pos[2] > 0:
                if self.pos_motor[6] == 0:
                    self.pos_motor[1] += (rospy.get_time() - self.time) * self.pos[2] * self.velocity_pull
                else:
                    self.pos_motor[6] = 0
            if self.pos[2] < 0:
                if self.pos_motor[1] == 0:
                    self.pos_motor[6] += (rospy.get_time() - self.time) * self.pos[2] * self.velocity_pull
                else:
                    self.pos_motor[1] = 0

            command = self.motor_conv_ratio * self.pos_motor
            print(self.pos_motor[0])
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


if __name__ == '__main__':
    Motors = Motors()
    try:
        Motors.move_motors()
    except rospy.ROSInterruptException:
        pass
