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


def home_motors():
    sampling_freq = 30
    motor_conv_ratio = 614400 / 2
    pub = rospy.Publisher('motor_homing_pos_error', Float64MultiArray, queue_size=10)
    rospy.init_node('motor_homing', anonymous=True)
    rate = rospy.Rate(sampling_freq)  # sampling frequency 30hz
    while not rospy.is_shutdown():
        e_d = read_input()  # read home position error
        print(e_d)
        # pid controller for homing
        K_p = 10 * np.eye(6)
        command = motor_conv_ratio * K_p @ e_d.reshape(6, 1)
        send_message(command.reshape(6, ).astype('int64'))
        # publishing motor homing pos error
        error = Float64MultiArray()
        error.data = e_d
        pub.publish(error)
        rate.sleep()


def read_input():
    bus = can.Bus(channel='0', bustype="kvaser", bitrate=500000)
    RX_PDO = 0x1A1
    pot_ratio = {1: 0.0415, 2: 0.0412, 3: 0.0418, 4: 0.0447, 5: 0.0413,
                 6: 0.0413}  # ratio for converting current to position in meter for each potentiometer, identified manually
    home_pos = np.array(
        [61.59, 44.99, 66.84, 82.9, 73.8, 54.72])  # home position of motors in mm, identified manually
    motor_pos = np.zeros((6,), dtype=float)  # initializing an array containing motors' positions
    msg = bus.recv()

    while np.count_nonzero(motor_pos) != 6:
        msg = bus.recv()
        id = msg.arbitration_id - RX_PDO
        motor_pos[id - 1] = float(pot_ratio[id] * int.from_bytes(msg.data, byteorder='little',
                                                                 signed=False))  # convert received message into float and put it in its place in home_pos array
        # disable motor if it is homed
        if abs(home_pos[id - 1] - motor_pos[id - 1]) < 0.5:
            msg = can.Message(arbitration_id=0x221 + id,
                              data=[0x00, 0x00],
                              is_extended_id=False)
            bus.send(msg)

    error = home_pos - motor_pos
    bus.shutdown()

    return error


def send_message(data):
    bus = can.Bus(channel='0', bustype="kvaser", bitrate=500000)
    for i in range(0, 6):  # len(data)):
        Data = pos2message(int(data[i]))
        # set position value
        msg = can.Message(arbitration_id=0x421 + i + 1,
                          data=[0x3F, 0x00, int(Data[0:2], 16), int(Data[2:4], 16), int(Data[4:6], 16),
                                int(Data[6:8], 16)],
                          is_extended_id=False)
        bus.send(msg)
        time.sleep(0.01)
        # toggle new position
        msg = can.Message(arbitration_id=0x421 + i + 1,
                          data=[0x0F, 0x00, 0x00, 0x00, 0x00, 0x00],
                          is_extended_id=False)
        bus.send(msg)

    bus.shutdown()
    return


# function for swapping bytes and turning position data into hexadecimal
def pos2message(i):
    num = ctypes.c_uint32(i).value  # convert int into uint
    num2 = struct.unpack("<I", struct.pack(">I", num))[0]  # swap bytes
    output = hex(num2)[2:]
    return output.zfill(8)


if __name__ == '__main__':
    try:
        home_motors()
    except rospy.ROSInterruptException:
        pass
