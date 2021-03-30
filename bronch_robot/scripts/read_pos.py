#!/usr/bin/env python3

import roslib
import sys
import rospy
import numpy as np
from std_msgs.msg import Float64MultiArray
from pyPS4Controller.controller import Controller
import os
import struct
import can
from can.interface import Bus
import struct
import ctypes
import time


def _get_message(msg):
    return msg


class read_pos():

    def __init__(self):
        self.bus = can.Bus(channel='0', bustype="kvaser", bitrate=500000)
        self.buffer = can.BufferedReader()
        self.notifier = can.Notifier(self.bus, [_get_message, self.buffer])
        self.insertion = np.array([0.0])
        self.RX_PDO = 0x1A1

        self.pot_ratio = {0: 4/614400, 1: 0.0415, 2: 0.0412, 3: 0.0418, 4: 0.0447, 5: 0.0413, 6: 0.0413}

    def read_input(self):
        while not rospy.is_shutdown():
            msg = self.bus.recv()
            data = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
            while not np.all(data):
                id = msg.arbitration_id - self.RX_PDO
                if id == 0:
                    msg = self.bus.recv()
                    print(msg.data[2])
                    byteswapped = bytearray(len(msg.data))
                    byteswapped[0::2] = msg.data[1::2]
                    byteswapped[1::2] = msg.data[0::2]
                    self.insertion[0] = self.pot_ratio[id] * int.from_bytes(byteswapped, byteorder='little', signed=False)
                else:
                    msg = self.bus.recv()
                    data[id-1] = self.pot_ratio[id] * int.from_bytes(msg.data, byteorder='little', signed=False)
            data = np.append(self.insertion, data)
            print(self.insertion)
            rospy.sleep(1 / 10)
        return


# run the code if the node is called
if __name__ == '__main__':
    read_pos = read_pos()
    try:
        read_pos.read_input()
    except rospy.ROSInterruptException:
        pass
