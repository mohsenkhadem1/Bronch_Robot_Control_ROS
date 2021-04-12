#!/usr/bin/env python3

import roslib
import sys
import rospy
import numpy as np
import can
from can.interface import Bus
import struct
from pathlib import Path
from datetime import datetime
from std_msgs.msg import Float64MultiArray


def _get_message(msg):
    return msg


def swap32(i):
    return struct.unpack("<I", struct.pack(">I", i))[0]


class read_pos():

    def __init__(self):
        rospy.init_node('read_pos', anonymous=True)
        self.bus = can.Bus(channel='0', bustype="kvaser", bitrate=500000)
        self.buffer = can.BufferedReader()
        self.sampling_freq = 30
        self.notifier = can.Notifier(self.bus, [_get_message, self.buffer])
        self.insertion = np.array([0.0])
        self.RX_PDO = 0x1A1
        self.POS = Float64MultiArray()
        self.POS.data = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
        self.time = rospy.get_time()
        self.pub = rospy.Publisher("pos_actual", Float64MultiArray, queue_size=10)
        self.pot_ratio = {0: 4 / 614400, 1: 0.0415, 2: 0.0412, 3: 0.0418, 4: 0.0447, 5: 0.0413, 6: 0.0413}
        self.pub.publish(self.POS)

    def read_input(self):
        while not rospy.is_shutdown():
            data = np.array([[0.0, 0.0, 0.0, 0.0, 0.0, 0.0]])
            while not np.all(data):
                msg = self.bus.recv()
                id = msg.arbitration_id - self.RX_PDO
                if id == 0:
                    pos_motor1 = msg.data
                    pos_motor1[0], pos_motor1[1], pos_motor1[2], pos_motor1[3] = msg.data[3], msg.data[2], msg.data[1], \
                                                                                 msg.data[0]
                    self.insertion[0] = self.pot_ratio[id] * int.from_bytes(msg.data, byteorder='big', signed=False)
                    if self.insertion[0] > 10000:
                        self.insertion[0] = 0
                else:
                    data[0, id - 1] = self.pot_ratio[id] * int.from_bytes(msg.data, byteorder='little', signed=False)
            self.POS.data = np.append(self.insertion, data)
            self.pub.publish(self.POS)
        return

# run the code if the node is called
if __name__ == '__main__':
    read_pos = read_pos()
    try:
        read_pos.read_input()
    except rospy.ROSInterruptException:
        pass
