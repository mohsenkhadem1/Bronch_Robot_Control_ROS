#!/usr/bin/env python3

from __future__ import print_function

from bronch_robot.srv import EnableMotors, EnableMotorsResponse
import rospy

import can
from can.interface import Bus


def handle_enable_motors_homing(req):
    # set up connection to hardware
    can.rc['interface'] = "kvaser"
    can.rc['channel'] = '0'
    can.rc['bitrate'] = 500000

    bus = Bus()

    # Clear fault commands
    for i in range(1, 8):
        msg = can.Message(arbitration_id=0x600 + i,
                          data=[int("40", 16), int("41", 16), int("60", 16), 0, 0, 0, 0, 0],
                          is_extended_id=False)
        bus.send(msg)

    # Start remote node via NMT
    #  different commands can be used to set operation mode (pre-op, start, etc). For all of them the
    #  Cob Id is 0 in NMT mode. Data has two bytes. First byte is a desired command, one of
    #  the five following commands can be used
    #  80 is pre-operational
    #  81 is reset node
    #  82 is reset communication
    #  01 is start
    #  02 is stop
    #  second byte is node id, can be 0 (all nodes) or a number between 1 to 256.
    for i in range(1, 8):
        msg = can.Message(arbitration_id=0x0,
                          data=[0x01, int("0%d\n" % i, 16)],
                          is_extended_id=False)
        bus.send(msg)

    # Enable All Motors
    for i in range(1, 8):
        msg = can.Message(arbitration_id=0x220 + i,
                          data=[0x00, 0x00],
                          is_extended_id=False)
        bus.send(msg)

    for i in range(1, 8):
        msg = can.Message(arbitration_id=0x220 + i,
                          data=[0x06, 0x00],
                          is_extended_id=False)
        bus.send(msg)

    for i in range(1, 8):
        msg = can.Message(arbitration_id=0x220 + i,
                          data=[0x0F, 0x00],
                          is_extended_id=False)
        bus.send(msg)

    # Set all motors to pos mode

    for i in range(1, 8):
        msg = can.Message(arbitration_id=0x320 + i,
                          data=[0x0F, 0x00, 0x01],
                          is_extended_id=False)
        bus.send(msg)

    bus.shutdown()
    print("Motors ready for homing.")
    return EnableMotorsResponse(req.a)


def enable_motors_server():
    rospy.init_node('enable_motors_homing_server')
    s = rospy.Service('enable_motors_homing', EnableMotors, handle_enable_motors_homing)
    print("Ready to enable all motors for homing.")
    rospy.spin()


if __name__ == "__main__":
    enable_motors_server()
