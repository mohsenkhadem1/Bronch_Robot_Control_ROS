#!/usr/bin/env python3

from __future__ import print_function

from bronch_robot.srv import EnableMotors, EnableMotorsResponse
import rospy

import can
from can.interface import Bus


def handle_enable_motors(req):
    # set up connection to hardware
    can.rc['interface'] = "kvaser"
    can.rc['channel'] = '1'
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

    # Set all motors to position mode
    # If you want to set the motors to velocity mode change
    # the comand from [hex2dec('0F') hex2dec('0') hex2dec('01')] to
    # [hex2dec('0F') hex2dec('0') hex2dec('03')];

    for i in range(1, 8):
        msg = can.Message(arbitration_id=0x320 + i,
                          data=[0x0F, 0x00, 0x01],
                          is_extended_id=False)
        bus.send(msg)

    # Set rotational speed of motors
    for i in range(2, 8):
        msg = can.Message(arbitration_id=0x600 + i,
                          data=[0x22, 0x81, 0x60, 0x0, 0x88, 0x13, 0x0, 0x0], # 5000 rpm
                          is_extended_id=False)
        bus.send(msg)
    msg = can.Message(arbitration_id=0x601,
                      data=[0x22, 0x81, 0x60, 0x0, 0x70, 0x17, 0x0, 0x0], #6000 rpm
                      is_extended_id=False)
    bus.send(msg)

    bus.shutdown()

    print("Motors Enabled")
    return EnableMotorsResponse(req.a)


def enable_motors_server():
    rospy.init_node('enable_motors_server')
    s = rospy.Service('enable_motors', EnableMotors, handle_enable_motors)
    print("Ready to enable all motors.")
    rospy.spin()


if __name__ == "__main__":
    enable_motors_server()
