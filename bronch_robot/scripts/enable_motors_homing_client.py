#!/usr/bin/env python3

from __future__ import print_function

import sys
import rospy
from bronch_robot.srv import *

import can
from can.interface import Bus


def enable_motors_homing_client(x):
    rospy.wait_for_service('enable_motors_homing')
    try:
        enable_motors_homing = rospy.ServiceProxy('enable_motors_homing', EnableMotors)
        resp1 = enable_motors_homing(x)
        return resp1.out
    except rospy.ServiceException as e:
        print("Service call failed: %s" % e)


def usage():
    return "%s [number]" % sys.argv[0]


if __name__ == "__main__":
    if len(sys.argv) == 2:
        x = int(sys.argv[1])
    else:
        print(usage())
        sys.exit(1)
    enable_motors_homing_client(x)
    print("All motors ready for homing")
