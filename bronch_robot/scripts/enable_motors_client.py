#!/usr/bin/env python3

from __future__ import print_function

import sys
import rospy
from bronch_robot.srv import *


import can
from can.interface import Bus

def enable_motors_client(x):
    rospy.wait_for_service('enable_motors')
    try:
        enable_motors = rospy.ServiceProxy('enable_motors', EnableMotors)
        resp1 = enable_motors(x)
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
    enable_motors_client(x)
    print("All motors enabled")
