#!/usr/bin/env python3

import roslib
import sys
import rospy
import cv2
import numpy as np
from pathlib import Path
from datetime import datetime
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image


def rotate_image(image, angle):
    image_center = tuple(np.array(image.shape[1::-1]) / 2)
    rot_mat = cv2.getRotationMatrix2D(image_center, angle, 1.0)
    result = cv2.warpAffine(image, rot_mat, image.shape[1::-1], flags=cv2.INTER_LINEAR)
    flipVertical = cv2.flip(result, 1)
    return flipVertical


class Cam_Read:

    def __init__(self):
        # initialize the bridge between openCV and ROS
        self.bridge = CvBridge()
        rospy.init_node('read_cam', anonymous=True)
        self.image_pub = rospy.Publisher("image_topic", Image, queue_size=1)
        self.sampling_freq = 30
        self.rate = rospy.Rate(self.sampling_freq)
        self.time = rospy.get_time()
        self.Time = np.array([0])
        self.cap = cv2.VideoCapture(0)  # open video capture
        # Check if camera opened successfully
        if not self.cap.isOpened():
            print("Unable to read camera feed")
        # save video file
        for i in range(1, 1000):
            self.file_name = datetime.today().strftime('%Y-%m-%d') + "_Vid_" + str(i) + ".avi"
            self.file_name2 = datetime.today().strftime('%Y-%m-%d') + "_Vid_time_" + str(i) + ".dat"
            my_file = Path(self.file_name)
            if not my_file.is_file():
                break
        self.out = cv2.VideoWriter(self.file_name, cv2.VideoWriter_fourcc('M', 'J', 'P', 'G'), 30.0, (800, 800))

    def record_show(self):
        while not rospy.is_shutdown():
            # Capture frame-by-frame
            ret, frame = self.cap.read()
            # Our operations on the frame come here
            angle_rot = 0
            frame_rot = rotate_image(frame, angle_rot)
            img = cv2.resize(frame_rot, (800, 800))  # Resize image
            # write the flipped+rotated frame
            self.out.write(img)
            try:
                self.image_pub.publish(self.bridge.cv2_to_imgmsg(img, "bgr8"))
            except CvBridgeError as e:
                print(e)
            self.Time = np.append(self.Time, np.array([rospy.get_time() - self.time]), axis=0)
            np.savetxt(self.file_name2, self.Time)
            rospy.sleep(1 / self.sampling_freq)  # enforce sampling frequency


if __name__ == '__main__':
    Cam_Read = Cam_Read()
    try:
        Cam_Read.record_show()
    except rospy.ROSInterruptException:
        Cam_Read.out.release()
        Cam_Read.cap.release()
        pass
