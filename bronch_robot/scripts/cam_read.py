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
from std_msgs.msg import Float64MultiArray


def rotate_image(image, angle):
    image_center = tuple(np.array(image.shape[1::-1]) / 2)
    rot_mat = cv2.getRotationMatrix2D(image_center, angle, 1.0)
    result = cv2.warpAffine(image, rot_mat, image.shape[1::-1], flags=cv2.INTER_LINEAR)
    # flipVertical = cv2.flip(result, 1)
    return result


class cam_read:

    def __init__(self):
        # initialize the bridge between openCV and ROS
        self.bridge = CvBridge()
        rospy.init_node('read_cam', anonymous=True)
        self.image_pub = rospy.Publisher("image_topic", Image, queue_size=1)
        self.sampling_freq = 30
        self.rate = rospy.Rate(self.sampling_freq)
        self.time = rospy.get_time()
        self.cap = cv2.VideoCapture(0)  # open video capture
        self.pos = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
        self.data = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                              0.0])  # recorded data are 7 joint variable plus time plus rotation angle
        self.rot_angle = np.array([0.0])
        # Check if camera opened successfully
        if not self.cap.isOpened():
            print("Unable to read camera feed")
        # save video file
        for i in range(1, 1000):
            self.file_name = datetime.today().strftime('%Y-%m-%d') + "_Vid_" + str(i) + ".avi"
            self.file_name2 = datetime.today().strftime('%Y-%m-%d') + "_Pos_time_" + str(i) + ".dat"
            my_file = Path(self.file_name)
            if not my_file.is_file():
                break
        self.out = cv2.VideoWriter(self.file_name, cv2.VideoWriter_fourcc('M', 'J', 'P', 'G'), 30.0, (800, 800))
        np.savetxt(self.file_name2, self.data.reshape(1, 9))

    def record_show(self):
        while not rospy.is_shutdown():
            # subscribe and read actual joint positions for saving
            rospy.Subscriber("pos_actual", Float64MultiArray, self.callback)
            # subscribe and read desired rotation for image
            rospy.Subscriber("image_rot", Float64MultiArray, self.callback2)
            # Capture frame-by-frame
            ret, frame = self.cap.read()
            # Our operations on the frame come here
            frame_rot = rotate_image(frame, self.rot_angle)
            img = cv2.resize(frame_rot, (800, 800))  # Resize image
            # write the flipped+rotated frame
            self.out.write(img)
            try:
                self.image_pub.publish(self.bridge.cv2_to_imgmsg(img, "bgr8"))
            except CvBridgeError as e:
                print(e)
            a = np.append(self.pos, np.array([rospy.get_time() - self.time]), axis=0)
            self.data = np.append(a, self.rot_angle, axis=0)
            print(self.data)
            with open(self.file_name2, "a") as file:
                #Append new data at the end of file
               np.savetxt(file, self.data.reshape(1, 9))
            self.rate.sleep()
            rospy.sleep(1 / self.sampling_freq)  # enforce sampling frequency

    # retrieve actual pos data from topic
    def callback(self, data):
        self.pos = np.asarray(data.data)

    # retrieve desired image rotation angle from topic
    def callback2(self, data):
        self.rot_angle = np.asarray(data.data)


if __name__ == '__main__':
    Cam_Read = cam_read()
    try:
        Cam_Read.record_show()
    except rospy.ROSInterruptException:
        Cam_Read.out.release()
        Cam_Read.cap.release()
        pass
