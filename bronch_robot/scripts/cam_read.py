#!/usr/bin/env python3

import cv2
import numpy as np


cap = cv2.VideoCapture(0)

def rotate_image(image, angle):
  image_center = tuple(np.array(image.shape[1::-1]) / 2)
  rot_mat = cv2.getRotationMatrix2D(image_center, angle, 1.0)
  result = cv2.warpAffine(image, rot_mat, image.shape[1::-1], flags=cv2.INTER_LINEAR)
  flipVertical = cv2.flip(result, 1)
  return flipVertical

while(True):
    # Capture frame-by-frame
    ret, frame = cap.read()
    # Our operations on the frame come here
    angle_rot = 60
    frame_rot = rotate_image(frame, angle_rot)
    img = cv2.resize(frame_rot, (800, 800))                    # Resize image
    # Display the resulting frame
    cv2.imshow('frame', img)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# When everything done, release the capture
cap.release()
cv2.destroyAllWindows()
