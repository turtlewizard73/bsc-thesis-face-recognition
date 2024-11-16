#!/usr/bin/env python3

# Ros modules
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

# Non-Ros modules
import sys
import cv2


capture = cv2.VideoCapture(0)
opened = capture.isOpened()
print('Capture opened: ', opened)
print('Python version: ', sys.version)
print('OpenCV version: ', cv2.__version__)

capture.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
capture.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
capture.set(cv2.CAP_PROP_FPS, 25)

bridge = CvBridge()

def image_publisher():
    publisher = rospy.Publisher('/image/webcam/raw', Image, queue_size = 1)
    rospy.init_node('image_publisher', anonymous=False)

    rate = rospy.Rate(10)

    while not rospy.is_shutdown():
        ret, frame = capture.read()
        if not ret:
            break

        msg = bridge.cv2_to_imgmsg(frame, 'bgr8')
        publisher.publish(msg)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

        if rospy.is_shutdown():
            capture.release()

if __name__ == '__main__':
    try:
        image_publisher()
    except rospy.ROSInterruptException:
        pass