#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge, CvBridgeError
import math

pub = None
bridge = CvBridge()
max_meters = 7


def imageCallback(msg):
    try:
        cv_image = bridge.imgmsg_to_cv2(msg, "32FC1")
    except CvBridgeError as e:
        print(e)

    height, width = cv_image.shape[0:2]
    for x in range(0, width):
        for y in range(0, height):
            if math.isnan(cv_image[y, x]):
                cv_image[y, x] = max_meters

    new_msg = bridge.cv2_to_imgmsg(cv_image, "32FC1")
    new_msg.header = msg.header

    pub.publish(new_msg)


def image_maxing():
    global pub
    pub = rospy.Publisher("/camera/depth/image_raw2", Image, queue_size=10)
    rospy.init_node("image_maxing", anonymous=True)

    sub = rospy.Subscriber("/camera/depth/image_raw", Image, imageCallback)

    rospy.spin()


if __name__ == '__main__':
    try:
        image_maxing()
    except rospy.ROSInterruptException:
        pass
