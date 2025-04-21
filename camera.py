#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
import cv2
from cv_bridge import CvBridge

class LineFollower:
    def __init__(self):
        rospy.init_node('line_follower_node')
        
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber('/camera/image_raw', Image, self.image_callback)
        self.cmd_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)

    def image_callback(self, msg):
        # تحويل صورة ROS إلى OpenCV
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

        # تقليل الصورة للجزء السفلي بس (عشان نشوف الخط بس)
        height, width, _ = cv_image.shape
        roi = cv_image[int(0.7*height):, :]

        # تحويل إلى HSV واستخراج الأبيض
        hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
        lower_white = (0, 0, 200)
        upper_white = (180, 30, 255)
        mask = cv2.inRange(hsv, lower_white, upper_white)

        # نحسب مركز الخط
        M = cv2.moments(mask)
        if M['m00'] > 0:
            cx = int(M['m10'] / M['m00'])
            err = cx - width // 2

            twist = Twist()
            twist.linear.x = 0.2
            twist.angular.z = -float(err) / 100.0  # تصحيح الاتجاه
            self.cmd_pub.publish(twist)
        else:
            # لو مش شايف خط → وقف
            twist = Twist()
            twist.linear.x = 0.0
            twist.angular.z = 0.0
            self.cmd_pub.publish(twist)

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    LineFollower().run()
