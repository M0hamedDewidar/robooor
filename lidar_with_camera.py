#!/usr/bin/env python3

import rospy
import numpy as np
from sensor_msgs.msg import LaserScan, Image
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge
import cv2

class SmartDecisionMaker:
    def __init__(self):
        rospy.init_node('smart_decision_node')

        # Subscriber للكاميرا و LiDAR
        self.scan_sub = rospy.Subscriber('/scan', LaserScan, self.scan_callback)
        self.camera_sub = rospy.Subscriber('/camera/rgb/image_raw', Image, self.camera_callback)
        
        self.cmd_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)

        self.stop_distance = 0.6
        self.safe_side_distance = 0.8
        self.reverse_mode = False

        self.bridge = CvBridge()
        self.last_camera_image = None

        # Initializing FSM States
        self.state = 'IDLE'

        # Limits for keeping within the path (representing the white lines)
        self.path_left_limit = 0.5  # نصف المسافة بين الخطين
        self.path_right_limit = 0.5  # نصف المسافة بين الخطين

    def camera_callback(self, msg):
        try:
            # تحويل صورة ROS إلى OpenCV
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            self.last_camera_image = cv_image
        except Exception as e:
            rospy.logerr("Error converting ROS Image to OpenCV Image: %s", str(e))

    def scan_callback(self, msg):
        ranges = np.array(msg.ranges)
        ranges = np.where(np.isnan(ranges), np.inf, ranges)

        front = np.concatenate((ranges[340:], ranges[:20]))
        left = ranges[60:120]
        right = ranges[240:300])
        back = ranges[160:200]

        front_min = np.min(front)
        left_avg = np.mean(left)
        right_avg = np.mean(right)
        back_avg = np.mean(back)

        twist = Twist()

        # إذا كانت الكاميرا متاحة وبتحليل صورة وتكشف عن عقبة
        if self.last_camera_image is not None:
            obstacle_detected = self.detect_obstacle_in_camera(self.last_camera_image)
            
            if obstacle_detected:
                rospy.logwarn("🚧 كاميرا: عقبة تم اكتشافها!")
                self.state = 'REVERSE'
                twist.linear.x = -0.2
                twist.angular.z = 0.0
            else:
                self.state = 'IDLE'

        # إذا كان لا يوجد عقبات مرئية بالكاميرا، نستخدم LiDAR
        if self.state == 'IDLE' and front_min < self.stop_distance:
            rospy.logwarn("🚧 عقبة أمامية!")

            # هنا نضيف قيود حول المسار بحيث نمنع الروبوت من الخروج عن المسار
            if left_avg > self.path_left_limit and right_avg > self.path_right_limit:
                rospy.loginfo("📍 الروبوت يحاول البقاء داخل المسار.")
                direction = 'left' if left_avg > right_avg else 'right'
                twist.angular.z = 0.5 if direction == 'left' else -0.5
                twist.linear.x = 0.0
            else:
                rospy.loginfo("🛑 الروبوت يحاول التراجع أو الالتفاف دون الخروج عن المسار.")
                twist.linear.x = 0.0
                twist.angular.z = 0.5  # يمكن تعديل الزاوية لتجنب الخروج عن المسار

        elif self.state == 'IDLE':
            rospy.loginfo("✅ الطريق آمن – كمل لقدام")
            twist.linear.x = 0.2
            twist.angular.z = 0.0
            self.reverse_mode = False

        self.cmd_pub.publish(twist)

    def detect_obstacle_in_camera(self, cv_image):
        # هنا بنستخدم OpenCV لاكتشاف العوائق. 
        # على سبيل المثال، يمكننا استخدام تقنيات للكشف عن الحواف أو الألوان
        gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
        _, thresholded = cv2.threshold(gray, 150, 255, cv2.THRESH_BINARY)
        contours, _ = cv2.findContours(thresholded, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        # إذا كان عدد العوائق أكبر من حد معين، فهذا يعني أن هناك عقبة.
        if len(contours) > 5:
            return True
        return False

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    SmartDecisionMaker().run()
