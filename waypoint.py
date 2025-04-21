#!/usr/bin/env python3

import rospy
import numpy as np
from sensor_msgs.msg import LaserScan, Imu
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, Point
import geomag
import math

class SmartDecisionMaker:
    def __init__(self):
        rospy.init_node('waypoint_navigation_node')

        # Subscriber للـ LiDAR
        self.scan_sub = rospy.Subscriber('/scan', LaserScan, self.scan_callback)
        
        # Subscriber للـ IMU
        self.imu_sub = rospy.Subscriber('/imu', Imu, self.imu_callback)
        
        # Subscriber للـ Encoder (هنا نستخدم Odometry كمثال)
        self.encoder_sub = rospy.Subscriber('/odom', Odometry, self.encoder_callback)

        self.cmd_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)

        # Initializing the destination (waypoint)
        self.target_point = Point(5.0, 5.0, 0.0)  # وضع الهدف في الإحداثيات (مثال)

        # Path following and LiDAR Variables
        self.stop_distance = 0.6
        self.safe_side_distance = 0.8
        self.reverse_mode = False

        # IMU and GPS Variables
        self.current_orientation = 0.0  # زاوية التوجيه من الـ IMU
        self.current_position = (0.0, 0.0)  # الموقع الحالي من الـ Encoder
        self.previous_position = (0.0, 0.0)

        # GPS and geomag
        self.latitude = None
        self.longitude = None
        self.mag_model = geomag.GeoMag()

    def imu_callback(self, msg):
        # قراءة بيانات التوجيه من الـ IMU
        self.current_orientation = msg.orientation.z  # هنا نفترض أن زاوية التوجيه في المحور z
        rospy.loginfo(f"IMU Orientation: {self.current_orientation}")

        # تصحيح التوجيه باستخدام geomag
        if self.latitude is not None and self.longitude is not None:
            # تصحيح التوجيه باستخدام بيانات الـ GPS
            declination = self.mag_model.declination(self.latitude, self.longitude)
            corrected_orientation = self.current_orientation + declination
            rospy.loginfo(f"Corrected IMU Orientation: {corrected_orientation}")
            self.current_orientation = corrected_orientation

    def encoder_callback(self, msg):
        # قراءة بيانات الموقع من الـ Encoder (نستخدم Odometry هنا كمثال)
        current_position = (msg.pose.pose.position.x, msg.pose.pose.position.y)
        self.current_position = current_position

        # تحديث الإحداثيات الـ GPS بناءً على بيانات الـ Odometry (إذا كانت متاحة)
        self.latitude = msg.pose.pose.position.x  # فرضًا أن البيانات تحتوي على إحداثيات latitude
        self.longitude = msg.pose.pose.position.y  # فرضًا أن البيانات تحتوي على إحداثيات longitude

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

        # إذا كانت هناك عقبة في الطريق
        if front_min < self.stop_distance:
            rospy.logwarn("🚧 عقبة أمامية!")
            # تجنب العقبة باستخدام LiDAR
            if left_avg > self.safe_side_distance and right_avg > self.safe_side_distance:
                rospy.loginfo("↩️↪️ مساحة في الجانبين – نختار التحرك على الجانبين.")
                direction = 'left' if left_avg > right_avg else 'right'
                twist.angular.z = 0.5 if direction == 'left' else -0.5
                twist.linear.x = 0.0
            else:
                rospy.loginfo("🛑 العقبة أمامنا – نقوم بالرجوع أو التفادي.")
                twist.linear.x = -0.2
                twist.angular.z = 0.0
        else:
            # لا توجد عقبات، نكمل في الاتجاه الصحيح نحو النقطة
            rospy.loginfo("✅ الطريق آمن، نكمل السير نحو الهدف.")
            self.move_towards_target(twist)

        self.cmd_pub.publish(twist)

    def move_towards_target(self, twist):
        # حساب الاتجاه بين موقع الروبوت الحالي والهدف
        dx = self.target_point.x - self.current_position[0]
        dy = self.target_point.y - self.current_position[1]
        
        # حساب الزاوية المطلوبة للوجهة
        target_angle = math.atan2(dy, dx)
        angle_diff = target_angle - self.current_orientation

        # تصحيح الزاوية لتكون بين -π و π
        angle_diff = np.arctan2(np.sin(angle_diff), np.cos(angle_diff))

        # تحريك الروبوت نحو الهدف
        rospy.loginfo(f"التوجه نحو الهدف: {target_angle} (زاوية الروبوت: {self.current_orientation})")

        # التحرك للأمام أو التوجيه نحو الهدف
        if abs(angle_diff) > 0.1:
            twist.linear.x = 0.0
            twist.angular.z = 0.5 if angle_diff > 0 else -0.5
        else:
            twist.linear.x = 0.2
            twist.angular.z = 0.0

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    SmartDecisionMaker().run()
