#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
import numpy as np

class SmartDecisionMaker:
    def __init__(self):
        rospy.init_node('smart_decision_node')
        self.scan_sub = rospy.Subscriber('/scan', LaserScan, self.scan_callback)
        self.cmd_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)

        self.stop_distance = 0.6
        self.safe_side_distance = 0.8
        self.reverse_mode = False

    def scan_callback(self, msg):
        ranges = np.array(msg.ranges)
        ranges = np.where(np.isnan(ranges), np.inf, ranges)

        front = np.concatenate((ranges[340:], ranges[:20]))
        left = ranges[60:120]
        right = ranges[240:300]
        back = ranges[160:200]

        front_min = np.min(front)
        left_avg = np.mean(left)
        right_avg = np.mean(right)
        back_avg = np.mean(back)

        twist = Twist()

        if front_min < self.stop_distance:
            rospy.logwarn("🚧 عقبة أمامية!")
            
            if left_avg > self.safe_side_distance and right_avg > self.safe_side_distance:
                direction = 'left' if left_avg > right_avg else 'right'
                rospy.loginfo(f"↩️↪️ مساحة في الجانبين – نختار: {direction}")
                twist.angular.z = 0.5 if direction == 'left' else -0.5
                twist.linear.x = 0.0
            
            elif left_avg > self.safe_side_distance:
                rospy.loginfo("↩️ مساحة على اليسار فقط – لف يسار")
                twist.angular.z = 0.5
                twist.linear.x = 0.0

            elif right_avg > self.safe_side_distance:
                rospy.loginfo("↪️ مساحة على اليمين فقط – لف يمين")
                twist.angular.z = -0.5
                twist.linear.x = 0.0

            elif back_avg > self.safe_side_distance:
                rospy.loginfo("🔙 مفيش مخرج قدام – بنرجع لورا")
                twist.linear.x = -0.2
                twist.angular.z = 0.0
                self.reverse_mode = True
            else:
                rospy.loginfo("🧍 محاصر تمامًا – وقوف")
                twist.linear.x = 0.0
                twist.angular.z = 0.0

        else:
            rospy.loginfo("✅ الطريق آمن – كمل لقدام")
            twist.linear.x = 0.2
            twist.angular.z = 0.0
            self.reverse_mode = False

        self.cmd_pub.publish(twist)

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    SmartDecisionMaker().run()
