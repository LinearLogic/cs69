#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist

RATE = 10 # 10 Hz

class CmdVelPublisher:
    def __init__(self, lin_vel, distance_to_travel):
        self.cmd_vel_pub = rospy.Publisher("cmd_vel", Twist, queue_size=0)
        self.lin_vel = lin_vel
        self.distance_to_travel = distance_to_travel
        self.time_to_travel_secs = distance_to_travel / lin_vel
        self.rate = rospy.Rate(RATE)
        self.state = 0 # 0: stopped, 1: move forward

    def spin(self):
       vel_msg = Twist()
       self.start_timestamp = rospy.get_rostime().secs
       self.state = 1
       print('starting')
       while not rospy.is_shutdown():
           if self.state == 0:
               vel_msg.linear.x = 0
           elif self.state == 1:
               vel_msg.linear.x = self.lin_vel
               print(self.start_timestamp)
               if (rospy.get_rostime().secs - self.start_timestamp > self.time_to_travel_secs):
                   self.state = 0
                   print('stopping')
           self.cmd_vel_pub.publish(vel_msg)

           self.rate.sleep()

if __name__ == "__main__":
    rospy.init_node("cmd_vel_publisher")
    r = CmdVelPublisher(0.2, 1.0) # travel 1.0 meters at 0.2 m/s
    r.spin()
