#!/usr/bin/env python

import rospy
from nav_msgs.msg import Odometry
from matplotlib import pyplot

RATE = 10 # 10 Hz

class OdomSubscriber:
    def __init__(self):
        self.odom_sub = rospy.Subscriber("odom", Odometry, self.odom_callback, queue_size=1)
        self.rate = rospy.Rate(RATE)

    def odom_callback(self, msg):
        pose = msg.pose.pose
        plt.plot(pose.position.y, pose.position.x, '*')
        plt.axis("equal")
        plt.draw()

    def spin(self):
       while not rospy.is_shutdown():
           self.rate.sleep()

if __name__ == "__main__":
    rospy.init_node("odom_subscriber")
    r = OdomSubscriber()
    r.spin()
