#!/usr/bin/env python2
# -*- coding: utf-8 -*-
import rospy
import time
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped
import math

def callback(msg):
	bolota = 1

class FollowTheGap(object):

    

    def __init__(self):

        #rospy.init_node('new_lego_team_node', anonymous = False)

        #self.sub = rospy.Subscriber('/scan', LaserScan, callback)
	#self.sub = rospy.Subscriber('/new_lego_team_id/scan', LaserScan, callback)
	direc_msg = AckermannDriveStamped()
        rate = rospy.Rate(40)

        while not rospy.is_shutdown():

		direc_msg.drive.speed = 0.1
		direc_msg.drive.steering_angle = 0
		gap_pub.publish(direc_msg)
		rate.sleep()


def main():
	rospy.init_node('new_lego_team_node', anonymous = False)

	#self.sub = rospy.Subscriber('/scan', LaserScan, callback)
	sub = rospy.Subscriber('/scan', LaserScan, callback)

	fg = FollowTheGap()
	rospy.spin()

#gap_pub = rospy.Publisher('/drive', AckermannDriveStamped, queue_size=10)
gap_pub = rospy.Publisher('/drive', AckermannDriveStamped, queue_size=10)

main()
