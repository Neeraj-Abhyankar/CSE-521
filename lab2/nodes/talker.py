#!/usr/bin/env python
# UB Person No 50290958  Name Neeraj Ajit Abhyankar
# license removed for brevity
# for reference http://wiki.ros.org/ROS/Tutorials/WritingPublisherSubscriber%28python%29
#               http://wiki.ros.org/tf/Tutorials/Writing%20a%20tf%20listener%20%28Python%29
#               http://wiki.ros.org/tf/Tutorials/tf%20and%20Time%20%28Python%29
#               http://wiki.ros.org/tf/Tutorials/Adding%20a%20frame%20%28Python%29
#               http://wiki.ros.org/tf/Tutorials/Time%20travel%20with%20tf%20%28Python%29
#               http://wiki.ros.org/ROS/Tutorials/WritingPublisherSubscriber%28python%29
#


import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
import random

vel = rospy.Publisher('cmd_vel', Twist, queue_size=10)
#scan = rospy.Publisher('base_scan', LaserScan, queue_size=30)

def talker():
    rospy.init_node('talker', anonymous=True)
    global vel
    global scan
    sub = rospy.Subscriber('base_scan', LaserScan, callback)
    move_cmd = Twist()
    rate = rospy.Rate(2) # 10hz
    while not rospy.is_shutdown():
        move_cmd.linear.x = 2.0
        move_cmd.linear.y = 0.0
        move_cmd.linear.z = 0.0
        move_cmd.angular.x = 0.0
        move_cmd.angular.y = 0.0
        move_cmd.angular.z = 0.0
        vel.publish(move_cmd)
        rate.sleep()


def callback(data):
	global vel
	#global scan
	#print data
	move_cmd = Twist()
	
	if (sum(data.ranges)/361) < 2.5:
		move_cmd.linear.x = 0.0
		move_cmd.linear.y = 0.0
		move_cmd.linear.z = 0.0
		move_cmd.angular.x = 0.0
		move_cmd.angular.y = 0.0
		move_cmd.angular.z = random.random() * 5


	vel.publish(move_cmd)
	#scan.publish(data.ranges)
		
	

	
	
if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
        
    
        

