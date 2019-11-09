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
import tf
from nav_msgs.msg import Odometry

vel = rospy.Publisher('robot_0/cmd_vel', Twist, queue_size=10)
#scan = rospy.Publisher('base_scan', LaserScan, queue_size=30)

def talker():
    rospy.init_node('talker', anonymous=True)
    global vel
    global scan
    sub = rospy.Subscriber('robot_0/base_scan', LaserScan, callback)
    rospy.Subscriber('robot_0/base_pose_ground_truth', Odometry,handle_Odom)
    move_cmd = Twist()
    rate = rospy.Rate(4) # 10hz
    while not rospy.is_shutdown():
        move_cmd.linear.x = 2.0
        move_cmd.linear.y = 0.0
        move_cmd.linear.z = 0.0
        move_cmd.angular.x = 0.0
        move_cmd.angular.y = 0.0
        move_cmd.angular.z = 0.0
        vel.publish(move_cmd)
        rate.sleep()

def handle_Odom(odom):
    br = tf.TransformBroadcaster()
    br.sendTransform((odom.pose.pose.position.x, odom.pose.pose.position.y, odom.pose.pose.position.z),
                     (odom.pose.pose.orientation.x, odom.pose.pose.orientation.y, odom.pose.pose.orientation.z, odom.pose.pose.orientation.w),
                     rospy.Time.now(),
                     "evader",
                     "world")

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
        
    
        

