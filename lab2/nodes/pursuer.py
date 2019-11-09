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
import math

vel = rospy.Publisher('robot_1/cmd_vel', Twist, queue_size=10)
#scan = rospy.Publisher('base_scan', LaserScan, queue_size=30)

def talker():
    rospy.init_node('pursuer', anonymous=True)
    global vel
    global scan
    rospy.Subscriber('robot_1/base_pose_ground_truth', Odometry,handle_Odom)
    move_cmd = Twist()
    listener = tf.TransformListener()
    rate = rospy.Rate(1) # 10hz
    now = rospy.Time.now()
    rate.sleep()
    while not rospy.is_shutdown():
        try:
            (trans,rot) = listener.lookupTransformFull('pursuer', now, 'evader', now, 'world')
        #except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
        
        except tf.LookupException:
            print 'tf.LookupException'
            continue
        except tf.ConnectivityException:
            print 'tf.ConnectivityException'
            continue
        except tf.ExtrapolationException:
            print 'tf.ExtrapolationException'
            continue
    
        move_cmd.linear.x = 0.4
        move_cmd.angular.z = math.atan2(trans[1],trans[0])
        vel.publish(move_cmd)
        rate.sleep()

def handle_Odom(odom):
    br = tf.TransformBroadcaster()
    br.sendTransform((odom.pose.pose.position.x, odom.pose.pose.position.y, odom.pose.pose.position.z),
                     (odom.pose.pose.orientation.x, odom.pose.pose.orientation.y, odom.pose.pose.orientation.z, odom.pose.pose.orientation.w),
                     rospy.Time.now(),
                     "pursuer",
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
	
	
if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
        
    
        

