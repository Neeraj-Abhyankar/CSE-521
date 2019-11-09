#!/usr/bin/env python
# UB Person No 50290958  Name Neeraj Ajit Abhyankar
# license removed for brevity
# for reference http://wiki.ros.org/ROS/Tutorials/WritingPublisherSubscriber%28python%29
#               http://wiki.ros.org/tf/Tutorials/Writing%20a%20tf%20listener%20%28Python%29
#               http://wiki.ros.org/tf/Tutorials/tf%20and%20Time%20%28Python%29
#               http://wiki.ros.org/tf/Tutorials/Adding%20a%20frame%20%28Python%29
#               http://wiki.ros.org/tf/Tutorials/Time%20travel%20with%20tf%20%28Python%29
#               http://wiki.ros.org/ROS/Tutorials/WritingPublisherSubscriber%28python%29
# https://github.com/ros-visualization/visualization_tutorials/blob/indigo-devel/interactive_marker_tutorials/scripts/simple_marker.py
# https://en.wikipedia.org/wiki/Random_sample_consensus
# https://forum.tutorials7.com/1466/triangle-area-with-given-x-and-y-as-coordinates-java-task
# https://docs.scipy.org/doc/numpy/reference/generated/numpy.arange.html
# https://docs.scipy.org/doc/numpy/reference/generated/numpy.zeros.html
# https://answers.ros.org/question/58443/line-following-in-ros/
# https://www.youtube.com/watch?v=yM2lXifzi4A
# https://www.youtube.com/watch?v=SZBTcLaQIK0
# https://github.com/millere/bugs/blob/master/nodes/bug.py


import rospy, roslib
import math
from geometry_msgs.msg import Twist, Point
from sensor_msgs.msg import LaserScan
import random
import numpy as np
import tf
from nav_msgs.msg import Odometry
from visualization_msgs.msg import Marker
import tf.transformations as transform

pub = rospy.Publisher("/robot_0/cmd_vel", Twist, queue_size=10)
mark = rospy.Publisher('visualization_msgs', Marker, queue_size=100)
My_position = 0
My_orientation = 0
threshold = 0.2
points = []
points2 = []
wall = False
in_lied = False
line_to_follow = False
ranges = np.zeros((361, 2))

def talker():
    global vel
    global scan
    rospy.init_node('talker', anonymous=True)
    vel = Twist()
    rospy.Subscriber('/robot_0/base_scan', LaserScan, callback)
    Bug_Main()

    sub = rospy.Subscriber('/robot0/base_scan', LaserScan, marker_code)
    rospy.spin()

def Bug_Main():
    global wall, My_orientation, My_position, in_lied
    odom = rospy.Subscriber('/robot_0/base_pose_ground_truth', Odometry, handle_Odom)
    #pub = rospy.Publisher("/robot_0/cmd_vel", Twist, queue_size=10)
    destination = False
    points2 = np.array([[-8, -2], [4.5, 9.0]])
    distance = math.sqrt((points2[1, 1] - 0) ** 2 + (points2[1, 0] - 0) ** 2)
    threshold_distance_of_robot = 0.7
    #vel = Twist()
    follow = "Goal_Seek"
    rate = rospy.Rate(1)
    #vel.angular.z = 0.5
    while not destination:
        if My_orientation != 0:
            My_angle = 2 * math.asin(My_orientation.z)
            angle_required = math.atan((points2[1, 1] - My_position.y) / (points2[1, 0] - My_position.x)) - My_angle
            distance_temp = math.sqrt((points2[1, 1] - My_position.y) ** 2 + (points2[1, 0] - My_position.x) ** 2)
            vel = Twist()

            if distance_temp < threshold_distance_of_robot:
                vel.linear.x = 0.0
                vel.linear.y = 0.0
                vel.linear.z = 0.0
                vel.angular.x = 0.0
                vel.angular.y = 0.0
                vel.angular.z = 0.0
                break
            else:
                vel.linear.x = 0.0
                if wall:
                    vel.linear.x = 0.0
                    vel.linear.y = 0.0
                    vel.linear.z = 0.0
                    vel.angular.x = 0.0
                    vel.angular.y = 0.0
                    vel.angular.z = 0.0
                else:
                    vel.linear.x = 1.0

                if follow == "Goal_Seek":
                    vel.angular.z = -1 * vel_ang_to_follow(angle_required)
                    if wall:
                        follow = "Wall_Follow"
                else:
                    vel.angular.z = vel_ang_to_follow(angle_required)
                    if in_lied and not wall:
                        follow = "Goal_Seek"

            pub.publish(vel)
            rate.sleep()

def handle_Odom(odom):
    global My_orientation
    global My_position
    My_position = odom.pose.pose.position
    My_orientation = odom.pose.pose.orientation

def velocity_angle(angle_required):
    global wall, in_lied
    if in_lied:
        return min(in_lied, 1)
    elif wall:
        return 1


def if_in_line ():
    global My_position
    #area of triangle to see if point is on line
    a = points2[0,:]
    b = points2[1,:]
    c = np.array([My_position.x, My_position.y])
    area = abs((a[0] * (b[1] - c[1]) + b[0] * (c[1] - a[1]) + c[0] * (a[1] - b[1])) / 2.0)
    threshold = 0.8
    if (area < threshold):
        return True
    else:
        return False

def vel_ang_to_follow(angle_required):
    global wall, line_to_follow
    if wall:
        return 0.6
    elif line_to_follow:
        return 0.0
    else:
        return 0.5 * -1




def callback(data):
    global ranges, wall, line_to_follow
    range = data.ranges
    range = np.array(range)
    count_1 = 0
    count_max = 140
    for i in  np.arange(count_max):
        if range [180-(count_max/2)+1] < 1:
            count_1 +=1
    if count_1 > 1:
        wall = True
    else:
        wall = False

    count_1 = 0
    count_max = 100

    for i in np.arange(count_max):
        if range[i] < 1:
            count_1 +=1
    if count_1 > 8 :
        line_to_follow = True
    else :
        line_to_follow = False


def marker_code(LaserScan):
    global mark
    laser_in = LaserScan.ranges
    inlier_count = []
    outlier_count = []
    inlier_points = []
    outlier_points = []
    inliers = []
    outliers = []
    for i in range(0, len(laser_in)):

        global r, theta,points
        p = Point()

        theta = (i / 2 - 90)

        if laser_in[i] == 3.0:
            continue

        x = laser_in[i] * math.cos(math.radians(theta))
        y = laser_in[i] * math.sin(math.radians(theta))
        p.x = x
        p.y = y
        p.z = 0
        points.append(p)
    counter_in = 0
    counter_out = 0
    pair = []
    pair_count = []
    max_count = 0
    for k in range(50):
        p1 = random.choice(points)
        p2 = random.choice(points)
        if p1.x == p2.x:
            continue
        if (p2.x - p1.x) != 0:
            slope = ((p2.y - p1.y) / (p2.x - p1.x))
            c = p2.y - (slope * p2.x)
        middle_points = []
        for i in points:
            if i.x > min(p1.x, p2.x) and i.x < max(p1.x, p2.x):
                middle_points.append(i)
        for b in middle_points:
            dist = 10
            if (p2.x - p1.x) != 0:
                dist = abs((p2.y - p1.y) * b.x - (p2.x - p1.x) * b.y + p2.x * p1.y - p2.y * p1.x) / math.sqrt(
                    (p2.y - p1.y) * (p2.y - p1.y) + (p2.x - p1.x) * (p2.x - p1.x))
            if dist < threshold:
                counter_in += 1
        pair.append([p1, p2])
        pair_count.append(counter_in)
    final_pair = pair[pair_count.index(max(pair_count))]

    rate = rospy.Rate(1)  # 10hz

    marker = Marker()
    marker.header.frame_id = "robot_0/base_link"
    marker.ns = "hi"
    marker.header.stamp = rospy.Time.now()
    marker.type = Marker.LINE_LIST
    marker.action = Marker.ADD
    marker.scale.x = 0.45
    marker.scale.y = 0.45
    marker.scale.z = 0.45
    marker.color.r = 0.1
    marker.color.g = 0.3
    marker.color.b = 0.5
    marker.color.a = 1.0
    marker.lifetime = rospy.Duration()
    # pp = Point()
    # pp.x = final_pair
    # pp.y = final_pair[1]
    print ("pair1" + str(final_pair[0]))
    print ("pair 2: "+ str(final_pair[1]))
    marker.points.append(final_pair[0])
    marker.points.append(final_pair[1])
    mark.publish(marker)
    points = []


if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
