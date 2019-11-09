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
# http://wiki.ros.org/rviz/DisplayTypes/Marker
# https://www.youtube.com/watch?v=SZBTcLaQIK0
#
# https://en.wikipedia.org/wiki/Random_sample_consensus
# http://ros-developer.com/tag/ransac/
# http://robots.stanford.edu/teichman/repos/track_classification/src/ros-pkg/point_cloud_mapping/src/sample_consensus/ransac.cpp
# https://www.cs.cmu.edu/~motionplanning/lecture/Chap2-Bug-Alg_howie.pdf
# https://forum.tutorials7.com/1466/triangle-area-with-given-x-and-y-as-coordinates-java-task
# https://docs.scipy.org/doc/numpy/reference/generated/numpy.arange.html
# https://docs.scipy.org/doc/numpy/reference/generated/numpy.zeros.html
# https://answers.ros.org/question/58443/line-following-in-ros/
# https://www.youtube.com/watch?v=yM2lXifzi4A
# https://github.com/millere/bugs/blob/master/nodes/bug.py

import rospy, roslib
import math
from geometry_msgs.msg import Twist, Point
from sensor_msgs.msg import LaserScan
import random
import numpy as np
import tf
from visualization_msgs.msg import Marker

vel = rospy.Publisher('cmd_vel', Twist, queue_size=100)
mark = rospy.Publisher('visualization_msgs', Marker, queue_size=100)
# scan = rospy.Publisher('base_scan', LaserScan, queue_size=30)
threshold = 0.2
points = []


def talker():
    rospy.init_node('talker', anonymous=True)

    global vel
    global scan
    sub = rospy.Subscriber('base_scan', LaserScan, marker_code)
    rospy.spin()

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
    marker.header.frame_id = "base_link"
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
