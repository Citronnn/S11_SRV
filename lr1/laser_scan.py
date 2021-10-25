#! /usr/bin/python3

import rospy
import math
from sensor_msgs.msg import LaserScan

pub_old = rospy.Publisher('/laser_scan_old', LaserScan, queue_size = 10)
pub_new = rospy.Publisher('/laser_scan_new', LaserScan, queue_size = 10)

def callback(msg):
    pub_old.publish(msg)
    msg.ranges = find_anomalies(msg.ranges, msg.angle_min, msg.angle_increment)
    pub_new.publish(msg)

def get_coords(r, theta):
    return r * math.cos(theta), r * math.sin(theta)

def get_angle(angle_min, angle_increment, position):
    return angle_min + angle_increment * position

def get_dist(p1, p2):
    return math.sqrt((p1[0] - p2[0]) ** 2 + (p1[1] - p2[1]) ** 2)

def find_anomalies(data, angle_min, angle_increment):
    filtered_data = []
    step = 3
    huge_number = 9999.0
    max_dist = 0.1 * step

    for i in range(step):
        filtered_data.append(huge_number)

    for i in range(step, len(data) - step):
        prev_step_point = get_coords(data[i - step], get_angle(angle_min, angle_increment, i - step))
        current_point = get_coords(data[i], get_angle(angle_min, angle_increment, i))
        next_step_point = get_coords(data[i + step], get_angle(angle_min, angle_increment, i + step))

        if get_dist(prev_step_point, current_point) > max_dist or get_dist(next_step_point, current_point) > max_dist:
            filtered_data.append(huge_number)
        else:
            filtered_data.append(data[i])

    for i in range(step):
        filtered_data.append(huge_number)

    return filtered_data

rospy.init_node('lr1_normal')
rospy.Subscriber('base_scan', LaserScan, callback)
r = rospy.Rate(0.5)

while not (rospy.is_shutdown()):
    r.sleep()