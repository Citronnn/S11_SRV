#! /usr/bin/python3

import rospy
import math
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import OccupancyGrid

pub_old = rospy.Publisher('/laser_scan_old', LaserScan, queue_size = 10)
pub_new = rospy.Publisher('/laser_scan_new', LaserScan, queue_size = 10)
pub_grid = rospy.Publisher('/occupancy_grid', OccupancyGrid, queue_size = 10)


dimension = 10
resolution = 0.1

def callback(msg):
    pub_old.publish(msg)
    filtered_data = find_anomalies(msg.ranges, msg.angle_min, msg.angle_increment)
    msg.ranges = filtered_data[0]
    pub_new.publish(msg)
    get_map(filtered_data[1])

def get_map(data):
    og_msg = get_occurancy_grid_msg()
    for point in data:
        if check_borders(point):
            new_x = transform_coords(point[0])
            new_y = transform_coords(point[1])
            og_msg.data = fill_data(og_msg.data, new_x, new_y, 100)
    og_msg.data = remove_unknown(og_msg.data, dimension / 2 - 1, dimension / 2, -1, 1)
    og_msg.data = remove_unknown(og_msg.data, dimension / 2,  dimension / 2, 1, 1)
    og_msg.data = remove_unknown(og_msg.data, dimension / 2 - 1, dimension / 2 - 1, -1, -1)
    og_msg.data = remove_unknown(og_msg.data, dimension / 2, dimension /2 - 1, 1, -1)
    pub_grid.publish(og_msg)

def check_borders(point):
    return abs(point[0]) < (dimension / 2) and abs(point[1]) < (dimension / 2)

def transform_coords(coord):
    return coord + dimension / 2

def fill_data(data, x, y, value):
    for i in range(int(1 / resolution)):
        for j in range(int(1 / resolution)):
            data[get_position_by_coords(x, y, i, j)] = value
    return data

def get_occurancy_grid_msg():
    og_msg = OccupancyGrid()
    width = int(dimension / resolution)
    height = int(dimension / resolution)
    og_msg.header.frame_id = 'base_link'
    og_msg.info.width = width
    og_msg.info.height = height
    og_msg.info.resolution = resolution
    og_msg.info.origin.position.x = -dimension / 2.0
    og_msg.info.origin.position.y = -dimension / 2.0
    og_msg.data = [0] * (width * height)
    return og_msg

def remove_unknown(data, start_x, start_y, direction_x, direction_y):
    for i in range(int(dimension / 2) - 1):
        for j in range(int(dimension / 2) - 1):
            current_value = data[get_position_by_coords(start_x + i * direction_x, start_y + j * direction_y, 0, 0)]
            if current_value != 0:
                first_neighbour = data[get_position_by_coords(start_x + (i + 1) * direction_x, start_y + j * direction_y, 0, 0)]
                if first_neighbour == 0 and j <= i:
                    data = fill_data(data, start_x + (i + 1) * direction_x, start_y + j * direction_y, -1)
                second_neighbour = data[get_position_by_coords(start_x + i * direction_x, start_y + (j + 1) * direction_y, 0, 0)]
                if second_neighbour == 0 and i <= j:
                    data = fill_data(data, start_x + i * direction_x, start_y + (j + 1) * direction_y, -1)
                third_neighbour = data[get_position_by_coords(start_x + (i + 1) * direction_x, start_y + (j + 1) * direction_y, 0, 0)]
                if third_neighbour == 0:
                    data = fill_data(data, start_x + (i + 1) * direction_x, start_y + (j + 1) * direction_y, -1)
    return data


def get_position_by_coords(x, y, row_index, col_index):
    return math.floor(y) * int(dimension / (resolution ** 2)) \
        + row_index * int(dimension / resolution) + math.floor(x) * int(1 / resolution) \
        + col_index + 1 * math.floor(y) 

def get_coords(r, theta):
    return r * math.cos(theta), r * math.sin(theta)

def get_angle(angle_min, angle_increment, position):
    return angle_min + angle_increment * position

def get_dist(p1, p2):
    return math.sqrt((p1[0] - p2[0]) ** 2 + (p1[1] - p2[1]) ** 2)

def find_anomalies(data, angle_min, angle_increment):
    filtered_data = []
    filtered_coords = []
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
            filtered_coords.append(current_point)

    for i in range(step):
        filtered_data.append(huge_number)

    return filtered_data, filtered_coords

rospy.init_node('lr2_advanced')
rospy.Subscriber('base_scan', LaserScan, callback)
r = rospy.Rate(0.5)

while not (rospy.is_shutdown()):
    r.sleep()