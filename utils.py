import math
from shapely.geometry import Polygon, Point
from apolo import Apolo
import matplotlib.pyplot as plt
import numpy as np
import random


def random_point_in_area(area):
    "This function is used to generate a random point within a given area, used to randomly place the victim"
    margin = 0.4
    x_min = area[0] + margin
    x_max = area[1] - margin
    y_min = area[2] + margin
    y_max = area[3] - margin
    x = random.uniform(x_min, x_max)
    y = random.uniform(y_min, y_max)
    return x, y


def place_victim(ap=None):
    "This function is used to place the victim in a random area"
    areas = [
    (-1, 1, 4.5, 9),  # corridor
    (1, 4, 7, 9),  # end of corridor
    (2.3, 3.8, 1, 5.5),  # kitchen
    (-5, -1, 0.5, 1.5),  # living room 1
    (-3.4, -1, 1.5, 6),  # living room 2
    (-4, -2, 6, 10),  # bedroom 1
    (-2, 0.5, 9, 11),  # bedroom 2
    (-0.5, 2.3, 11, 12),  # bathroom 1
    (2, 5, 9.6, 11),  # bathroom 2
]
   
    chosen_area = random.choice(areas)
    x, y = random_point_in_area(chosen_area)
    ap.placeObject('Patient', x, y, 0.4, 0, 0, 0)
    ap.placeObject('arrow', x, y, 0.6, math.pi/2, 0, 0)
    ap.updateWorld()
    return 


def euclidean_distance(x1, y1, x2, y2):
    return math.sqrt((x1 - x2) ** 2 + (y1 - y2) ** 2)


def normalize_angle(angle):
    "This function is used to normalize an angle to the range [-pi, pi]"
    return (angle + math.pi) % (2 * math.pi) - math.pi


def angle_difference(x, y, Xk):
    "This function is used to calculate the angle difference between the robot and a target point, it returns heading error to be aligned with the target"
    x_robot, y_robot, yaw = Xk
    target_angle = math.atan2(y - y_robot, x - x_robot)
    return normalize_angle(target_angle - normalize_angle(yaw))


def detect_balisa(robot, target_id=None, ap=None):
    "This function is used to detect a balisa in the environment with a specific ID, used to detect the victim"
    landmarks = ap.getLaserLandMarks("LMS100")
    for landmark in landmarks:
        if target_id is None or landmark[0] == target_id:  # Check for specific balisa ID
            theta = landmark[1]
            r = landmark[2]
            x_robot, y_robot, _, robot_angle = ap.getLocationWB(robot)
            # Calculate balisa position
            x_balisa = x_robot + r * math.cos(robot_angle + theta)
            y_balisa = y_robot + r * math.sin(robot_angle + theta)
            return True, x_balisa, y_balisa
    return False, None, None


def calculate_obstacle_points(lidar_data, x_robot, y_robot, yaw):
    "This function is used to calculate the obstacle points detected by the lidar, transforming the polar coordinates to cartesian"
    obstacle_points = []
    for i, distance in enumerate(lidar_data):
        angle = (225 - i/len(lidar_data) * 270) * math.pi / 180
        x_r = distance * math.sin(angle)
        y_r = distance * math.cos(angle)
        x_0 = x_robot + x_r * math.cos(yaw) - y_r * math.sin(yaw)
        y_0 = y_robot + x_r * math.sin(yaw) + y_r * math.cos(yaw)
      
        obstacle_points.append((x_0, y_0))
    return obstacle_points

def plot_points(points, intermediate_point, centroid, most_left, most_right, robot_pos, target_pos, margins):
    "Debugging function used to plot when the reactive control is activated"

    plt.subplot(1, 2, 1)
    
    # actual map
    plt.plot([0.5, 5], [0, 0], color='black', label='Actual map')
    plt.plot([-0.5, -5], [0, 0], color='black')
    plt.plot([5, 5], [0, 12], color='black')
    plt.plot([-5, -5], [0, 12], color='black')
    plt.plot([-1, -5], [6, 6], color='black')
    plt.plot([1, 5], [7, 7], color='black')
    plt.plot([1, 1], [0, 1], color='black')
    plt.plot([1, 1], [2, 7], color='black')
    plt.plot([-1, -1], [0, 4.5], color='black')
    plt.plot([-1, -1], [5.5, 6.5], color='black')
    plt.plot([-1, -1], [7.5, 9], color='black')
    plt.plot([-1, 5], [9, 9], color='black')
    plt.plot([-5, 5], [12, 12], color='black')
    
    x_coords, y_coords = zip(*points)
   
    plt.scatter(x_coords, y_coords, label='Points detected by lidar')
    
    x_margin, y_margin = margins.exterior.xy
    
    plt.plot(x_margin, y_margin, color='green', label='Prediction polygon')

    if target_pos is not None:
        plt.scatter(target_pos[0], target_pos[1], color='pink', label='Goal point')
    if robot_pos is not None:
        plt.scatter(robot_pos[0], robot_pos[1], color='gray', label='Current robot position')
    if intermediate_point is not None:
        plt.scatter(intermediate_point[0], intermediate_point[1], color='red', label='Intermediate point')
        plt.plot([intermediate_point[0], target_pos[0]], [intermediate_point[1], target_pos[1]], color='orange', label='New path')
        plt.plot([robot_pos[0], intermediate_point[0]], [robot_pos[1], intermediate_point[1]], color='orange')
    if centroid is not None:
        plt.scatter(centroid[0], centroid[1], color='lightblue', label='Centroid')
    if most_left is not None:
        plt.scatter(most_left[0], most_left[1], color='yellow', label='Obstacle most left point')
    if most_right is not None:
        plt.scatter(most_right[0], most_right[1], color='lightgreen', label='Obstacle most right point')

    plt.axis('equal')
    plt.xlim(-5.5, 5.5)
    plt.ylim(-0.5, 12.5)
    plt.xlabel('X')
    plt.ylabel('Y')
    plt.title('Obstacle Points and Margins - total map')
    plt.legend(loc='upper left')

    # same plot but
    plt.subplot(1, 2, 2)

    # actual map
    plt.plot([0.5, 5], [0, 0], color='black', label='Actual map')
    plt.plot([-0.5, -5], [0, 0], color='black')
    plt.plot([5, 5], [0, 12], color='black')
    plt.plot([-5, -5], [0, 12], color='black')
    plt.plot([-1, -5], [6, 6], color='black')
    plt.plot([1, 5], [7, 7], color='black')
    plt.plot([1, 1], [0, 1], color='black')
    plt.plot([1, 1], [2, 7], color='black')
    plt.plot([-1, -1], [0, 4.5], color='black')
    plt.plot([-1, -1], [5.5, 6.5], color='black')
    plt.plot([-1, -1], [7.5, 9], color='black')
    plt.plot([-1, 5], [9, 9], color='black')
    plt.plot([-5, 5], [12, 12], color='black')
    
    x_coords, y_coords = zip(*points)
   
    plt.scatter(x_coords, y_coords, label='Points detected by lidar')
    
    x_margin, y_margin = margins.exterior.xy
    
    plt.plot(x_margin, y_margin, color='green', label='Prediction polygon')

    if target_pos is not None:
        plt.scatter(target_pos[0], target_pos[1], color='pink', label='Goal point')
    if robot_pos is not None:
        plt.scatter(robot_pos[0], robot_pos[1], color='gray', label='Current robot position')
    if intermediate_point is not None:
        plt.scatter(intermediate_point[0], intermediate_point[1], color='red', label='Intermediate point')
        plt.plot([intermediate_point[0], target_pos[0]], [intermediate_point[1], target_pos[1]], color='orange', label='New path')
        plt.plot([robot_pos[0], intermediate_point[0]], [robot_pos[1], intermediate_point[1]], color='orange')
    if centroid is not None:
        plt.scatter(centroid[0], centroid[1], color='lightblue', label='Centroid')
    if most_left is not None:
        plt.scatter(most_left[0], most_left[1], color='yellow', label='Obstacle most left point')
    if most_right is not None:
        plt.scatter(most_right[0], most_right[1], color='lightgreen', label='Obstacle most right point')
    

    plt.axis('equal') 
    plt.xlim(-1 + centroid[0], 1 + centroid[0])
    plt.ylim(-1 + centroid[1], 1 + centroid[1])
    plt.xlabel('X')
    plt.ylabel('Y')
    plt.title('Obstacle Points and Margins - close up')
    plt.legend(loc='upper left')

    plt.show()

def filter_points(lidar_data, x_robot, y_robot, yaw, margins):
    "This function is used to filter the obstacle points detected by the lidar, selecting only the points relevant to the reactive control"

    obstacle_points = calculate_obstacle_points(lidar_data, x_robot, y_robot, yaw)

    points_in_margin = []
    centroid = None
    most_right = None
    most_left = None

    for point in obstacle_points:
        if margins.contains(Point(point)):
            #distance = math.sqrt((point[0] - x_robot) ** 2 + (point[1] - y_robot) ** 2)
            points_in_margin.append(point)

    if points_in_margin:
        
        trajectory_vector = (math.cos(yaw), math.sin(yaw))
        min_sin = 2
        max_sin = -2
        xc = 0
        yc = 0

        # Find the most left and most right points, and the centroid
        for i in range(len(points_in_margin)):
            vector = (points_in_margin[i][0] - x_robot, points_in_margin[i][1] - y_robot)
            cross_product = trajectory_vector[0] * vector[1] - trajectory_vector[1] * vector[0]
            if min_sin > cross_product:
                min_sin = cross_product
                most_right = points_in_margin[i]
            elif max_sin < cross_product:
                max_sin = cross_product
                most_left = points_in_margin[i]
            xc += points_in_margin[i][0]
            yc += points_in_margin[i][1]

        xc /= len(points_in_margin)
        yc /= len(points_in_margin)
        centroid = (xc,yc)

    return centroid, most_left, most_right
    

def calculate_intermediate_point(x_robot, y_robot, yaw, centroid, most_left, most_right, offset):
    "This function is used to calculate the intermediate point to avoid the obstacle"

    # Compute the trajectory vector from yaw
    trajectory_vector = (math.cos(yaw), math.sin(yaw))

    # Vector from robot to centroid
    to_centroid_vector = (centroid[0] - x_robot, centroid[1] - y_robot)

    # Compute the cross product
    cross_product = trajectory_vector[0] * to_centroid_vector[1] - trajectory_vector[1] * to_centroid_vector[0]
    
    # Determine direction to correct based on cross product
    if cross_product > 0:  # Closest point is on the left
        print("Obstacle on the left, going right")
        correcting_angle = yaw - math.pi / 2  # Adjust to the right
       
        # Calculate the intermediate point
        intermediate_point = (
        most_right[0] + offset * math.cos(correcting_angle),
        most_right[1] + offset * math.sin(correcting_angle)
        )

    else:  # Closest point is on the right
        print("Obstacle on the right, going left")
        correcting_angle = yaw + math.pi / 2  # Adjust to the left
       
        # Calculate the intermediate point
        intermediate_point = (
        most_left[0] + offset * math.cos(correcting_angle),
        most_left[1] + offset * math.sin(correcting_angle)
        )
    
    return intermediate_point


def avoid_obstacle(robot, target_x, target_y, safety_margin, offset, ap=None):
    "This function is used to avoid obstacles in the environment, calculating the intermediate point to avoid the obstacle it calls many helper functions, see above"

    obstacle_detected = False
    lidar_data = ap.getLaserData("LMS100")
    ap.updateWorld()
    x_robot, y_robot, _, yaw = ap.getLocationWB(robot)

    # Calculate vector to target
    vector_x = target_x - x_robot
    vector_y = target_y - y_robot
    vector_length = math.sqrt(vector_x ** 2 + vector_y ** 2)

    # Normalize vector
    vector_x /= vector_length
    vector_y /= vector_length

    # Limit the distance to a maximum of 2 meters
    limited_distance = min(3, vector_length)
    new_target_x = x_robot + vector_x * limited_distance
    new_target_y = y_robot + vector_y * limited_distance

    # Calculate safety margin around the path
    margin_x1 = x_robot + safety_margin * vector_y
    margin_y1 = y_robot - safety_margin * vector_x
    margin_x2 = new_target_x + safety_margin * vector_y
    margin_y2 = new_target_y - safety_margin * vector_x

    margin_x3 = x_robot - safety_margin * vector_y
    margin_y3 = y_robot + safety_margin * vector_x
    margin_x4 = new_target_x - safety_margin * vector_y
    margin_y4 = new_target_y + safety_margin * vector_x

    # Create the polygon using the limited target position
    margins = Polygon([(margin_x1, margin_y1), (margin_x2, margin_y2), (margin_x4, margin_y4), (margin_x3, margin_y3)])
   
    centroid, most_left, most_right = filter_points(lidar_data, x_robot, y_robot, yaw, margins)

    if centroid is None or most_left is None or most_right is None: # No obstacle detected
        return False, None, None

    intermediate_point = calculate_intermediate_point(x_robot, y_robot, yaw, centroid, most_left, most_right, offset)
    
    if intermediate_point:
        obstacle_detected = True
        print(f"Intermediate point: {intermediate_point}")
        robot_pos = (x_robot, y_robot)
        target_pos = (target_x, target_y)
        plot_points(calculate_obstacle_points(lidar_data, x_robot, y_robot, yaw), intermediate_point, centroid, most_left, most_right, robot_pos, target_pos, margins)

    return obstacle_detected, intermediate_point[0], intermediate_point[1]
