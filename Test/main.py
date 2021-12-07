import os
from math import cos, sin, pi, floor
import pygame
from rplidar import RPLidar
import os
import time

# Set up pygame and the display
# os.putenv('SDL_FBDEV', '/dev/fb1')
# pygame.init()
# lcd = pygame.display.set_mode((320, 240))
# pygame.mouse.set_visible(False)
# lcd.fill((0, 0, 0))
# pygame.display.update()

# Setup the RPLidar
from Point import Point
import numpy as np

PORT_NAME = 'COM3'
lidar = RPLidar(PORT_NAME)

# used to scale data to fit on the screen
max_distance = 0
max_dist = 0
robot_current_position = (0, 0)


# pylint: disable=redefined-outer-name,global-statement
# def process_data(data):
#     global max_distance
#     lcd.fill((0, 0, 0))
#     for angle1 in range(360):
#         distance1 = data[angle1]
#         if distance1 > 0:  # ignore initially ungathered data points
#             max_distance = max([min([5000, distance1]), max_distance])
#             radians = angle1 * pi / 180.0
#             x = distance1 * cos(radians)
#             y = distance1 * sin(radians)
#             point = (160 + int(x / max_distance * 119), 120 + int(y / max_distance * 119))
#             lcd.set_at(point, pygame.Color(255, 255, 255))
#     pygame.display.update()


def return_list_data(data):
    global max_dist
    points_list = []
    for angle1 in range(360):
        distance1 = data[angle1]
        if distance1 > 0:  # ignore initially ungathered data points
            max_dist = max([min([5000, distance1]), max_dist])
            x = distance1 * cos(angle)
            y = distance1 * sin(angle)
            point = Point(float(format(x, ".2f")), float(format(y, ".2f")))
            points_list.append(point)
    return points_list


def get_bottom_left_corner(points_list):
    min_x = min((point.obstacles_points for point in points_list))
    min_y = min((point.y for point in points_list))
    return min_x, min_y


def get_top_right_corner(points_list):
    max_x = max((point.obstacles_points for point in points_list))
    max_y = max((point.y for point in points_list))
    return max_x, max_y


def generate_room(bottom_left, top_right, file1):
    entire_room_points = []
    for x in np.arange(bottom_left.obstacles_points, top_right.obstacles_points + 1, 1):
        for y in np.arange(bottom_left.y, top_right.y + 1, 1):
            new_point = Point(format(x, ".2f"), format(y, ".2f"))
            entire_room_points.append(new_point)
            file1.write(str(new_point) + "\n")
    file1.close()
    return entire_room_points


scan_data = [0] * 360

try:
    print(lidar.get_info())
    obstacles_points = []
    for i, scan in enumerate(lidar.iter_scans()):
        print('Iteration: %d, ' % i)
        if i > 5:
            break
        # print('%d: Got %d measurments' % (i, len(scan)))
        for (_, angle, distance) in scan:
            scan_data[min([359, floor(angle)])] = distance
            print('quality: %d \n angle: %d \n distance: %d' % (_, angle, distance))
        obstacles_points = return_list_data(scan_data)
        # calculez traiectoria intre start si finish

        bottom_left = Point(get_bottom_left_corner(obstacles_points)[0], get_bottom_left_corner(obstacles_points)[1])
        top_right = Point(get_top_right_corner(obstacles_points)[0], get_top_right_corner(obstacles_points)[1])
        print("Bottom left corner is: ", bottom_left)
        print("Top right corner is: ", top_right)
        print("Horizontal line: ", top_right.x - bottom_left.x)
        print("Vertical line: ", top_right.y - bottom_left.y)

        # generare puncte din camera de fiecare data la fiecare iteratie
        file1 = open("room.txt", 'w')
        file1.truncate()
        time.sleep(1.5)
        entire_room_without_obstacles = generate_room(bottom_left, top_right, file1)


        print("test")

except KeyboardInterrupt:
    print('Stopping.')
lidar.stop()
lidar.disconnect()
