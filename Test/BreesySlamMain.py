import rplidar
from Point import Point
from scipy.ndimage import rotate
from imageio import imread
import csv
import operator
import cv2

import numpy as np
import pandas as pd
from numpy import sin, cos

MAP_SIZE_PIXELS = 250  # 500
MAP_SIZE_METERS = 5  # 10
LIDAR_DEVICE = 'COM3'

# Ideally we could use all 250 or so samples that the RPLidar delivers in one
# scan, but on slower computers you'll get an empty map and unchanging position
# at that rate.
MIN_SAMPLES = 180  # 200

from breezyslam.algorithms import RMHC_SLAM
from breezyslam.sensors import RPLidarA1 as LaserModel
from rplidar import RPLidar as Lidar, RPLidar
from roboviz import MapVisualizer

from PIL import ImageTk, Image
import tkinter as tk
from tkinter import filedialog as fd
import os

import matplotlib.pyplot as plt
import matplotlib.image as mpimg
import turtle as trt

import time
import math

# def figure(matrix):
#     point_list = []
#     for i in range(0, matrix.shape[0]):
#         for j in range(0, matrix.shape[1]):
#             if matrix[i][j] == 1:
#                 point_list.append((i, j))
#
#     point_list = np.asarray(point_list)
#     return point_list
#
#
# def draw_dots(matrix):
#     points = figure(matrix)
#     root = tk.Tk()
#     root.title('Python Turtle')
#
#     Width, Height = matrix.shape[0], matrix.shape[1]
#     canvas = tk.Canvas(root, width=Width * 2, height=Height * 2)
#     canvas.pack()
#
#     turtle = trt.RawTurtle(canvas=canvas)
#     ts = turtle.getscreen()
#     ts.tracer(0)
#     for point in points:
#         turtle.penup()
#         dx, dy = point[0] - Width / 2, point[1] - Height / 2
#         turtle.goto(dx, dy)
#         turtle.pendown()
#         turtle.dot(1)
#     trt.update()
#     # eps="canvas.eps"
#     # ts.getcanvas().postscript(file=eps)
#     # print('saved', eps)
#     # img = Image.open(eps)  ##  use PIL to convert to PNG
#     # fig = img.convert('RGBA')
#     # fig.save("pengeu.png", lossless=True)
#     filename = 'penegeu'
#     directory = fd.askdirectory(initialdir='', title='Choose folder')
#     fullpath = os.path.join(directory, filename)
#     eps, png = f'{fullpath}.eps', f'{fullpath}.png'
#     canvas.postscript(file=eps, pagewidth=Width - 1, pageheight=Height - 1)
#     print('saved', eps)
#     img = Image.open(eps)  ##  use PIL to convert to PNG
#     img.save(png, 'png', optimize=True, compress_level=9)


if __name__ == '__main__':

    # Connect to Lidar unit
    lidar = Lidar(LIDAR_DEVICE)

    # Create an RMHC SLAM object with a laser model and optional robot model
    slam = RMHC_SLAM(LaserModel(), MAP_SIZE_PIXELS, MAP_SIZE_METERS)

    # Set up a SLAM display
    viz = MapVisualizer(MAP_SIZE_PIXELS, MAP_SIZE_METERS, 'SLAM')

    # Initialize an empty trajectory
    trajectory = []

    # Initialize empty map
    mapbytes = bytearray(MAP_SIZE_PIXELS * MAP_SIZE_PIXELS)

    # Create an iterator to collect scan data from the RPLidar
    iterator = lidar.iter_scans()

    # We will use these to store previous scan in case current scan is inadequate
    previous_distances = None
    previous_angles = None

    # First scan is crap, so ignore it
    next(iterator)

    while True:

        # Extract (quality, angle, distance) triples from current scan
        items = [item for item in next(iterator, 'end')]
        # Extract distances and angles from triples
        distances = [item[2] for item in items]
        angles = [item[1] for item in items]
        quality = [item[0] for item in items]
        # Update SLAM with current Lidar scan and scan angles if adequate
        if len(distances) > MIN_SAMPLES:
            slam.update(distances, scan_angles_degrees=angles)
            previous_distances = distances.copy()
            previous_angles = angles.copy()
        # If not adequate, use previous
        elif previous_distances is not None:
            slam.update(previous_distances, scan_angles_degrees=previous_angles)
        # Get current robot position
        x, y, theta = slam.getpos()
        print("x= ", x)
        print("y= ", y)
        # Get current map bytes as grayscale
        slam.getmap(mapbytes)
        print(len(mapbytes))
        np_map_bytes = np.asarray(mapbytes)
        # create image
        reshaped_map_bytes = np_map_bytes.reshape((MAP_SIZE_PIXELS, MAP_SIZE_PIXELS, 1))
        print(np.shape(reshaped_map_bytes))
        # img_test = Image.fromarray((reshaped_map_bytes * 255).astype(np.uint8))
        # print("HEy there" + np.shape(img_test))
        # img_test.show()
        # img_test.save('tibi.png')
        # Scrierea asta in fisier a array-ului 3D reshaped_map_bytes2 face ca si iterarea lidarului sa crape dupa una singura si nu se stie de ce
        # with open("empty_rows.csv", 'w', newline='') as f:
        #     write = csv.writer(f, delimiter='\n')
        #     write.writerows(reshaped_map_bytes2)
        #     f.close()
        obstacle_matrix = np.zeros(shape=(reshaped_map_bytes2.shape[0], reshaped_map_bytes2.shape[1]), dtype=int)
        white_pixel_min = np.array([200, 200, 200])
        white_pixel_max = np.array([255, 255, 255])
        for index, values in np.ndenumerate(reshaped_map_bytes2):
            comparison = (white_pixel_min <= values).all() and (values <= white_pixel_max).all()
            if comparison:
                obstacle_matrix[index[0]][index[1]] = 1
            else:
                obstacle_matrix[index[0]][index[1]] = 0
        # draw_dots(obstacle_matrix)
        # Daca incercam sa facem, alt plot, crapa toata aplicatia
        array_of_ones = [[i, j] for i in range(0, obstacle_matrix.shape[0]) for j in range(0, obstacle_matrix.shape[1])
                         if obstacle_matrix[i][j] == 1]
        array_of_ones = np.array(array_of_ones)
        # plt.figure(999)
        # plt.scatter(array_of_ones[:, 0], array_of_ones[:, 1], marker=".", color="black", s="5")
        # plt.show()
        with open("obstacle.csv", "w+") as my_csv:
            csvWriter = csv.writer(my_csv, delimiter=' ')
            csvWriter.writerows(obstacle_matrix)
        # scale_percent = 300
        # width = int(reshaped_map_bytes.shape[0] * scale_percent / 100)
        # height = int(reshaped_map_bytes.shape[1] * scale_percent / 100)
        # dsize = (width, height)
        #
        # treime = reshaped_map_bytes.shape[1] / 3.0
        # fraction = reshaped_map_bytes.reshape((MAP_SIZE_PIXELS, MAP_SIZE_PIXELS, reshaped_map_bytes.shape[2]))
        # for point in quality:
        #     radian = [x * (3.14 / 180) for x in angles]
        #     x_points_in_plan = [a * sin(b) for a, b in zip(distances, radian)]
        #     y_points_in_plan = [a * cos(b) for a, b in zip(distances, radian)]
        #
        #     i = 0
        #     for x, y in zip(x_points_in_plan, y_points_in_plan):
        #         punct = Point(x, y, angles[i], np_map_bytes[i], False)
        #         points_list.append(punct)
        #         i += 1
        #     np_map = np.array([x_points_in_plan, y_points_in_plan])
        #     # np_map_bytes = np_map_bytes.reshape(MAP_SIZE_PIXELS, MAP_SIZE_PIXELS)
        #     with open("data.csv", 'w') as f:
        #         # using csv.writer method from CSV package
        #         write = csv.writer(f, delimiter=' ')
        #         write.writerows(np_map)
        # delta = [(point.x, point.y, point.rgb) for point in points_list]
        # delta = np.array(delta)
        # with open("delta.csv", 'w') as f:
        #     # using csv.writer method from CSV package
        #     write = csv.writer(f, delimiter='\n')
        #     write.writerow(delta)
        # print(" The points list: " + str(points_list[1:5]))
        # fraction = np.rot90(fraction, 3)
        # new_img = cv2.resize(fraction, dsize)
        # rotated_image = cv2.rotate(new_img, cv2.ROTATE_90_CLOCKWISE)
        # delta = delta.reshape(MAP_SIZE_PIXELS, MAP_SIZE_PIXELS, 1)
        # colors = np_map_bytes
        # colors = np.array(colors)
        # colors = colors.reshape(MAP_SIZE_PIXELS, MAP_SIZE_PIXELS, 1)
        # # cv2.imwrite('image.png', delta)
        # img = Image.fromarray(colors, 'RGB')
        # img.save('image.png')
        # cv2.imwrite('cv.png', colors)
        # reshaped_map_bytes = np.fliplr(reshaped_map_bytes)
        # reshaped_map_bytes = np.rot90(reshaped_map_bytes, 2)
        # cv2.imwrite('reshapedMapBytes.jpg', reshaped_map_bytes[:, :, 1])
        # length = len(reshaped_map_bytes.shape)
        # print("length: ", length)
        # threeChannelImage = cv2.cvtColor(reshaped_map_bytes, cv2.COLOR_BGR2GRAY)
        # cv2.imwrite('mapbytes.jpg', threeChannelImage)
        # oneChannelImage = cv2.imread('mapbytes.jpg', cv2.IMREAD_UNCHANGED)
        # print("One channel shape: ", oneChannelImage.shape)
        # cv2.imwrite("oneChannelImage.jpg", oneChannelImage)
        # cvImage = cv2.imread('oneChannelImage.jpg', cv2.IMREAD_UNCHANGED)
        # cv2.imwrite('cvGrayScale.jpg', cvImage)
        # cv2.imshow('Image', fraction)
        # Display map and robot pose, exiting gracefully if user closes it
        if not viz.display(x / 1000., y / 1000., theta, mapbytes):
            lidar.stop()
            lidar.stop_motor()
            lidar.disconnect()
            exit(0)

    # Shut down the lidar connection
    lidar.stop()
    lidar.stop_motor()
    lidar.disconnect()
