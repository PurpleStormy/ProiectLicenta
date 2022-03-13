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
from rplidar import RPLidarException
from roboviz import MapVisualizer

from a_star2 import *

from PIL import ImageTk, Image
import tkinter as tk
from tkinter import filedialog as fd
import os

import matplotlib.pyplot as plt
import matplotlib.image as mpimg
import turtle as trt

import time
import math
import threading


# incetineste foarte tare programul
def get_obstacle_matrix(some_bytes):
    obstacles = np.zeros(shape=(some_bytes.shape[0], some_bytes.shape[1]), dtype=int)
    white_pixel_min = np.array([200, 200, 200])
    white_pixel_max = np.array([255, 255, 255])
    for index, values in np.ndenumerate(some_bytes):
        comparison = (white_pixel_min <= values).all() and (values <= white_pixel_max).all()
        if comparison:
            obstacles[index[0]][index[1]] = 1
        else:
            obstacles[index[0]][index[1]] = 0

    with open("obstacle.csv", "w+") as my_csv:
        csvWriter = csv.writer(my_csv, delimiter=' ')
        csvWriter.writerows(obstacles)
    return obstacles


def figure(matrix):
    point_list = []
    for i in range(0, matrix.shape[0]):
        for j in range(0, matrix.shape[1]):
            if matrix[i][j] == 1:
                point_list.append((i, j))

    point_list = np.asarray(point_list)
    return point_list


def draw_dots(matrix):
    points = figure(matrix)
    root = tk.Tk()
    root.title('Python Turtle')

    Width, Height = matrix.shape[0], matrix.shape[1]
    canvas = tk.Canvas(root, width=Width * 2, height=Height * 2)
    canvas.pack()

    turtle = trt.RawTurtle(canvas=canvas)
    ts = turtle.getscreen()
    ts.tracer(0)
    for point in points:
        turtle.penup()
        dx, dy = point[0] - Width / 2, point[1] - Height / 2
        turtle.goto(dx, dy)
        turtle.pendown()
        turtle.dot(1)
    trt.update()
    # eps="canvas.eps"
    # ts.getcanvas().postscript(file=eps)
    # print('saved', eps)
    # img = Image.open(eps)  ##  use PIL to convert to PNG
    # fig = img.convert('RGBA')
    # fig.save("pengeu.png", lossless=True)
    filename = 'penegeu'
    directory = fd.askdirectory(initialdir='', title='Choose folder')
    fullpath = os.path.join(directory, filename)
    eps, png = f'{fullpath}.eps', f'{fullpath}.png'
    canvas.postscript(file=eps, pagewidth=Width - 1, pageheight=Height - 1)
    print('saved', eps)
    img = Image.open(eps)  ##  use PIL to convert to PNG
    img.save(png, 'png', optimize=True, compress_level=9)


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

iterator = lidar.iter_scans()

# We will use these to store previous scan in case current scan is inadequate
previous_distances = None
previous_angles = None

next(iterator)
obstacle_matrix = []
while True:

    try:
        # Extract (quality, angle, distance) triples from current scan
        items = [item for item in next(iterator, 'end')]

        # Extract distances and angles from triples
        distances = [item[2] for item in items]
        angles = [item[1] for item in items]
        quality = [item[0] for item in items]

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
        reshaped_map_bytes2 = cv2.merge((reshaped_map_bytes, reshaped_map_bytes, reshaped_map_bytes))
        img_test = Image.fromarray(reshaped_map_bytes2.astype(np.uint8), mode='RGB')
        # img_test.show()
        img_test.save('tibi.png')

        obstacle_matrix = get_obstacle_matrix(reshaped_map_bytes2)

        if not viz.display(x / 1000., y / 1000., theta, mapbytes):
            lidar.stop()
            lidar.stop_motor()
            lidar.disconnect()
            exit(0)

    except:
        lidar.stop()
        lidar.stop_motor()
        lidar.disconnect()
        break

stryng ="0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 1 0 0 0 1 0 0 0 1 1"
stryng = stryng.replace(" ", "")
print(len(stryng))
print("start point value ", obstacle_matrix[147][117])
print("end point value", obstacle_matrix[148][119])
draw_dots(obstacle_matrix)
print(astar(obstacle_matrix, 147, 117, 148, 119))
