from Point import Point
import csv
import operator
import cv2

import numpy as np
import pandas as pd
from numpy import sin, cos

MAP_SIZE_PIXELS = 498
MAP_SIZE_METERS = 10
LIDAR_DEVICE = 'COM3'

# Ideally we could use all 250 or so samples that the RPLidar delivers in one
# scan, but on slower computers you'll get an empty map and unchanging position
# at that rate.
MIN_SAMPLES = 200

from breezyslam.algorithms import RMHC_SLAM
from breezyslam.sensors import RPLidarA1 as LaserModel
from rplidar import RPLidar as Lidar, RPLidar
from roboviz import MapVisualizer

from tkinter import *
from PIL import ImageTk, Image

import matplotlib.pyplot as plt
import matplotlib.image as mpimg

import time


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
        items = [item for item in next(iterator)]

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

        points_list = []

        # Get current map bytes as grayscale
        slam.getmap(mapbytes)

        print(len(mapbytes))
        np_map_bytes = np.asarray(mapbytes)


        reshaped_map_bytes = np_map_bytes.reshape((332, 249, 3))
        # create image

        with open("empty_rows.csv", 'w', newline='') as f:
            write = csv.writer(f, delimiter=' ')
            for vector in reshaped_map_bytes:
                for rgb in vector:
                    write.writerow(rgb)
            f.close()

        print("mapbytes: ", np_map_bytes)
        scale_percent = 300
        width = int(reshaped_map_bytes.shape[0] * scale_percent / 100)
        height = int(reshaped_map_bytes.shape[1] * scale_percent / 100)
        dsize = (width, height)

        treime = reshaped_map_bytes.shape[1] / 3.0
        fraction = reshaped_map_bytes.reshape((996, 83, reshaped_map_bytes.shape[2]))
        # fraction = np.rot90(fraction, 3)

        # new_img = cv2.resize(fraction, dsize)
        # rotated_image = cv2.rotate(new_img, cv2.ROTATE_90_CLOCKWISE)
        # cv2.imwrite('Test/new_image.png', rotated_image)
        # cv2.imshow('Image', reshaped_map_bytes)

        # Display map and robot pose, exiting gracefully if user closes it
        if not viz.display(x / 1000., y / 1000., theta, mapbytes):
            exit(0)

        for point in quality:
            if point == 15:
                radian = [x * (3.14 / 180) for x in angles]
                x_points_in_plan = [a * sin(b) for a, b in zip(distances, radian)]
                y_points_in_plan = [a * cos(b) for a, b in zip(distances, radian)]

                i = 0
                for x, y in zip(x_points_in_plan, y_points_in_plan):
                    punct = Point(x, y, angles[i], (np_map_bytes[i], np_map_bytes[i+1], np_map_bytes[i+2]), False)
                    points_list.append(punct)
                    i += 1
                np_map = np.array([x_points_in_plan, y_points_in_plan])
                #np_map_bytes = np_map_bytes.reshape(MAP_SIZE_PIXELS, MAP_SIZE_PIXELS)
                with open("data.csv", 'w') as f:
                    # using csv.writer method from CSV package
                    write = csv.writer(f, delimiter=' ')
                    write.writerows(np_map)
        delta = list(map(operator.sub, x_points_in_plan, y_points_in_plan))
        delta = np.array(delta)
        with open("delta.csv", 'w') as f:
            # using csv.writer method from CSV package
            write = csv.writer(f, delimiter=' ')
            write.writerow(delta)
        print(" The points list: " + str(points_list[1:5]))

    # Shut down the lidar connection
    lidar.stop()
    lidar.disconnect()
