#!/usr/bin/python3
import os
import numpy as np
import cv2 as cv

def find_range(mask_files, typ, space, cutoff):
    stat = np.zeros((3, 256), dtype=np.int32)

    for mask_file in mask_files:
        # Load
        image_file = os.path.dirname(os.path.dirname(mask_file)) + "/" + os.path.basename(mask_file)
        image = cv.imread(image_file)
        image = cv.GaussianBlur(image, (5, 5), 5)
        mask = cv.imread(mask_file, cv.IMREAD_GRAYSCALE)
        
        # Convert to space
        if space == "bgr":
            pass
        elif space == "hsv":
            image = cv.cvtColor(image, cv.COLOR_BGR2HSV)
        else: raise Exception("Invalid canny_space")

        # Count
        for i in range(mask.shape[0]):
            for j in range(mask.shape[1]):
                if (mask[i, j] != 0):
                    stat[0, image[i, j, 0]] += 1
                    stat[1, image[i, j, 1]] += 1
                    stat[2, image[i, j, 2]] += 1

    # Cut off
    min = np.zeros((3), dtype=np.int32)
    max = np.zeros((3), dtype=np.int32)
    for i in range(3):    
        total = np.sum(stat[i])
        sum = 0
        for j in range(256):
            sum += stat[i, j]
            min[i] = j
            if sum / total > cutoff / 2: break
        sum = 0
        for j in range(256-1, -1, -1):
            sum += stat[i, j]
            max[i] = j
            if sum / total > cutoff / 2: break

    # Print
    print(f"{typ}_lower = ({min[0]}, {min[1]}, {min[2]})")
    print(f"{typ}_upper = ({max[0]}, {max[1]}, {max[2]})")
    print(f"{typ}_color_min: [{min[0]},{min[1]},{min[2]}]")
    print(f"{typ}_color_max: [{max[0]},{max[1]},{max[2]}]")

def find_png(directory):
    return [ directory + "/" + f for f in os.listdir(directory) if os.path.isfile(directory + "/" + f) and f.split('.')[-1] == "png" ]

def main():
    directory = os.path.dirname(os.path.realpath(__file__))
    ball_masks = find_png(directory + "/ball")
    wall_masks = find_png(directory + "/wall")
    grass_masks = find_png(directory + "/grass")
    find_range(ball_masks, "ball", "hsv", 0.05)
    find_range(wall_masks, "wall", "hsv", 0.01)
    find_range(grass_masks, "grass", "hsv", 0.05)

if __name__ == "__main__":
    main()