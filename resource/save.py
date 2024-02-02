#!/usr/bin/python3
import os, math
import yaml
import numpy as np
import cv2 as cv

class Camera:
    def __init__(self, config):
        self.blur_strength = config['blur_strength']
        self.blur_size = config['blur_size']

class BallTracker:
    def __init__(self, config):
        self.min_hue = config['min_hue']
        self.max_hue = config['max_hue']
        self.min_saturation = config['min_saturation']
        self.max_saturation = config['max_saturation']
        self.min_value = config['min_value']
        self.max_value = config['max_value']
        self.dilate_size = config['dilate_size']

class RobotTracker:
    def __init__(self, config):
        self.canny_threshold1 = config['canny_threshold1']
        self.canny_threshold2 = config['canny_threshold2']
        self.canny_aperture = config['canny_aperture']
        self.hough_distance_resolution = config['hough_distance_resolution']
        self.hough_angle_resolution = math.pi * config['hough_angle_resolution'] / 180
        self.hough_threshold = config['hough_threshold']
        self.horizontal_max_angle = math.pi * config['horizontal_max_angle'] / 180
        self.vertical_max_angle = math.pi * config['vertical_max_angle'] / 180

class Config:
    def __init__(self, filename):
        file = open(filename)
        config = yaml.safe_load(file)
        self.camera = Camera(config['camera'])
        self.ball = BallTracker(config['ball_tracker'])
        self.robot = RobotTracker(config['robot_tracker'])

def test(c, filename):
    # Load
    image = cv.imread(filename)

    # Preprocess
    blurred_image = cv.GaussianBlur(image, (c.camera.blur_size, c.camera.blur_size), c.camera.blur_strength)
    buffer = blurred_image.copy()
    gray_image = cv.cvtColor(blurred_image, cv.COLOR_BGR2GRAY)
    binary_image = cv.Canny(gray_image, c.robot.canny_threshold1, c.robot.canny_threshold2, None, c.robot.canny_aperture)
    
    # Hough
    lines = np.zeros((0, 1, 2))
    lines1 = cv.HoughLines(binary_image.copy(), c.robot.hough_distance_resolution, c.robot.hough_angle_resolution, c.robot.hough_threshold,
        None, 0, 0, 0, c.robot.vertical_max_angle)
    if not lines1 is None: lines = np.concatenate((lines, lines1), axis=0)
    lines2 = cv.HoughLines(binary_image.copy(), c.robot.hough_distance_resolution, c.robot.hough_angle_resolution, c.robot.hough_threshold,
        None, 0, 0, math.pi - c.robot.vertical_max_angle, math.pi)
    if not lines2 is None: lines = np.concatenate((lines, lines2), axis=0)
    lines3 = cv.HoughLines(binary_image.copy(), c.robot.hough_distance_resolution, c.robot.hough_angle_resolution, c.robot.hough_threshold,
        None, 0, 0, math.pi / 2 - c.robot.horizontal_max_angle, math.pi / 2 + c.robot.horizontal_max_angle)
    if not lines3 is None: lines = np.concatenate((lines, lines3), axis=0)
    for line in lines:
        rho = line[0][0]
        theta = line[0][1]
        a = math.cos(theta)
        b = math.sin(theta)
        x0 = a * rho
        y0 = b * rho
        span = 2000
        point1 = (int(x0 + span * (-b)), int(y0 + span * (a)))
        point2 = (int(x0 - span * (-b)), int(y0 - span * (a)))
        cv.line(buffer, point1, point2, (0,0,255), 2)

    return buffer

def main():
    directory = os.path.dirname(os.path.realpath(__file__))
    config = Config(directory + "/../config/config.yaml")
    files = [ directory + "/" + f for f in os.listdir(directory) if os.path.isfile(directory + "/" + f) and f.split('.')[-1] == "png" ]
    tests = [ test(config, f) for f in files ]
    image_height = tests[0].shape[0]
    image_width = tests[0].shape[1]
    collage_height = math.ceil(math.sqrt(len(files)))
    collage_width = math.ceil(len(files) / collage_height)
    collage = np.zeros([collage_height * image_height, collage_width * image_width, 3], dtype=np.uint8)
    for x in range(collage_width):
        for y in range(collage_height):
            i = y * collage_width + x
            if i < len(tests):
                collage[y * image_height : (y + 1) * image_height, x * image_width : (x + 1) * image_width, :] = tests[i]
    cv.imshow("Collage", cv.resize(collage, (collage.shape[1] // 3, collage.shape[0] // 3)))
    cv.waitKey(0)

if __name__ == "__main__":
    main()