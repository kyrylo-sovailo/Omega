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

def bitwise_or_multichannel(buffer, binary_image):
    buffer_b, buffer_g, buffer_r = cv.split(buffer)
    buffer_b = cv.bitwise_or(buffer_b, binary_image)
    buffer_g = cv.bitwise_or(buffer_g, binary_image)
    buffer_r = cv.bitwise_or(buffer_r, binary_image)
    return cv.merge((buffer_b, buffer_g, buffer_r))

def bitwise_nand_multichannel(buffer, binary_image):
    binary_image = cv.bitwise_not(binary_image)
    buffer_b, buffer_g, buffer_r = cv.split(buffer)
    buffer_b = cv.bitwise_and(buffer_b, binary_image)
    buffer_g = cv.bitwise_and(buffer_g, binary_image)
    buffer_r = cv.bitwise_and(buffer_r, binary_image)
    return cv.merge((buffer_b, buffer_g, buffer_r))

def test(c, filename, canny_space):
    # Load
    image = cv.imread(filename)

    # Preprocess
    blurred_image = cv.GaussianBlur(image, (c.camera.blur_size, c.camera.blur_size), c.camera.blur_strength)
    buffer = blurred_image.copy()
    if canny_space == "mono":
        gray_image = cv.cvtColor(blurred_image, cv.COLOR_BGR2GRAY)
        binary_image = cv.Canny(gray_image, c.robot.canny_threshold1, c.robot.canny_threshold2, None, c.robot.canny_aperture)
    elif canny_space == "bgr":
        image_b, image_g, image_r = cv.split(blurred_image)
        image_b = cv.Canny(image_b, c.robot.canny_threshold1, c.robot.canny_threshold2, None, c.robot.canny_aperture)
        image_g = cv.Canny(image_g, c.robot.canny_threshold1, c.robot.canny_threshold2, None, c.robot.canny_aperture)
        image_r = cv.Canny(image_r, c.robot.canny_threshold1, c.robot.canny_threshold2, None, c.robot.canny_aperture)
        binary_image = cv.bitwise_or(image_b, cv.bitwise_or(image_g, image_r))
    elif canny_space == "hsv":
        hsv_image = cv.cvtColor(blurred_image, cv.COLOR_BGR2HSV)
        image_h, image_s, image_v = cv.split(hsv_image)
        image_h = cv.Canny(image_h, c.robot.canny_threshold1, c.robot.canny_threshold2, None, c.robot.canny_aperture)
        image_s = cv.Canny(image_s, c.robot.canny_threshold1, c.robot.canny_threshold2, None, c.robot.canny_aperture)
        image_v = cv.Canny(image_v, c.robot.canny_threshold1, c.robot.canny_threshold2, None, c.robot.canny_aperture)
        binary_image = np.zeros((hsv_image.shape[0], hsv_image.shape[1]), dtype=np.uint8)
        #binary_image = cv.bitwise_or(binary_image, image_h)
        #binary_image = cv.bitwise_or(binary_image, image_s)
        binary_image = cv.bitwise_or(binary_image, image_v)
    else: raise Exception("Invalid canny_space")
    
    buffer = bitwise_or_multichannel(buffer, binary_image)
    
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

def test2(c, filename, canny_space):
    # Load
    image = cv.imread(filename)

    # Blur
    blurred_image = cv.GaussianBlur(image, (c.camera.blur_size, c.camera.blur_size), c.camera.blur_strength)
    buffer = blurred_image.copy()

    # Color recognition
    hsv_image = cv.cvtColor(blurred_image, cv.COLOR_BGR2HSV)
    ball_lower = (24, 105, 72)
    ball_upper = (39, 237, 226)
    wall_lower = (21, 7, 56)
    wall_upper = (107, 69, 200)
    grass_lower = (59, 95, 75)
    grass_upper = (76, 146, 140)
    wall_binary_image = cv.inRange(hsv_image, wall_lower, wall_upper)
    grass_binary_image = cv.inRange(hsv_image, grass_lower, grass_upper)
    wall_grass_binary_image = cv.bitwise_or(wall_binary_image, grass_binary_image)
    wall_grass_binary_image = cv.dilate(wall_grass_binary_image, cv.getStructuringElement(cv.MORPH_RECT, (15, 15)))
    
    # Edge detector
    if canny_space == "mono":
        gray_image = cv.cvtColor(blurred_image, cv.COLOR_BGR2GRAY)
        binary_image = cv.Canny(gray_image, c.robot.canny_threshold1, c.robot.canny_threshold2, None, c.robot.canny_aperture)
    elif canny_space == "bgr":
        image_b, image_g, image_r = cv.split(blurred_image)
        image_b = cv.Canny(image_b, c.robot.canny_threshold1, c.robot.canny_threshold2, None, c.robot.canny_aperture)
        image_g = cv.Canny(image_g, c.robot.canny_threshold1, c.robot.canny_threshold2, None, c.robot.canny_aperture)
        image_r = cv.Canny(image_r, c.robot.canny_threshold1, c.robot.canny_threshold2, None, c.robot.canny_aperture)
        binary_image = cv.bitwise_or(image_b, cv.bitwise_or(image_g, image_r))
    elif canny_space == "hsv":
        hsv_image = cv.cvtColor(blurred_image, cv.COLOR_BGR2HSV)
        image_h, image_s, image_v = cv.split(hsv_image)
        image_h = cv.Canny(image_h, c.robot.canny_threshold1, c.robot.canny_threshold2, None, c.robot.canny_aperture)
        image_s = cv.Canny(image_s, c.robot.canny_threshold1, c.robot.canny_threshold2, None, c.robot.canny_aperture)
        image_v = cv.Canny(image_v, c.robot.canny_threshold1, c.robot.canny_threshold2, None, c.robot.canny_aperture)
        binary_image = np.zeros((hsv_image.shape[0], hsv_image.shape[1]), dtype=np.uint8)
        #binary_image = cv.bitwise_or(binary_image, image_h)
        #binary_image = cv.bitwise_or(binary_image, image_s)
        binary_image = cv.bitwise_or(binary_image, image_v)
    else: raise Exception("Invalid canny_space")

    buffer = bitwise_or_multichannel(buffer, wall_grass_binary_image) #White
    buffer = bitwise_nand_multichannel(buffer, binary_image) #Black

    binary_image = cv.bitwise_and(binary_image, wall_grass_binary_image)
    
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

def find_png(directory):
    return [ directory + "/" + f for f in os.listdir(directory) if os.path.isfile(directory + "/" + f) and f.split('.')[-1] == "png" ]

def main():
    directory = os.path.dirname(os.path.realpath(__file__))
    config = Config(directory + "/../config/config.yaml")
    files = find_png(directory)
    tests = [ test2(config, f, "hsv") for f in files ]
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