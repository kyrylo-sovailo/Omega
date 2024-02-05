#!/usr/bin/python3
import os
import yaml
import numpy as np
import cv2 as cv

class Camera:
    def __init__(self, config):
        self.blur_strength = config['blur_strength']
        self.blur_size = config['blur_size']

class BallTracker:
    def __init__(self, config):
        self.ball_color_min = (config['ball_color_min'][0], config['ball_color_min'][1], config['ball_color_min'][2])
        self.ball_color_max = (config['ball_color_max'][0], config['ball_color_max'][1], config['ball_color_max'][2])
        self.dilate_size = config['dilate_size']

class RobotTracker:
    def __init__(self, config):
        self.wall_color_min = (config['wall_color_min'][0], config['wall_color_min'][1], config['wall_color_min'][2])
        self.wall_color_max = (config['wall_color_max'][0], config['wall_color_max'][1], config['wall_color_max'][2])
        self.grass_color_min = (config['grass_color_min'][0], config['grass_color_min'][1], config['grass_color_min'][2])
        self.grass_color_max = (config['grass_color_max'][0], config['grass_color_max'][1], config['grass_color_max'][2])
        self.dilate_size = config['dilate_size']
        self.canny_threshold1 = config['canny_threshold1']
        self.canny_threshold2 = config['canny_threshold2']
        self.canny_aperture = config['canny_aperture']
        self.hough_distance_resolution = config['hough_distance_resolution']
        self.hough_angle_resolution = np.pi * config['hough_angle_resolution'] / 180
        self.horizontal_hough_threshold = config['horizontal_hough_threshold']
        self.vertical_hough_threshold = config['vertical_hough_threshold']
        self.horizontal_max_angle = np.pi * config['horizontal_max_angle'] / 180
        self.vertical_max_angle = np.pi * config['vertical_max_angle'] / 180
        self.horizontal_max_lines = config['horizontal_max_lines']
        self.vertical_max_lines = config['vertical_max_lines']

class Config:
    def __init__(self, filename):
        file = open(filename)
        config = yaml.safe_load(file)
        self.camera = Camera(config['camera'])
        self.ball = BallTracker(config['ball_tracker'])
        self.robot = RobotTracker(config['robot_tracker'])

def draw_mask(buffer, binary_image, bgr_color = (255, 255, 255)):
    buffer_b, buffer_g, buffer_r = cv.split(buffer)
    if bgr_color[0] > 0: buffer_b = cv.bitwise_or(buffer_b, binary_image)
    else: buffer_b = cv.bitwise_and(buffer_b, cv.bitwise_not(binary_image))
    if bgr_color[1] > 0: buffer_g = cv.bitwise_or(buffer_g, binary_image)
    else: buffer_g = cv.bitwise_and(buffer_g, cv.bitwise_not(binary_image))
    if bgr_color[2] > 0: buffer_r = cv.bitwise_or(buffer_r, binary_image)
    else: buffer_r = cv.bitwise_and(buffer_r, cv.bitwise_not(binary_image))
    return cv.merge((buffer_b, buffer_g, buffer_r))

def test_simple(c, filename):
    # Load
    image = cv.imread(filename)

    # Preprocess
    blurred_image = cv.GaussianBlur(image, (c.camera.blur_size, c.camera.blur_size), c.camera.blur_strength)
    buffer = blurred_image.copy()
    hsv_image = cv.cvtColor(blurred_image, cv.COLOR_BGR2HSV)
    image_h, image_s, image_v = cv.split(hsv_image)
    image_h = cv.Canny(image_h, c.robot.canny_threshold1, c.robot.canny_threshold2, None, c.robot.canny_aperture)
    image_s = cv.Canny(image_s, c.robot.canny_threshold1, c.robot.canny_threshold2, None, c.robot.canny_aperture)
    image_v = cv.Canny(image_v, c.robot.canny_threshold1, c.robot.canny_threshold2, None, c.robot.canny_aperture)
    edge_image = np.zeros((hsv_image.shape[0], hsv_image.shape[1]), dtype=np.uint8)
    #edge_image = cv.bitwise_or(edge_image, image_h)
    #edge_image = cv.bitwise_or(edge_image, image_s)
    edge_image = cv.bitwise_or(edge_image, image_v)
    
    # Hough
    lines = np.zeros((0, 1, 2))
    lines1 = cv.HoughLines(edge_image.copy(), c.robot.hough_distance_resolution, c.robot.hough_angle_resolution, c.robot.vertical_hough_threshold,
        None, 0, 0, 0, c.robot.vertical_max_angle)
    if not lines1 is None: lines = np.concatenate((lines, lines1), axis=0)
    lines2 = cv.HoughLines(edge_image.copy(), c.robot.hough_distance_resolution, c.robot.hough_angle_resolution, c.robot.vertical_hough_threshold,
        None, 0, 0, np.pi - c.robot.vertical_max_angle, np.pi)
    if not lines2 is None: lines = np.concatenate((lines, lines2), axis=0)
    lines3 = cv.HoughLines(edge_image.copy(), c.robot.hough_distance_resolution, c.robot.hough_angle_resolution, c.robot.horizontal_hough_threshold,
        None, 0, 0, np.pi / 2 - c.robot.horizontal_max_angle, np.pi / 2 + c.robot.horizontal_max_angle)
    if not lines3 is None: lines = np.concatenate((lines, lines3), axis=0)
    for line in lines:
        rho = line[0][0]
        theta = line[0][1]
        a = np.cos(theta)
        b = np.sin(theta)
        x0 = a * rho
        y0 = b * rho
        span = 2000
        point1 = (int(x0 + span * (-b)), int(y0 + span * (a)))
        point2 = (int(x0 - span * (-b)), int(y0 - span * (a)))
        cv.line(buffer, point1, point2, (0,0,255), 2)

    return buffer

def test(c, filename):
    # Load
    image = cv.imread(filename)

    # Blur
    blurred_image = cv.GaussianBlur(image, (c.camera.blur_size, c.camera.blur_size), c.camera.blur_strength)
    buffer = blurred_image.copy()

    # Color recognition
    hsv_image = cv.cvtColor(blurred_image, cv.COLOR_BGR2HSV)
    wall_binary_image = cv.inRange(hsv_image, c.robot.wall_color_min, c.robot.wall_color_max)
    grass_binary_image = cv.inRange(hsv_image, c.robot.grass_color_min, c.robot.grass_color_max)
    wall_grass_binary_image = cv.bitwise_or(wall_binary_image, grass_binary_image)
    wall_grass_binary_image = cv.dilate(wall_grass_binary_image, cv.getStructuringElement(cv.MORPH_RECT, (c.robot.dilate_size, c.robot.dilate_size)))
    buffer = draw_mask(buffer, wall_grass_binary_image)

    ball_binary_image = cv.inRange(hsv_image, c.ball.ball_color_min, c.ball.ball_color_max)
    ball_binary_image = cv.dilate(ball_binary_image, cv.getStructuringElement(cv.MORPH_RECT, (c.ball.dilate_size, c.ball.dilate_size)))
    ball_binary_image = cv.erode(ball_binary_image, cv.getStructuringElement(cv.MORPH_RECT, (c.ball.dilate_size, c.ball.dilate_size)))
    buffer = draw_mask(buffer, ball_binary_image, (0, 255, 255))
    
    # Edge detector
    image_h, image_s, image_v = cv.split(hsv_image)
    image_h = cv.Canny(image_h, c.robot.canny_threshold1, c.robot.canny_threshold2, None, c.robot.canny_aperture)
    image_s = cv.Canny(image_s, c.robot.canny_threshold1, c.robot.canny_threshold2, None, c.robot.canny_aperture)
    image_v = cv.Canny(image_v, c.robot.canny_threshold1, c.robot.canny_threshold2, None, c.robot.canny_aperture)
    edge_image = np.zeros((hsv_image.shape[0], hsv_image.shape[1]), dtype=np.uint8)
    #edge_image = cv.bitwise_or(edge_image, image_h)
    #edge_image = cv.bitwise_or(edge_image, image_s)
    edge_image = cv.bitwise_or(edge_image, image_v)
    edge_image = cv.bitwise_and(edge_image, wall_grass_binary_image)
    buffer = draw_mask(buffer, edge_image, (255, 0, 0))
    
    # Hough
    lines = np.zeros((0, 1, 2))
    lines1 = cv.HoughLines(edge_image.copy(), c.robot.hough_distance_resolution, c.robot.hough_angle_resolution, c.robot.hough_threshold,
        None, 0, 0, 0, c.robot.vertical_max_angle)
    if not lines1 is None: lines = np.concatenate((lines, lines1), axis=0)
    lines2 = cv.HoughLines(edge_image.copy(), c.robot.hough_distance_resolution, c.robot.hough_angle_resolution, c.robot.hough_threshold,
        None, 0, 0, np.pi - c.robot.vertical_max_angle, np.pi)
    if not lines2 is None: lines = np.concatenate((lines, lines2), axis=0)
    lines3 = cv.HoughLines(edge_image.copy(), c.robot.hough_distance_resolution, c.robot.hough_angle_resolution, c.robot.hough_threshold,
        None, 0, 0, np.pi / 2 - c.robot.horizontal_max_angle, np.pi / 2 + c.robot.horizontal_max_angle)
    if not lines3 is None: lines = np.concatenate((lines, lines3), axis=0)
    for line in lines:
        rho = line[0][0]
        theta = line[0][1]
        a = np.cos(theta)
        b = np.sin(theta)
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
    tests = [ test(config, f) for f in files ]
    image_height = tests[0].shape[0]
    image_width = tests[0].shape[1]
    collage_height = int(np.ceil(np.sqrt(len(files))))
    collage_width = int(np.ceil(len(files) / collage_height))
    collage = np.zeros((collage_height * image_height, collage_width * image_width, 3), dtype=np.uint8)
    for x in range(collage_width):
        for y in range(collage_height):
            i = y * collage_width + x
            if i < len(tests):
                collage[y * image_height : (y + 1) * image_height, x * image_width : (x + 1) * image_width, :] = tests[i]
    cv.imshow("Collage", cv.resize(collage, (collage.shape[1] // 3, collage.shape[0] // 3)))
    cv.waitKey(0)

if __name__ == "__main__":
    main()