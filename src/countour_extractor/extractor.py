# Version : V1
# Deadline : 2023 / 07 / 15
# Author : PoLin Jiang
# Discription : HongLang Project
#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import cv2
import numpy as np

def boundary_detection(image_path):
    # read image
    image = cv2.imread(image_path)
    if image is None:
        print("Failed to load the image")
        exit()

    # reshape
    scale_size = 10
    height, width, _ = image.shape
    height = int(height/scale_size)
    width = int(width/scale_size)
    image = cv2.resize(image, (width, height))

    # Pre-processing
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    blurred = cv2.GaussianBlur(gray, (5, 5), 0)

    # Apply Canny edge detection
    edges = cv2.Canny(blurred, threshold1=100, threshold2=200)

    # Apply adaptive thresholding to obtain a binary image
    _, binary = cv2.threshold(edges, 0, 255, cv2.THRESH_BINARY | cv2.THRESH_OTSU)

    # Perform closing to fill in gaps in the contours
    kernel = np.ones((5, 5), np.uint8)
    closed = cv2.morphologyEx(binary, cv2.MORPH_CLOSE, kernel, iterations=3)

    # find boundary
    max_x = 0
    min_x = 1000
    max_y = 0
    min_y = 1000
    for x in range(width):
        for y in range(height):
            if closed[y, x] == 255:
                max_x = max(max_x, x)
                min_x = min(min_x, x)
                max_y = max(max_y, y)
                min_y = min(min_y, y)

    image = cv2.resize(image, (width, height))
    closed = cv2.resize(closed, (width, height))
    # Draw a rectangle around the shoe contours
    rectangle_thickness = 2
    cv2.rectangle(image, (min_x, min_y), (max_x, max_y), (0, 255, 0), rectangle_thickness)
    cv2.rectangle(closed, (min_x, min_y), (max_x, max_y), (255, 255, 255), rectangle_thickness)

    # Display the original image with the rectangle
    # cv2.imshow('Original Image', image)
    # cv2.imshow('Edges', edges)
    # cv2.imshow('Binary Image', binary)
    # cv2.imshow('After Detection', closed)

    # Press 'q' or ESC to exit
    # while True:
    #     key = cv2.waitKey(1) & 0xFF
    #     if key == ord('q') or key == 27:   
    #         break
    # cv2.destroyAllWindows()

    # save image
    cv2.imwrite('result/Original_Image.png',image)
    cv2.imwrite('result/Edges.png',closed)

    # re-scale
    height = height*scale_size
    width = width*scale_size
    max_x = max_x*scale_size
    min_x = min_x*scale_size
    max_y = max_y*scale_size
    min_y = min_y*scale_size
    # move to center
    max_x = max_x - width/2
    min_x = min_x - width/2
    max_y = max_y - height/2
    min_y = min_y - height/2

    return max_x, min_x, max_y, min_y

def path_generation(max_x, min_x, max_y, min_y):
    expand_size = 20
    step_size = 30
    reverse = True
    cur_high = max_y

    max_x = max_x + expand_size
    min_x = min_x - expand_size
    max_y = max_y + expand_size
    min_y = min_y - expand_size
    print(max_x, min_x, max_y, min_y)

    # generate path
    points = []
    points.append([min_x, max_y, 0])
    points.append([max_x, max_y, 0])
    while(True):
        cur_high = cur_high - step_size
        if(cur_high <= min_y):
            break

        if(reverse):
            points.append([max_x, cur_high, 0])
            points.append([min_x, cur_high, 0])
            reverse = False
        else:
            points.append([min_x, cur_high, 0])
            points.append([max_x, cur_high, 0])
            reverse = True

    if(reverse):
            points.append([max_x, min_y, 0])
            points.append([min_x, min_y, 0])
    else:
        points.append([min_x, min_y, 0])
        points.append([max_x, min_y, 0])

    return points

def main():
    image_path = 'source/IMG_3443.jpg'
    max_x, min_x, max_y, min_y = boundary_detection(image_path)
    points = path_generation(max_x, min_x, max_y, min_y)
    print(points)

if __name__ == '__main__':
    main()