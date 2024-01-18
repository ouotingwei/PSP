import sys, os, math, time
import matplotlib.pyplot as plt 
from numpy import *
from pupil_apriltags import Detector
import cv2

def apriltagLocalization(files,K):
    img = cv2.imread(files)
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    plt.figure(figsize=(10, 10))
    plt.imshow(gray, cmap=plt.cm.gray)

    at_detector = Detector(families='tag36h11')
    tags = at_detector.detect(gray)

    print("tags: {}".format(tags))

    for tag in tags:
        print(tuple(tag.corners[0].astype(int)))
        for i in range(4):
            cv2.circle(img, tuple(tag.corners[i].astype(int)), 4, (255, 0, 0), 2)

        cv2.circle(img, tuple(tag.center.astype(int)), 4, (2, 180, 200), 4)

    plt.figure(figsize=(10,10))
    plt.imshow(img)
    plt.show()

def main():
    files = '/home/wei/PSP/files/test_2.jpg'
    K = [623.6932761844039, 0.0, 329.01981432231025, 0.0, 624.1571561352914, 235.02669920374768, 0.0, 0.0, 1.0]
    apriltagLocalization = os.path.join(files, K)



if __name__ == '__main__':
    main()