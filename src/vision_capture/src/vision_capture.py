import sys, os, math, time
import matplotlib.pyplot as plt 
import numpy as np
from pupil_apriltags import Detector
import cv2

def apriltagLocalization(files,K):
    img = cv2.imread(files)
    gray = cv2.cvtColor(img, cv2.COLOR_RGB2GRAY)
    plt.figure(figsize=(10, 10))
    plt.imshow(gray, cmap=plt.cm.gray)

    at_detector = Detector(families='tag36h11')
    tags = at_detector.detect(gray)

    #print("tags: {}".format(tags))

    # homography matrix
    H = np.array(tags[0].homography.astype(float))

    num, Rs, Ts, Ns = cv2.decomposeHomographyMat(H, K)

    print("num: {}".format(num), "Rs: {}".format(Rs), "Ts: {}".format(Ts), "Ns: {}".format(Ns))


    plt.figure(figsize=(10,10))
    plt.imshow(img)
    plt.show()


def main():
    files = '/home/wei/PSP/files/test.jpg'
    K = np.array([[623.6932761844039, 0.0, 329.01981432231025], [0.0, 624.1571561352914, 235.02669920374768], [0.0, 0.0, 1.0]])
    apriltagLocalization(files, K)



if __name__ == '__main__':
    main()