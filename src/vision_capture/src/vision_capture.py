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
    tags = at_detector.detect(gray,tag_size=0.015)

    #print("tags: {}".format(tags))

    # homography matrix
    H = np.array(tags[0].homography.astype(float))

    num, Rs, Ts, Ns = cv2.decomposeHomographyMat(H, K)

    # nums = possible solutions will be returned
    # Rs = contains a list of the rotation matrix
    # Ts = contains a list of the translation matrix
    # Ns = contains a list of the normal vector of the plane
    #print("num: {}".format(num), "Rs: {}".format(Rs), "Ts: {}".format(Ts), "Ns: {}".format(Ns))
    for i in range(num):
        if Ns[i][2] < 0:
            correct_solution_index = i
            break

    correct_R = Rs[correct_solution_index]
    correct_T = Ts[correct_solution_index]
    correct_N = Ns[correct_solution_index]

    print("Correct Rotation Matrix:")
    print(correct_R)
    print("Correct Translation Vector:")
    print(correct_T)
    print("Correct Normal Vector:")
    print(correct_N)


    plt.figure(figsize=(10,10))
    plt.imshow(img)
    plt.show()


def main():
    files = '/home/wei/PSP/files/14.jpg'
    K = np.array([[623.6932761844039, 0.0, 329.01981432231025], [0.0, 624.1571561352914, 235.02669920374768], [0.0, 0.0, 1.0]])
    apriltagLocalization(files, K)



if __name__ == '__main__':
    main()