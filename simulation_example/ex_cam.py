import socket
import cv2
import numpy as np
import time
from lidar_util import get_lidar, visualize_lidar_img, transform_lidar2cam
from cam_util import get_img

params_cam = {
    "SOCKET_TYPE": 'JPG',
    "WIDTH": 720, # image width # 값 같아야함 주의
    "HEIGHT": 480, # image height
    "FOV": 90, # Field of view
    "localIP": "127.0.0.1", # 시뮬레이터의 DST 값
    "localPort": 1232, # 시뮬레이터의 DST 값
    "Block_SIZE": int(65000),
    "X": 0.5, # meter
    "Y": 0,
    "Z": 1,
    "YAW": 0, # deg
    "PITCH": -10,
    "ROLL": 0
}


params_visual = {
    "USE_INTENSITY_MAP": True,
    "DISTANCE_MAX": 50, # the maximum distance shown on the distance map
    "COLORMAP": cv2.COLORMAP_RAINBOW, # a type of color maps
    "DILATION_KERNAL": cv2.MORPH_ELLIPSE, # cv2.MORPH_RECT, cv2.MORPH_ELLIPSE, cv2.MORPH_CROSS
    "DILATION_SIZE": 5 # a type of color maps
}


UDP_cam = socket.socket(family=socket.AF_INET, type=socket.SOCK_DGRAM)
UDP_cam.bind((params_cam["localIP"], params_cam["localPort"]))

def main():

    while(1):

        # measurement
        t_s1 = time.time()
        img_cam = get_img(UDP_cam, params_cam)

        cv2.imshow('Result', cv2.resize(img_cam, (params_cam["WIDTH"], params_cam["HEIGHT"]), interpolation=cv2.INTER_LINEAR))
        cv2.waitKey(1)

    UDP_cam.close()

    
if __name__ == '__main__':

    main()