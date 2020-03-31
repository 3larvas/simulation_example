import socket
import cv2
import numpy as np
import time
from lidar_util import get_lidar, visualize_lidar_img, transform_lidar2cam
from cam_util import get_img


params_lidar = {
    "Range" : 90, #min & max range of lidar azimuths
    "CHANNEL" : 16, #verticla channel of a lidar
    "localIP": "127.0.0.1", #DST IP
    "localPort": 2368, # DST port
    "Block_SIZE": int(1206),
    "X": 0.5, # meter
    "Y": 0,
    "Z": 0.8,
    "YAW": 0, # deg
    "PITCH": 0,
    "ROLL": 0
}


params_cam = {
    "SOCKET_TYPE": 'JPG',
    "WIDTH": 720, # image width
    "HEIGHT": 360, # image height
    "FOV": 90, # Field of view
    "localIP": "127.0.0.1",
    "localPort": 1232,
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


UDP_lidar = socket.socket(family=socket.AF_INET, type=socket.SOCK_DGRAM)
UDP_lidar.bind((params_lidar["localIP"], params_lidar["localPort"]))

UDP_cam = socket.socket(family=socket.AF_INET, type=socket.SOCK_DGRAM)
UDP_cam.bind((params_cam["localIP"], params_cam["localPort"]))


R_T = transform_lidar2cam(params_lidar, params_cam)

def main():

    for _ in range(1000):

        # measurement
        t_s1 = time.time()
        img_cam = get_img(UDP_cam, params_cam)

        azimuth, distance, intensity = get_lidar(UDP_lidar,
                                                 params_lidar["Block_SIZE"],
                                                 params_lidar["CHANNEL"],
                                                 Range=90)

        print("lidar:", time.time()-t_s1)

        point_img = visualize_lidar_img(azimuth, distance, intensity, params_lidar, params_cam, params_visual, R_T)
        
        # overlap the lidar points on the camera image
        img_cam = cv2.addWeighted(img_cam, 0.5, point_img, 0.5, 0.0)
        cv2.imshow('Result', cv2.resize(img_cam, (params_cam["WIDTH"], params_cam["HEIGHT"]), interpolation=cv2.INTER_LINEAR))
        cv2.waitKey(1)

    UDP_lidar.close()
    UDP_cam.close()

    
if __name__ == '__main__':

    main()