import cv2
import numpy as np
import math
from common_util import RotationMatrix, TraslationMatrix


VerticalAngleDeg_16 = np.array([[-15,1,-13,3,-11,5,-9,7,-7,9,-5,11,-3,13,-1,15]])
VerticalAngleDeg_32 = np.array([[-30.67,-9.33,-29.33,-8.0,-28.0,-6.67,-26.67,-5.33,-25.33,-4,-24,-2.67,-22.67,-1.33,-21.33,
                                    0.0,-20.,1.33,-18.67,2.67,-17.33,4,-16,5.33,-14.67,6.67,-13.33,8,-12,9.33,-10.67,10.67]])


def get_lidar(UDPSocket_lidar, UnitBlockSize_lidar, Channel, Range=90):
    '''
    receive lidar measurements
    '''

    buffer = []

    if Channel==16:
        max_len = 40
    else:
        max_len = 65

    while len(buffer)<max_len:
    
        bytesAddressPair = UDPSocket_lidar.recvfrom(UnitBlockSize_lidar)
        UnitBlock = bytesAddressPair[0]

        packet = np.frombuffer(UnitBlock[:1200], dtype = np.uint8)
        
        Azimuth_1 = packet[2] + 256*packet[3]

        if Azimuth_1 > (36000 - 100*int(Range)) or Azimuth_1 < 100*int(Range):
            
            buffer.append(packet.reshape([-1,100]))
    
    buf_len = len(buffer)

    buffer=np.vstack(buffer)

    if Channel==16:
        Azimuth = np.zeros((24*buf_len,))
        Azimuth[0::2] = buffer[:,2] + 256*buffer[:,3]
        Azimuth[1::2] = buffer[:,2] + 256*buffer[:,3] + 20
    else:
        Azimuth = buffer[:,2] + 256*buffer[:,3]
    
    Distance = (buffer[:,4::3].astype(np.float) + 256*buffer[:,5::3].astype(np.float))*2
    Intensity = buffer[:,6::3]

    # reshape outputs based on 16 channels
    Azimuth = Azimuth.reshape([-1, 1])/100
    Distance = Distance.reshape([-1, Channel])/1000
    Intensity = Intensity.reshape([-1, Channel])

    return Azimuth, Distance, Intensity


def sph2cart(R, a, VerticalAngleDeg):
    '''
    transform the sphere coordinate of lidar points to the cartesian coordinate
    \n R : distance from the origin
    \n a : azimuth
    '''

    x = R * np.cos(np.deg2rad(VerticalAngleDeg)) * np.sin(np.deg2rad(a))
    y = R * np.cos(np.deg2rad(VerticalAngleDeg)) * np.cos(np.deg2rad(a))
    z = R * np.sin(np.deg2rad(VerticalAngleDeg))
    
    return x, y, z


def crop_by_azimuth(adeg_p, adeg, d, intens):
    '''
    crop the lidar points between -adeg_p and +adeg_p
    \n adeg_p : clip value of azimuth
    \n adeg : azimuth
    \n d : distance
    \n intens : intensity of lidar point
    '''

    crop_idx = np.concatenate([np.where(adeg<adeg_p)[0], np.where(adeg>(360-adeg_p))[0]])
    d = d[crop_idx,:]
    intens = intens[crop_idx,:]
    adeg = adeg[crop_idx,:]

    return adeg, d, intens


def coordinate_lidar2cam(xs, ys, zs, params_lidar, params_cam):
    '''
    transform the coordinate of the lidar points to the camera coordinate
    \n xs, ys, zs : xyz components of lidar points w.r.t a lidar coordinate
    \n params_lidar : parameters from lidars 
    \n params_cam : parameters from cameras 
    '''
    lidar_pos = [params_lidar.get(i) for i in (["X","Y","Z"])]
    cam_pos = [params_cam.get(i) for i in (["X","Y","Z"])]

    #Relative position of lidar w.r.t cam
    x_rel = lidar_pos[0] - cam_pos[0]
    y_rel = lidar_pos[1] - cam_pos[1]
    z_rel = lidar_pos[2] - cam_pos[2]

    #Eular angle of sensor
    lidar_yaw, lidar_pitch, lidar_roll = [params_lidar.get(i) for i in (["YAW","PITCH","ROLL"])]
    cam_yaw, cam_pitch, cam_roll = [params_cam.get(i) for i in (["YAW","PITCH","ROLL"])]

    #Transfrom matrix
    RV1 = RotationMatrix(np.deg2rad(0), np.deg2rad(0), np.deg2rad(lidar_pitch))
    RV2 = RotationMatrix(-np.deg2rad(lidar_yaw), np.deg2rad(0), np.deg2rad(0))
    RV3 = RotationMatrix(np.deg2rad(0), np.deg2rad(lidar_roll), np.deg2rad(0))

    R1 = RotationMatrix(0, 0, np.deg2rad(90))
    
    RC1 = RotationMatrix(np.deg2rad(cam_roll), np.deg2rad(0), np.deg2rad(0))
    RC2 = RotationMatrix(np.deg2rad(0), np.deg2rad(0), np.deg2rad(cam_pitch))
    RC3 = RotationMatrix(np.deg2rad(0), np.deg2rad(cam_yaw), np.deg2rad(0))
        
    TM1 = TraslationMatrix(y_rel, x_rel, z_rel)
    
    #reshape and concatenate the xyz of lidar signal
    xyz_c = np.concatenate([xs, ys, zs, np.ones_like(zs)], axis=1)
    
    #coordinate trasformation
    #rotate the coordinate of a lidar
    xyz_c = np.matmul(np.matmul(np.matmul(xyz_c, RV3.T), RV1.T), RV2.T) 
    #translate the coordinate of a lidar to cam
    xyz_c = np.matmul(xyz_c, TM1.T)

    #rotate the coordinate to general image coordinate
    xyz_c = np.matmul(xyz_c, R1.T)

    #rotate the coordinate from cam coordinate
    xyz_c = np.matmul(xyz_c, RC3)
    xyz_c = np.matmul(xyz_c, RC2)
    xyz_c = np.matmul(xyz_c, RC1)
    
    xc, yc, zc = xyz_c[:,0].reshape([-1,1]), xyz_c[:,1].reshape([-1,1]), xyz_c[:,2].reshape([-1,1])

    return xc, yc, zc


# def coordinate_lidar2cam(xs, ys, zs, params_lidar, params_cam):
#     '''
#     transform the coordinate of the lidar points to the camera coordinate
#     \n xs, ys, zs : xyz components of lidar points w.r.t a lidar coordinate
#     \n params_lidar : parameters from lidars 
#     \n params_cam : parameters from cameras 
#     '''
#     lidar_pos = [params_lidar.get(i) for i in (["X","Y","Z"])]
#     cam_pos = [params_cam.get(i) for i in (["X","Y","Z"])]

#     #Relative position of lidar w.r.t cam
#     x_rel = lidar_pos[0] - cam_pos[0]
#     y_rel = lidar_pos[1] - cam_pos[1]
#     z_rel = lidar_pos[2] - cam_pos[2]

#     #Eular angle of sensor
#     lidar_yaw, lidar_pitch, lidar_roll = [params_lidar.get(i) for i in (["YAW","PITCH","ROLL"])]
#     cam_yaw, cam_pitch, cam_roll = [params_cam.get(i) for i in (["YAW","PITCH","ROLL"])]

#     #Transfrom matrix
#     RV1 = RotationMatrix(np.deg2rad(0), np.deg2rad(0), np.deg2rad(lidar_pitch))
#     RV2 = RotationMatrix(np.deg2rad(lidar_yaw), np.deg2rad(0), np.deg2rad(0))
#     RV3 = RotationMatrix(np.deg2rad(0), np.deg2rad(lidar_roll), np.deg2rad(0))

#     R1 = RotationMatrix(0, 0, np.deg2rad(90))
    
#     RC1 = RotationMatrix(np.deg2rad(cam_roll), np.deg2rad(0), np.deg2rad(0))
#     RC2 = RotationMatrix(np.deg2rad(0), np.deg2rad(0), np.deg2rad(cam_pitch))
#     RC3 = RotationMatrix(np.deg2rad(0), np.deg2rad(cam_yaw), np.deg2rad(0))
        
#     TM1 = TraslationMatrix(-y_rel, x_rel, z_rel)
    
#     #reshape and concatenate the xyz of lidar signal
#     xyz_c = np.concatenate([xs, ys, zs, np.ones_like(zs)], axis=1)
    
#     #coordinate trasformation
#     #rotate and translate the coordinate of a lidar
#     T_velo2fr = np.matmul(np.matmul(np.matmul(np.matmul(np.matmul(np.matmul(np.matmul(RV3.T, RV1.T), RV2.T), TM1.T), R1.T), RC3), RC2), RC1).T
    
#     # print(T_velo2fr)
    
#     xyz_c = np.matmul(xyz_c, T_velo2fr.T)
    
#     xc, yc, zc = xyz_c[:,0].reshape([-1,1]), xyz_c[:,1].reshape([-1,1]), xyz_c[:,2].reshape([-1,1])

#     return xc, yc, zc


def transform_lidar2cam(params_lidar, params_cam):
    '''
    transform the coordinate of the lidar points to the camera coordinate
    \n xs, ys, zs : xyz components of lidar points w.r.t a lidar coordinate
    \n params_lidar : parameters from lidars 
    \n params_cam : parameters from cameras 
    '''
    lidar_pos = [params_lidar.get(i) for i in (["X","Y","Z"])]
    cam_pos = [params_cam.get(i) for i in (["X","Y","Z"])]

    #Relative position of lidar w.r.t cam
    x_rel = lidar_pos[0] - cam_pos[0]
    y_rel = lidar_pos[1] - cam_pos[1]
    z_rel = lidar_pos[2] - cam_pos[2]

    #Eular angle of sensor
    lidar_yaw, lidar_pitch, lidar_roll = [params_lidar.get(i) for i in (["YAW","PITCH","ROLL"])]
    cam_yaw, cam_pitch, cam_roll = [params_cam.get(i) for i in (["YAW","PITCH","ROLL"])]

    #Transfrom matrix
    RV1 = RotationMatrix(np.deg2rad(0), np.deg2rad(0), np.deg2rad(lidar_pitch))
    RV2 = RotationMatrix(np.deg2rad(lidar_yaw), np.deg2rad(0), np.deg2rad(0))
    RV3 = RotationMatrix(np.deg2rad(0), np.deg2rad(lidar_roll), np.deg2rad(0))

    R1 = RotationMatrix(0, 0, np.deg2rad(90))
    
    RC1 = RotationMatrix(np.deg2rad(cam_roll), np.deg2rad(0), np.deg2rad(0))
    RC2 = RotationMatrix(np.deg2rad(0), np.deg2rad(0), np.deg2rad(cam_pitch))
    RC3 = RotationMatrix(np.deg2rad(0), np.deg2rad(cam_yaw), np.deg2rad(0))
        
    TM1 = TraslationMatrix(-y_rel, x_rel, z_rel)
        
    #coordinate trasformation
    #rotate and translate the coordinate of a lidar
    R_T = np.matmul(np.matmul(np.matmul(np.matmul(np.matmul(np.matmul(np.matmul(RV3.T, RV1.T), RV2.T), TM1.T), R1.T), RC3), RC2), RC1).T
    
    print('r : \n')

    print(R_T[:3,:3])

    print('t : \n')

    print(R_T[:3,3])

    return R_T


def project2img(xc, yc, zc, params_cam):

    '''
    project the lidar points to 2d plane
    \n xc, yc, zc : xyz components of lidar points w.r.t a camera coordinate
    \n params_cam : parameters from cameras 

    '''
    #focal lengths
    fc_x = params_cam["WIDTH"]/(2*np.tan(np.deg2rad(params_cam["FOV"]/2)))
    fc_y = params_cam["WIDTH"]/(2*np.tan(np.deg2rad(params_cam["FOV"]/2)))

    #the center of image
    cx = params_cam["WIDTH"]/2
    cy = params_cam["HEIGHT"]/2
    
    #transformation matrix from 3D to 2D
    R_f = np.array([[fc_x,  0,      cx],
                    [0,     fc_y,   cy]])

    #2d lidar points on the normalized plane 
    xyn = np.concatenate([xc/(zc+0.001), yc/(zc+0.001)], axis=1)

    #mapping 2d lidar points on the image plane 
    xyc = np.concatenate([xyn, np.ones_like(zc)], axis=1)
    xy_i = np.matmul(xyc, R_f.T)
    
    xi, yi = xy_i[:,0].reshape([-1,1]), xy_i[:,1].reshape([-1,1])

    return xi, yi


def crop_points(xi, yi, intens, img_w, img_h):
    '''
    crop the lidar points on images within width and height
    \n xi, yi : xy components of lidar points w.r.t a 2d plane
    \n intens : intensities of lidar points
    \n img_w, img_h : a width and a height of a image from a camera

    '''
    #cut the lidar points out of width
    crop_x_max_idx = np.where(xi<img_w)[0] 

    xi = xi[crop_x_max_idx]
    yi = yi[crop_x_max_idx]
    intens = intens[crop_x_max_idx]

    crop_x_min_idx = np.where(xi>=0)[0]

    xi = xi[crop_x_min_idx]
    yi = yi[crop_x_min_idx]
    intens = intens[crop_x_min_idx]

    #cut the lidar points out of height
    crop_y_max_idx = np.where(yi<img_h)[0]

    xi = xi[crop_y_max_idx]
    yi = yi[crop_y_max_idx]
    intens = intens[crop_y_max_idx]

    crop_y_min_idx = np.where(yi>=0)[0]

    xi = xi[crop_y_min_idx]
    yi = yi[crop_y_min_idx]
    intens = intens[crop_y_min_idx]

    return xi, yi, intens

def make_intensity_img(xi, yi, intens, img_w, img_h):
    '''
    place the lidar points into numpy arrays in order to make intensity map
    \n xi, yi : xy components of lidar points w.r.t a 2d normalized plane
    \n intens : intensities of lidar points
    \n img_w, img_h : a width and a height of a image from a camera
    '''
    point_np = np.zeros((img_h,img_w,3), dtype=np.uint8)

    #Asphalt
    point_np[yi[intens==0].astype(np.int),xi[intens==0].astype(np.int),1] = 255
    
    #Crosswalk
    point_np[yi[intens==20].astype(np.int),xi[intens==20].astype(np.int),0] = 255

    #Lane
    point_np[yi[intens==50].astype(np.int),xi[intens==50].astype(np.int),:] = 255

    #Object
    point_np[yi[intens>=200].astype(np.int),xi[intens>=200].astype(np.int),2] = 255
    

    return point_np

def dilate_point_np(point_np, params_visual):
    '''
    dilate the lidar point lines
    \n point_np: lidar point images based on numpy array
    \n params_visual : parameters of visualization 
    '''
    ks=params_visual["DILATION_SIZE"]

    kernel = cv2.getStructuringElement(params_visual["DILATION_KERNAL"], (ks, ks))
    dilation = cv2.dilate(point_np,kernel,iterations = 1)

    return dilation


def make_intensity_img_v2(xi, yi, intens, img_w, img_h, clr_map):
    '''
    place the lidar points into numpy arrays in order to make intensity map
    \n xi, yi : xy components of lidar points w.r.t a 2d normalized plane
    \n intens : intensities of lidar points
    \n img_w, img_h : a width and a height of a image from a camera
    '''
    intens_max = int(255/3)

    point_np = np.zeros((img_h,img_w,1), dtype=np.uint8)
    point_binary = np.zeros((img_h,img_w,3), dtype=np.uint8)

    point_np[yi.astype(np.int), xi.astype(np.int), :] = ((np.clip(intens,0,intens_max)).reshape([-1,1,1])/intens_max*255).astype(np.uint8)
    point_binary[yi.astype(np.int), xi.astype(np.int), :] = 1

    point_np = cv2.applyColorMap(point_np, clr_map)

    point_np = point_np*point_binary

    return point_np


def make_distance_img(xi, yi, distance, img_w, img_h, dis_max, clr_map):
    '''
    place the lidar points into numpy arrays in order to make distance map
    \n xi, yi : xy components of lidar points w.r.t a 2d normalized plane
    \n distance : distance measurement from the origin of the lidar coordinate
    \n img_w, img_h : a width and a height of a image from a camera
    \n dis_max : maximum of distance shown in the distance map 
    \n clr_map : colormap
    '''
    point_np = np.zeros((img_h,img_w,1), dtype=np.uint8)
    point_binary = np.zeros((img_h,img_w,3), dtype=np.uint8)

    point_np[yi.astype(np.int), xi.astype(np.int), :] = (np.clip(distance,0,dis_max).reshape([-1,1,1])/dis_max*255).astype(np.uint8)
    point_binary[yi.astype(np.int), xi.astype(np.int), :] = 1

    point_np = cv2.applyColorMap(point_np, clr_map)

    point_np = point_np*point_binary

    return point_np


def visualize_lidar_img(azimuth, distance, intensity, params_lidar, params_cam, params_visual, R_T):
    '''
    \n azimuth, distance, intensity : measurements from the lidar
    \n params_lidar : parameters from lidars 
    \n params_cam : parameters from cameras 
    \n params_visual : parameters of visualization 
    '''

    if params_lidar["CHANNEL"]==16:
        VerticalAngleDeg = VerticalAngleDeg_16
    else:
        VerticalAngleDeg = VerticalAngleDeg_32
    
    xs, ys, zs = sph2cart(distance, azimuth, VerticalAngleDeg)

    # reshape the vectors on 16 channels into 1 axis
    xs = xs.reshape([-1,1])
    ys = ys.reshape([-1,1])
    zs = zs.reshape([-1,1])

    # apply the transformation matrix R, T
    xyz_c = np.matmul(np.concatenate([xs, ys, zs, np.ones_like(zs)], axis=1), R_T.T)
    xc, yc, zc = xyz_c[:,0].reshape([-1,1]), xyz_c[:,1].reshape([-1,1]), xyz_c[:,2].reshape([-1,1])
    xi, yi = project2img(xc, yc, zc, params_cam)
    
    if params_visual["USE_INTENSITY_MAP"]:
        intensity = intensity.reshape([-1,1])
        xi, yi, intensity = crop_points(xi, yi, intensity, params_cam["WIDTH"], params_cam["HEIGHT"])
        point_img = make_intensity_img(xi, yi, intensity, params_cam["WIDTH"], params_cam["HEIGHT"])
        # point_img = make_intensity_img_v2(xi, yi, intensity, params_cam["WIDTH"],
        #                                                 params_cam["HEIGHT"],
        #                                                 params_visual["COLORMAP"])

    else:
        distance = distance.reshape([-1,1])
        xi, yi, distance = crop_points(xi, yi, distance, params_cam["WIDTH"], params_cam["HEIGHT"])
        point_img = make_distance_img(xi, yi, distance, params_cam["WIDTH"],
                                                        params_cam["HEIGHT"],
                                                        params_visual["DISTANCE_MAX"],
                                                        params_visual["COLORMAP"])

    point_img = dilate_point_np(point_img, params_visual)

    return point_img


def visualize_lidar_img_v2(azimuth, distance, intensity, params_lidar, params_cam, params_visual):
    '''
    \n azimuth, distance, intensity : measurements from the lidar
    \n params_lidar : parameters from lidars 
    \n params_cam : parameters from cameras 
    \n params_visual : parameters of visualization 
    '''

    if params_lidar["CHANNEL"]==16:
        VerticalAngleDeg = VerticalAngleDeg_16
    else:
        VerticalAngleDeg = VerticalAngleDeg_32
    
    xs, ys, zs = sph2cart(distance, azimuth, VerticalAngleDeg)

    
    xs = xs.reshape([-1,1])
    ys = ys.reshape([-1,1])
    zs = zs.reshape([-1,1])

    xc, yc, zc = coordinate_lidar2cam(xs, ys, zs, params_lidar, params_cam)

    xi, yi = project2img(xc, yc, zc, params_cam)
    
    if params_visual["USE_INTENSITY_MAP"]:
        intensity = intensity.reshape([-1,1])
        xi, yi, intensity = crop_points(xi, yi, intensity, params_cam["WIDTH"], params_cam["HEIGHT"])
        point_img = make_intensity_img(xi, yi, intensity, params_cam["WIDTH"], params_cam["HEIGHT"])
        # point_img = make_intensity_img_v2(xi, yi, intensity, params_cam["WIDTH"],
        #                                                 params_cam["HEIGHT"],
        #                                                 params_visual["COLORMAP"])

    else:
        distance = distance.reshape([-1,1])
        xi, yi, distance = crop_points(xi, yi, distance, params_cam["WIDTH"], params_cam["HEIGHT"])
        point_img = make_distance_img(xi, yi, distance, params_cam["WIDTH"],
                                                        params_cam["HEIGHT"],
                                                        params_visual["DISTANCE_MAX"],
                                                        params_visual["COLORMAP"])

    point_img = dilate_point_np(point_img, params_visual)

    return point_img