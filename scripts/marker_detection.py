#!/usr/bin/env python3

import rospy
import cv2
from cv2 import aruco
import tf
import numpy as np
import math
from camera import Camera
from std_msgs.msg import Header
from geometry_msgs.msg import Transform, Vector3, Quaternion, TransformStamped

def RotationToEuler(R):
    sy = math.sqrt(R[0,0] * R[0,0] +  R[1,0] * R[1,0])
    singular = sy < 1e-6

    if not singular:
        x = math.atan2(R[2,1], R[2,2])
        y = math.atan2(-R[2,0], sy)
        z = math.atan2(R[1,0], R[0,0])
    else:
        x = math.atan2(-R[1,2], R[1,1])
        y = math.atan2(-R[2,0], sy)
        z = 0
    
    return np.array([x,y,z])

if __name__ == '__main__':

    cam = Camera()
    aruco_dict = aruco.Dictionary_get(aruco.DICT_6X6_250)
    marker_size = 0.068 # actual marker size in meters

    rospy.init_node("marker_detection")

    br = tf.TransformBroadcaster()
    listener = tf.TransformListener()
    rate = rospy.Rate(10.0)

    while not rospy.is_shutdown():
        rgb, depth = cam.stream()
        gray_img = cv2.cvtColor(rgb, cv2.COLOR_BGR2GRAY)
        parameters = aruco.DetectorParameters_create()
        corners, ids, _ = aruco.detectMarkers(gray_img, aruco_dict, parameters=parameters)
        frame_markers = aruco.drawDetectedMarkers(rgb.copy(), corners, ids)
        
        if np.shape(corners)[0] > 0:
                for i in range(np.shape(corners)[0]):
                    rvecs, tvecs, _ = aruco.estimatePoseSingleMarkers(corners[i], marker_size, cameraMatrix=cam.camera_mat, distCoeffs=cam.distCoeffs)
                    frame_markers = cv2.drawFrameAxes(frame_markers, cameraMatrix=cam.camera_mat, distCoeffs=cam.distCoeffs, rvec=rvecs, tvec=tvecs, length=0.050, thickness=2)
                    ## for SE3 trasnformation matrix (marker with respect to the camera)
                    R, _ = cv2.Rodrigues(rvecs)
                    tvecs = np.reshape(tvecs, (3, 1))
                    cam2marker = np.concatenate((R, tvecs), axis = 1)
                    ## This 3 X 4 sample array ONLY includes rotation and position matrix
                    sample = cam2marker
                    ## add [0, 0, 0, 1] to make it SE3 format
                    cam2marker = np.concatenate((cam2marker, np.array([[0, 0, 0, 1]])), axis = 0)
                    
                    pos_mat = np.dot(sample,[0, 0, 0, 1])
                    rot_mat = sample[:,0:3]
                    euler = RotationToEuler(rot_mat)
                    
                    xval = pos_mat[0]
                    yval = pos_mat[1]
                    zval = pos_mat[2]
                    sum = math.pow(xval,2) + math.pow(yval,2) + math.pow(zval,2)

                    dist = 2 * math.sqrt(sum)

                    print(dist)

                    # These codes are for testing appropriate variable values
                    # print(ids)
                    # print(pos_mat)
                    # print(sample)
                    # print(rot_mat)
                    # print(RotationToEuler(rot_mat))
                    # print(len(ids))
                    
                    trans = Transform(translation=Vector3(pos_mat[2], -pos_mat[0], -pos_mat[1]),
                            rotation=Quaternion(*tf.transformations.quaternion_from_euler(euler[0], euler[1], euler[2])) # quaternion_from_euler(euler angles: yaw, pitch, roll)
                            )

                    header = Header()
                    header.stamp = rospy.Time.now()
                    header.frame_id = 'camera_link'   # the parent link

                    trans_stamp = TransformStamped(header, 'aruco_marker' + str(ids), trans)
                    br.sendTransformMessage(trans_stamp)


        cv2.imshow("res", frame_markers)
        cv2.waitKey(1)