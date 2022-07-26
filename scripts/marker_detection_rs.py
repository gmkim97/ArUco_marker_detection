#!/usr/bin/env python3

import rospy
import cv2
from cv2 import aruco
from cv_bridge import CvBridge, CvBridgeError
import pyrealsense2 as rs
import tf
import numpy as np
import math
from std_msgs.msg import Header
from geometry_msgs.msg import Transform, Vector3, Quaternion, TransformStamped
from sensor_msgs.msg import Image, CameraInfo

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

class marker_detection:
    def __init__(self, topic1, topic2, topic3):
        
        self.topic1 = topic1
        self.topic2 = topic2
        self.topic3 = topic3
        self.bridge = CvBridge()

        self.sub = rospy.Subscriber(topic1, Image, self.rgbMat_callback)
        self.sub2 = rospy.Subscriber(topic2, CameraInfo, self.rgbimgInfo_callback)
        self.sub3 = rospy.Subscriber(topic3, Image, self.marker_detection)
        
        self.intrinsics = None
    
    # Convert rgb image to rgb Matrix
    def rgbMat_callback(self, rgb_img):

        global rgb_Mat
        try:
            rgb_Mat = self.bridge.imgmsg_to_cv2(rgb_img, rgb_img.encoding)

        except CvBridgeError as e:
            print(e)
            return

    # Return intrinsic informations of depth camera
    def rgbimgInfo_callback(self, cameraInfo):

        try:
            if self.intrinsics:
                return
            self.intrinsics = rs.intrinsics()
            self.intrinsics.width = cameraInfo.width
            self.intrinsics.height = cameraInfo.height
            self.intrinsics.ppx = cameraInfo.K[2]
            self.intrinsics.ppy = cameraInfo.K[5]
            self.intrinsics.fx = cameraInfo.K[0]
            self.intrinsics.fy = cameraInfo.K[4]
            self.intrinsics.model = rs.distortion.none
            self.intrinsics.coeffs = [i for i in cameraInfo.D]
        except CvBridgeError as e:
            print(e)
            return

    # Main callback func.
    # Generate axes on the aruco marker
    # Publish tf topics (ROS)
    def marker_detection(self, depth_img):

        global depth_Mat
        try:
            depth_Mat = self.bridge.imgmsg_to_cv2(depth_img, depth_img.encoding)

        except CvBridgeError as e:
            print(e)
            return
        
        br = tf.TransformBroadcaster()
        listener = tf.TransformListener()
        rate = rospy.Rate(10.0)

        fx = self.intrinsics.fx
        fy = self.intrinsics.fy
        ppx = self.intrinsics.ppx
        ppy = self.intrinsics.ppy

        camera_mat = np.array([[fx, 0, ppx], [0, fy, ppy], [0 ,0 ,1]], dtype=np.float)
        distCoeffs = np.zeros(4)
        rgb = rgb_Mat
        depth = depth_Mat
        bgr = cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)
        gray_img = cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY)
        parameters = aruco.DetectorParameters_create()
        corners, ids, _ = aruco.detectMarkers(gray_img, aruco_dict, parameters=parameters)
        frame_markers = aruco.drawDetectedMarkers(bgr, corners, ids)

        if np.shape(corners)[0] > 0:
                for i in range(np.shape(corners)[0]):
                    rvecs, tvecs, _ = aruco.estimatePoseSingleMarkers(corners[i], marker_size, cameraMatrix=camera_mat, distCoeffs=distCoeffs)
                    frame_markers = cv2.drawFrameAxes(frame_markers, cameraMatrix=camera_mat, distCoeffs=distCoeffs, rvec=rvecs, tvec=tvecs, length=0.050, thickness=2)
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

                    # print(dist)

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


if __name__ == '__main__':

    aruco_dict = aruco.Dictionary_get(aruco.DICT_6X6_250)
    marker_size = 0.068 # actual marker size in meters
    
    rospy.init_node("marker_detection")

    topic1 = "/camera/color/image_raw"
    topic2 = "/camera/aligned_depth_to_color/camera_info"
    topic3 = "/camera/aligned_depth_to_color/image_raw"

    detection = marker_detection(topic1, topic2, topic3)
    rospy.spin()