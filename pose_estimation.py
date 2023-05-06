'''
Sample Usage:-
python pose_estimation.py --K_Matrix calibration_matrix.npy --D_Coeff distortion_coefficients.npy --type DICT_5X5_100
Prints pixel information to redis 
Change your marker size in the cv2.aruco.estimatePoseSingleMarkers() function 
'''

import numpy as np
import cv2
import sys
from utils import ARUCO_DICT
import argparse
import time
import redis 

'''
Redis setup
'''
r = redis.Redis(host='localhost', port=6379, db=0)
aruco_marker_key = "aruco::marker"

def pose_estimation(frame, aruco_dict_type, matrix_coefficients, distortion_coefficients):
    '''
    frame - Frame from the video stream
    matrix_coefficients - Intrinsic matrix of the calibrated camera
    distortion_coefficients - Distortion coefficients associated with your camera

    return:-
    frame - The frame with the axis drawn on it
    '''

    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    cv2.aruco_dict = cv2.aruco.Dictionary_get(aruco_dict_type)
    parameters = cv2.aruco.DetectorParameters_create()
    parameters.useAruco3Detection = True
    # Add additional parameters here from reference:
    # https://docs.opencv.org/4.x/d1/dcd/structcv_1_1aruco_1_1DetectorParameters.html

    corners, ids, rejected_img_points = cv2.aruco.detectMarkers(gray, cv2.aruco_dict,parameters=parameters)

    # If markers are detected
    if len(corners) > 0:
        for i in range(0, len(ids)):
            # Estimate pose of each marker and return the values rvec and tvec---(different from those of camera coefficients)
            rvec, tvec, markerPoints = cv2.aruco.estimatePoseSingleMarkers(corners[i], 0.0762, matrix_coefficients, distortion_coefficients)
            # Draw a square around the markers
            cv2.aruco.drawDetectedMarkers(frame, corners) 
            
            # Convert rvec and tvec to transformation matrix (rodrigues vector, translation vector)
            # Transformation matrix from marker coordinate frame to camera coordinate frame 
            transformation_matrix = np.array([[0, 0, 0, 0],
                                                [0, 0, 0, 0],
                                                [0, 0, 0, 0],
                                                [0, 0, 0, 1]],
                                                dtype=float)
            transformation_matrix[:3, :3], _ = cv2.Rodrigues(rvec)
            transformation_matrix[:3, -1] = tvec
            
            # Send data to redis server (make sure to send to the correct marker id key if more than one tag)
            r.set(aruco_marker_key, transformation_matrix.tobytes())
            # t_matrix_check = np.reshape(np.frombuffer(r.get(aruco_marker_key)), (4, 4))  # to read the transformation matrix 
            # if (np.linalg.norm(transformation_matrix - t_matrix_check)) > 1e-6:
            #     print("Error")
                            
            # Draw Axis
            # cv2.aruco.drawAxis(frame, matrix_coefficients, distortion_coefficients, rvec, tvec, 0.01)  
            cv2.drawFrameAxes(frame, matrix_coefficients, distortion_coefficients, rvec, tvec, 0.05)  

    return frame

if __name__ == '__main__':

    ap = argparse.ArgumentParser()
    ap.add_argument("-k", "--K_Matrix", required=True, help="Path to calibration matrix (numpy file)")
    ap.add_argument("-d", "--D_Coeff", required=True, help="Path to distortion coefficients (numpy file)")
    ap.add_argument("-t", "--type", type=str, default="DICT_ARUCO_ORIGINAL", help="Type of ArUCo tag to detect")
    args = vars(ap.parse_args())
    
    if ARUCO_DICT.get(args["type"], None) is None:
        print(f"ArUCo tag type '{args['type']}' is not supported")
        sys.exit(0)

    aruco_dict_type = ARUCO_DICT[args["type"]]
    calibration_matrix_path = args["K_Matrix"]
    distortion_coefficients_path = args["D_Coeff"]
    
    k = np.load(calibration_matrix_path)
    d = np.load(distortion_coefficients_path)

    video = cv2.VideoCapture(1)
    time.sleep(0.1)

    while True:
        ret, frame = video.read()

        if not ret:
            break
        
        h, w = frame.shape[:2]
        newcameramtx, roi = cv2.getOptimalNewCameraMatrix(k, d, (w,h), 1, (w,h))
        dst = cv2.undistort(frame, k, d, None, newcameramtx)
        x, y, w, h = roi
        dst = dst[y:y+h, x:x+w]
        
        output = pose_estimation(dst, aruco_dict_type, k, d)

        cv2.imshow('Estimated Pose', output)

        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):
            break

    video.release()
    cv2.destroyAllWindows()