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
# r.setX, setY, setTheta
aruco_marker_key = "aruco::marker"
x_pos_key = "mallet::x"
y_pos_key = "mallet::y"
ori_key = "mallet::theta"

def pose_estimation(frame, aruco_dict_type, matrix_coefficients, distortion_coefficients):
    '''
    frame - Frame from the video stream
    matrix_coefficients - Intrinsic matrix of the calibrated camera
    distortion_coefficients - Distortion coefficients associated with your camera

    return:-
    frame - The frame with the axis drawn on it
    '''

    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    aruco_dict = cv2.aruco.getPredefinedDictionary(aruco_dict_type)
    aruco_parameters = cv2.aruco.DetectorParameters()
    aruco_detector = cv2.aruco.ArucoDetector(aruco_dict, aruco_parameters)
    # parameters.useAruco3Detection = True
    # Add additional parameters here from reference:
    # https://docs.opencv.org/4.x/d1/dcd/structcv_1_1aruco_1_1DetectorParameters.html

    corners, ids, rejected_img_points = aruco_detector.detectMarkers(gray)

    # If markers are detected
    if len(corners) > 0:
        for i in range(0, len(ids)):
            # Estimate pose of each marker and return the values rvec and tvec---(different from those of camera coefficients)
            marker_size = 0.036
            marker_points = np.array([[-marker_size / 2, marker_size / 2, 0],
                              [marker_size / 2, marker_size / 2, 0],
                              [marker_size / 2, -marker_size / 2, 0],
                              [-marker_size / 2, -marker_size / 2, 0]], dtype=np.float32)
            markPts = []
            rvec = []
            tvec = []
            for c in corners:
                mp, R, t = cv2.solvePnP(marker_points, c, matrix_coefficients, distortion_coefficients, False, cv2.SOLVEPNP_IPPE_SQUARE)
                rvec.append(R)
                tvec.append(t)
                markPts.append(mp)
            rvec = np.array(rvec)
            tvec = np.array(tvec)
            # Draw a square around the markers
            cv2.aruco.drawDetectedMarkers(frame, corners) 
            
            # # Convert rvec and tvec to transformation matrix (rodrigues vector, translation vector)
            # # Transformation matrix from marker coordinate frame to camera coordinate frame 
            if rvec.shape[0] == 1 and tvec.shape[0] == 1:
                transformation_matrix = np.array([[0, 0, 0, 0],
                                                    [0, 0, 0, 0],
                                                    [0, 0, 0, 0],
                                                    [0, 0, 0, 1]],
                                                    dtype=float)
                transformation_matrix[:3, :3], _ = cv2.Rodrigues(rvec)
                transformation_matrix[:3, [-1]] = tvec
                
                # Send data to redis server (make sure to send to the correct marker id key if more than one tag)
                # r.set(aruco_marker_key, transformation_matrix)
                r.set(x_pos_key, tvec[0,0,0])
                r.set(y_pos_key, tvec[0,2,0])
                # rotation about the z-axis
                # converted to degrees (0 - 360)
                angle = (180.0 / np.pi) * (np.arctan2(transformation_matrix[1,0], transformation_matrix[0,0]) + np.pi)
                r.set(ori_key, angle)
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

    video = cv2.VideoCapture(0)
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