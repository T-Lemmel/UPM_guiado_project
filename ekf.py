import numpy as np
import math
from utils import euclidean_distance, normalize_angle

def initialize_parameters(x0,y0,yaw0):    
    
    # Initial state
    Xk = np.array([x0, y0, yaw0])

    # Initial covariance matrix
    Px_init, Py_init, Pyaw_init = 0.335, 0.335, 0.0827
    Pk = np.array([[Px_init, 0, 0], [0, Py_init, 0], [0, 0, Pyaw_init]])

    # Measurement noise covariance
    angle_variance = 0.00045
    R = np.array([angle_variance])

    return Xk, Pk, R


def detect_landmark(x, y, yaw, landmark_positions,ap=None):
    landmark_data = ap.getLaserLandMarks("LMS100")
   
    # Remove the human body from the list
    landmark_data = [landmark for landmark in landmark_data if landmark[0] != 0] 
    
    # Get the closest landmark as at a larger distance, the same bearing noise will have a larger impact on the observed balisa position
    closest_landmark = min(landmark_data, key=lambda landmark: landmark[2], default=None) 
    
    for i, refence_pos in enumerate(landmark_positions):
        if closest_landmark is not None :
            if closest_landmark[0] == i+1:
                px, py = refence_pos[0], refence_pos[1]
                
                bearing_measure = normalize_angle(closest_landmark[1])

                # The observation is the angle to the landmark
                Zk = np.array([bearing_measure])

                # The ground truth is the actual angle to the landmark
                real_bearing = normalize_angle(math.atan2(py - y, px - x) - yaw)

                Zk_predicted = np.array([real_bearing])

                # The Jacobian of the observation model
                H_1_1 = (py - y) / euclidean_distance(px, py, x, y) ** 2
                H_1_2 = (x - px) / euclidean_distance(px, py, x, y) ** 2

                Hk = np.array([[H_1_1, H_1_2, -1]])

                return Zk, Zk_predicted, Hk
    
    return None, None, None


def ekf_step(Xk, Pk, R, linear_speed, angular_speed, command_duration, robot, landmarks, odometry, ap):
    Xk_next = Xk.copy()
    x_increment, y_increment, yaw_increment = odometry

    # Predict the next state
    Xk_next[0] += x_increment * math.cos(Xk[2]) - y_increment * math.sin(Xk[2])
    Xk_next[1] += y_increment * math.cos(Xk[2]) + x_increment * math.sin(Xk[2])
    Xk_next[2] += yaw_increment
    Xk_next[2] = normalize_angle(Xk_next[2])

    # Calculate the Jacobian of the motion model
    Ak = np.array([
        [1, 0, -linear_speed * command_duration * np.sin(Xk[2] + angular_speed * command_duration / 2)],
        [0, 1, linear_speed * command_duration * np.cos(Xk[2] + angular_speed * command_duration / 2)],
        [0, 0, 1]
    ])
    
    # Calculate the Jacobian of the control model
    Bk = np.array([
        [command_duration * np.cos(Xk[2] + angular_speed * command_duration / 2), -0.5 * linear_speed * command_duration ** 2 * np.sin(Xk[2] + angular_speed * command_duration / 2)],
        [command_duration * np.sin(Xk[2] + angular_speed * command_duration / 2), 0.5 * linear_speed * command_duration ** 2 * np.cos(Xk[2] + angular_speed * command_duration / 2)],
        [0, command_duration]
    ])
    
    # Calculate the process noise covariance
    Ql = 0.01 * linear_speed * command_duration
    Qa = 0.01 * angular_speed * command_duration
    Qk = np.array([[Ql, 0], [0, Qa]])

    # Predict the next covariance
    Pk_next = Ak @ Pk @ Ak.T + Bk @ Qk @ Bk.T

    # Get the observation from the landmark recognition function
    Zk, Zk_predicted, Hk = detect_landmark(Xk_next[0], Xk_next[1], Xk_next[2], landmarks, ap)

    if Zk is not None:

        # Calculate the prediction of the observation
        Yk = Zk - Zk_predicted

        # Increase the size of R to match the size of Yk
        Sk = Hk @ Pk @ Hk.T + R

        # Calculate the Kalman gain
        Wk = Pk @ Hk.T @ np.linalg.pinv(Sk) 
        Wk = Wk/1000 # Experimentally found necessary to stabilize the filter

        # Update the state estimate
        Xk_next = Xk_next + Wk @ Yk

        # Update the covariance estimate
        Pk_next = (np.eye(3) - Wk @ Hk) @ Pk_next

    return Xk_next, Pk_next