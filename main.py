import math
import time
from apolo import Apolo
from graph import create_graph, add_node_and_find_path
from utils import euclidean_distance, angle_difference, detect_balisa, avoid_obstacle, place_victim
from ekf import initialize_parameters, ekf_step
import matplotlib.pyplot as plt

class RobotNavigator:
    def __init__(self, robot_name, apolo_instance):
        self.ap = apolo_instance
        self.robot = robot_name

        # Initialize EKF parameters
        self.Xk, self.Pk, self.R = initialize_parameters(0.0, 0.0, math.pi / 2)

        # Tracking states for plotting
        self.X_states = []
        self.odometry_values = [] # Odometry values are [x_increment, y_increment, yaw_increment]
        self.odometry_states = [] # Odometry states are [x, y, yaw], used to plot the odometry path
        self.ground_truth_states = []

        # Navigation and state flags
        self.STOP_FLAG = False # Flag to stop the navigation
        self.going_to_balisa = False # Flag to indicate if the robot is going to a balisa
        self.latest_point = [0,0] # Latest point reached by the robot
        self.direct_to_target = False # Flag to go directly to target after reaching first intermediate point

        # PI Controller parameters
        self.Kp_angle = 1.0
        self.Ki_angle = 0.1
        self.integral_angle = 0.0
        self.Kp_distance = 2.0
        self.Ki_distance = 0.05
        self.integral_distance = 0.0

        # Landmark for loop closure
        self.landmark_positions = [[]]

        # Obstacle avoidance parameters
        self.safety_margin = 0.3 # Safety margin to avoid obstacles
        self.avoidance_offset = 0.5 # Offset to avoid obstacles

    def iterate_simulation(self, linear_speed, angular_speed, command_duration):
        """Function to iterate the simulation, update EKF, and track states."""
        if self.STOP_FLAG: 
            return
        self.ap.moveWheeledBase(self.robot, linear_speed, angular_speed, command_duration)
        self.ap.updateWorld()
        time.sleep(command_duration)

        odometry = self.ap.getOdometry(self.robot) # Get odometry values, the odometry is [x_increment, y_increment, yaw_increment]
        self.ap.resetOdometry(self.robot) # Reset odometry 
        if self.odometry_values == []: # for some reason the first odometry is always wayyy off so it's better to ignore it
            self.odometry_values.append([0,0,0])
            odometry = [0,0,0]
        else:
            self.odometry_values.append(odometry)

        Xk_next,Pk_next = ekf_step(
            self.Xk, self.Pk, self.R, linear_speed, angular_speed, command_duration,
            self.robot, self.landmark_positions, odometry, self.ap
        ) 
        self.Xk = Xk_next # Update the state using the EKF
        self.Pk = Pk_next
        self.X_states.append(self.Xk) # Append the state to the list of states
        self.ground_truth_states.append(self.ap.getLocationWB(self.robot)) # Append the ground truth to the list of states for plotting

    def navigate_to(self, x, y, is_an_intermediate_point):
        """Navigate to a specific target point."""

        if is_an_intermediate_point == 1 and self.direct_to_target:
            return

        if self.STOP_FLAG: # If the task is finished, stop the navigation
            return

        print(f"Navigating to point ({x}, {y})")
        print("current position: ", self.Xk[:2])

        while not self.STOP_FLAG:
            if not self.going_to_balisa: # If the robot is not going to the human body, check if the corresponding balisa is detected
                found_balisa, x_balisa, y_balisa = detect_balisa(self.robot, target_id=0, ap=self.ap)
                if found_balisa:
                    print(f"Balisa found! Navigating to ({x_balisa}, {y_balisa})")
                    self.latest_point = (x,y)
                    self.going_to_balisa = True # Set the flag to indicate that the robot is going to the balisa and stop looking for it
                    self.navigate_to(x_balisa, y_balisa, False) # Navigate to the balisa
                    time.sleep(5)
                    print("Establishing connection with the medical team...")
                    self.get_home() # Once the balisa is reached, return to the home position backtracking
                    return

            # To go to any point, the robot must first align with the point with a PI controller and 2° tolerance
            angle = angle_difference(x, y, self.Xk)
            while abs(angle) >= (2 * math.pi / 180):
                if self.STOP_FLAG:
                    return
                if not self.going_to_balisa: # If the robot is not going to the balisa, check for it
                    found_balisa, x_balisa, y_balisa = detect_balisa(self.robot, target_id=0, ap=self.ap)
                    if found_balisa:
                        print(f"Balisa found! Navigating to ({x_balisa}, {y_balisa})")
                        self.latest_point = (x,y)
                        self.going_to_balisa = True
                        self.navigate_to(x_balisa, y_balisa, False)
                        time.sleep(5)
                        print("Establishing connection with the medical team...")
                        self.get_home()
                        return
                angle = angle_difference(x, y, self.Xk)
                self.integral_angle += angle
                angular_speed = self.Kp_angle * angle + self.Ki_angle * self.integral_angle
                self.iterate_simulation(0, angular_speed, 0.01)
            print("Initial alignment complete!")
            self.integral_angle = 0

            # Once the robot is aligned with the point, it must move forward to reach it with a PI controller and 5 cm tolerance
            print("going forward...")
            while not self.STOP_FLAG:
                if not self.going_to_balisa:
                    found_balisa, x_balisa, y_balisa = detect_balisa(self.robot, target_id=0, ap=self.ap)
                    if found_balisa:
                        print(f"Balisa found! Navigating to ({x_balisa}, {y_balisa})")
                        self.latest_point = (x,y)
                        self.going_to_balisa = True
                        self.navigate_to(x_balisa, y_balisa, False)
                        time.sleep(5)
                        print("Establishing connection with the medical team...")
                        self.get_home()
                        return

                x_robot, y_robot = self.Xk[0], self.Xk[1]
                distance = euclidean_distance(x, y, x_robot, y_robot)
                angle = angle_difference(x, y, self.Xk)

                # Check for obstacles and add intermediate points to avoid them before going forward
                obstacle_detected, intermediate_x, intermediate_y = avoid_obstacle(
                    self.robot, x, y, self.safety_margin, self.avoidance_offset, ap=self.ap
                )

                if obstacle_detected:
                    print(f"Obstacle detected! Adding intermediate point ({intermediate_x}, {intermediate_y})")
                    self.navigate_to(intermediate_x, intermediate_y, True)

                if distance < 0.05: # If the robot is close enough to the target, stop the navigation
                    if is_an_intermediate_point == 1:
                        direct_to_target = True
                    else:
                        direct_to_target = False
                    print("Target reached!", is_an_intermediate_point, direct_to_target)
                    if not self.going_to_balisa:
                        self.latest_point = (x, y)
                    return
                
                if abs(angle) > (10 * math.pi / 180): # If the robot is misaligned during forward motion, realign it with a PI controller and 5° tolerance
                    print("Triggering realignment during forward motion...")
                    while abs(angle) > (5 * math.pi / 180):
                        angle = angle_difference(x, y, self.Xk)
                        self.integral_angle += angle
                        angular_speed = self.Kp_angle * angle + self.Ki_angle * self.integral_angle
                        self.iterate_simulation(0, angular_speed, 0.01)
                    print("Realignment complete!")
                    self.integral_angle = 0

                self.integral_distance += distance
                linear_speed = self.Kp_distance * distance + self.Ki_distance * self.integral_distance
                self.iterate_simulation(linear_speed, 0, 0.01)

    def get_home(self):
        """Return the robot to the home position."""
        if self.STOP_FLAG:
            return

        robot_pose = self.Xk[:2]
        G, nodes = create_graph()
        print("latest_point: ", self.latest_point)
        path_to_home = add_node_and_find_path(G, nodes, (robot_pose[0], robot_pose[1]), (self.latest_point[0], self.latest_point[1]))

        for point in path_to_home[1:]:
            self.navigate_to(point[0], point[1], False)
            print(f"Reached reverse point ({point[0]}, {point[1]})")

        print("Reached home (0,0). Task complete.")
        self.STOP_FLAG = True

    def plot(self):
        """Plot the paths and covariance matrix."""
        # Build the odometry states
        x = 0
        y = 0
        yaw = self.X_states[0][2]
        for odometry in self.odometry_values:
            x += odometry[0] * math.cos(yaw) - odometry[1] * math.sin(yaw)
            y += odometry[1] * math.cos(yaw) + odometry[0] * math.sin(yaw)
            yaw += odometry[2] 
            self.odometry_states.append([x, y, yaw])

        # Plot the paths
        plt.plot([x[0] for x in self.X_states], [x[1] for x in self.X_states], label='EKF path')
        plt.plot([x[0] for x in self.odometry_states], [x[1] for x in self.odometry_states], label='Odometry path')
        plt.plot([x[0] for x in self.ground_truth_states], [x[1] for x in self.ground_truth_states], label='Ground Truth path')
        plt.scatter(self.X_states[0][0], self.X_states[0][1], color='red', label='Initial position')
        plt.xlim(-5.5, 5.5)
        plt.ylim(-0.5, 12.5)
        plt.legend(loc='upper right')
        plt.show()

if __name__ == "__main__":
    ap = Apolo()
    navigator = RobotNavigator('Marvin', ap)
    
    # balizas list: first one has target_id = 1; second one has target_id = 2; ...
    navigator.landmark_positions = [[1,1.5], [-1,5], [-1,7]]
    
    navigator.safety_margin = 0.3
    navigator.avoidance_offset = 0.5
    place_victim(ap=ap) # Place the victim in the environment randomly
    ap.resetOdometry('Marvin')
    ap.placeWheeledBase('Marvin', 0 , 0, 0, math.pi / 2)

    points = [
                (0, 1.5), (3, 1.5), (3, 5), (3, 1.5), (0, 1.5),
                (0, 5), (-2, 5), (-2, 1.5), (-2, 5), (0, 5), (0, 8), 
                (0, 7), (-2.5, 7), (-2.5, 9), (1, 11.5),(1.5, 11.5), 
                (1, 11.5), (-2.5, 9), (-2.5, 7), (0, 7), (0, 0)
            ]

    try:
        for point in points:
            if navigator.STOP_FLAG:
                break
            navigator.navigate_to(point[0], point[1], False)
            print(f"Reached point ({point[0]}, {point[1]})")
            time.sleep(0.2)
    except KeyboardInterrupt:
        print("Keyboard interrupt received. Stopping navigation and plotting results.")
        navigator.STOP_FLAG = True
    finally:
        navigator.plot()
