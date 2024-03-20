import numpy as np

class ExtendedKalmanFilter:
    def __init__(self, initial_state, initial_covariance, motion_noise, measurement_noise):
        self.state = initial_state
        self.covariance = initial_covariance
        self.motion_noise = motion_noise
        self.measurement_noise = measurement_noise

    def predict(self, dt, linear_acceleration, angular_velocity, gyro_measurement):
        # Predicted state using motion model
        self.state = self.motion_model(dt, linear_acceleration, angular_velocity, gyro_measurement)

        # Jacobian of the motion model
        F = self.motion_model_jacobian(dt, linear_acceleration, angular_velocity, gyro_measurement)

        # Predicted covariance
        self.covariance = np.dot(np.dot(F, self.covariance), F.T) + self.motion_noise

    def update_landmark(self, landmark_measurement):
        # Compute measurement Jacobian
        H = self.compute_measurement_jacobian(landmark_measurement)

        # Kalman gain
        K = np.dot(np.dot(self.covariance, H.T), np.linalg.inv(np.dot(np.dot(H, self.covariance), H.T) + self.measurement_noise))

        # Update state estimate
        innovation = landmark_measurement - self.compute_measurement_prediction()
        self.state = self.state + np.dot(K, innovation)

        # Update covariance
        self.covariance = np.dot((np.eye(len(self.state)) - np.dot(K, H)), self.covariance)

    def update_odometry(self, odometry_measurement):
        # Measurement Jacobian for odometry update
        H = np.eye(len(self.state))

        # Kalman gain
        K = np.dot(np.dot(self.covariance, H.T), np.linalg.inv(np.dot(np.dot(H, self.covariance), H.T) + self.measurement_noise))

        # Update state estimate
        innovation = odometry_measurement - self.state
        self.state = self.state + np.dot(K, innovation)

        # Update covariance
        self.covariance = np.dot((np.eye(len(self.state)) - np.dot(K, H)), self.covariance)

    def motion_model(self, dt, linear_acceleration, angular_velocity, gyro_measurement):
        # Simple motion model: constant velocity
        state_transition_matrix = np.array([[1, 0, 0, dt, 0],
                                            [0, 1, 0, 0, dt],
                                            [0, 0, 1, 0, 0],
                                            [0, 0, 0, 1, 0],
                                            [0, 0, 0, 0, 1]])

        # State update with IMU measurements
        self.state[3] += dt * linear_acceleration[0]  # Update velocity in x-direction
        self.state[4] += dt * linear_acceleration[1]  # Update velocity in y-direction
        self.state[2] += dt * angular_velocity        # Update angular velocity
        self.state[2] += dt * gyro_measurement        # Update angular position (gyro measurement)

        # Apply motion model to predict next state
        return np.dot(state_transition_matrix, self.state)

    def motion_model_jacobian(self, dt, linear_acceleration, angular_velocity, gyro_measurement):
        # Jacobian of the motion model
        return np.array([[1, 0, 0, dt, 0],
                         [0, 1, 0, 0, dt],
                         [0, 0, 1, 0, dt],
                         [0, 0, 0, 1, 0],
                         [0, 0, 0, 0, 1]])
    

    def compute_measurement_prediction(self):
        # Extract relevant components from the state vector
        x, y, theta = self.state[0], self.state[1], self.state[2]
    
        # Suppose the Limelight camera is mounted on the robot, facing forward
        # Assuming the landmark position is given in the robot's coordinate frame
        # You may need to transform this into the camera's coordinate frame

        # Example: Compute expected landmark position relative to camera
        # Assuming landmark coordinates are given in meters
        landmark_distance = 5.0  # Example distance to landmark
        landmark_angle = np.pi / 4  # Example angle to landmark (in radians)
    
        # Convert landmark position relative to the robot to global coordinates
        landmark_x_global = x + landmark_distance * np.cos(theta + landmark_angle)
        landmark_y_global = y + landmark_distance * np.sin(theta + landmark_angle)
    
        # For simplicity, let's assume the expected measurement is the landmark's global position
        expected_measurement = np.array([landmark_x_global, landmark_y_global])
    
        return expected_measurement


    def compute_measurement_jacobian(self, landmark_measurement):
        # Extract relevant components from the state vector
        x, y, theta = self.state[0], self.state[1], self.state[2]

        # Extract components of the landmark measurement
        landmark_x, landmark_y = landmark_measurement

        # Compute distance and bearing from robot to landmark
        delta_x = landmark_x - x
        delta_y = landmark_y - y
        delta = np.sqrt(delta_x**2 + delta_y**2)
        bearing = np.arctan2(delta_y, delta_x) - theta

        # Compute Jacobian matrix
        H = np.zeros((2, len(self.state)))  # Assuming 2D landmark measurement

        # Partial derivatives of measurement model with respect to state variables
        H[0, 0] = -delta_x / delta
        H[0, 1] = -delta_y / delta
        H[0, 2] = (delta_x * np.sin(theta) - delta_y * np.cos(theta)) / delta
        H[1, 0] = delta_y / delta**2
        H[1, 1] = -delta_x / delta**2
        H[1, 2] = (-delta_x * np.cos(theta) - delta_y * np.sin(theta)) / delta**2

        return H


# Example usage
initial_state = np.array([0, 0, 0, 0, 0])  # Initial state: [x, y, theta, vx, vy]
initial_covariance = np.eye(5)  # Initial covariance matrix
motion_noise = np.eye(5) * 0.01  # Motion noise covariance
measurement_noise = np.eye(2) * 0.1  # Measurement noise covariance

ekf = ExtendedKalmanFilter(initial_state, initial_covariance, motion_noise, measurement_noise)

# Main loop
dt = 0.1  # Time step
linear_acceleration = np.array([0.1, 0.1])  # Linear acceleration: [ax, ay]
angular_velocity = 0.01    # Angular velocity
gyro_measurement = 0.01     # Gyro measurement

for _ in range(100):
    ekf.predict(dt, linear_acceleration, angular_velocity, gyro_measurement)

    # Suppose we have a landmark measurement [range, bearing]
    landmark_measurement = np.array([10.0, np.pi/4])
    ekf.update_landmark(landmark_measurement)

    # Update using odometry measurement from SwerveDrive4Odometry
    odometry_measurement = np.array([1.0, 1.0, 0.0, 0.1, 0.1])  # Odometry measurement: [x, y, theta, vx, vy]
    ekf.update_odometry(odometry_measurement)

    print("State:", ekf.state)
