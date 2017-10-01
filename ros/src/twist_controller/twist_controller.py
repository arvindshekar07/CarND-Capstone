import pid
import tf
from math import sqrt, cos, sin
import numpy as np
import rospy

GAS_DENSITY = 2.858
ONE_MPH = 0.44704


class Controller(object):
    def __init__(self, vehicle_mass, wheel_radius, accel_limit, decel_limit, max_steer_angle):
        self.vehicle_mass = vehicle_mass
        self.wheel_radius = wheel_radius
        self.accel_limit = accel_limit
        self.decel_limit = decel_limit
        self.max_steer_angle = max_steer_angle
        self.steer_pid = pid.PID(kp=0.15, ki=0, kd=0.99, mn=-max_steer_angle, mx=max_steer_angle)
        self.timestamp = rospy.get_time()		

    def control(self, target_velocity, current_velocity,  pose, waypoints, twist):
        # TODO: Change the arg, kwarg list to suit your needs
        # Return throttle, brake, steer
        cte = self.calculate_cte(pose, waypoints)
        steer = self.steer_pid.step(cte, 0.1)
        self.steer_pid.reset()		
 
        acceleration = target_velocity - current_velocity / 0.5
        rospy.loginfo("Current Velocity: " + str(current_velocity) + " Target Velocity: " + str(target_velocity) + " Expected Acceleration: " + str(acceleration))
        if acceleration > 0:
          acceleration = min(self.accel_limit, acceleration)
        else:
            acceleration = max(self.decel_limit, acceleration)
        torque = self.vehicle_mass * acceleration * self.wheel_radius
        throttle, brake = None, None
        if torque > 0:
            throttle, brake = min(1.0, torque / 200.0), 0.0
        else:
            throttle, brake = 0.0, min(abs(torque), 20000.0)		
        return throttle, brake, steer

    def get_rpy_from_quaternion(self, pose):
        quaternion = (pose.orientation.x,pose.orientation.y,pose.orientation.z,pose.orientation.w)
        return tf.transformations.euler_from_quaternion(quaternion)
 
    def calculate_cte(self, pose, waypoints):
        x_vals, y_vals = self.transform_waypoints(pose, waypoints)
        coefficients = np.polyfit(x_vals, y_vals, 5)
        cte = np.polyval(coefficients, 5.0)

        return cte		
	
    def transform_waypoints(self, pose, waypoints):
       x_vals = []
       y_vals = []

       _ig1, _ig2, yaw = self.get_rpy_from_quaternion(pose)
       originX = pose.position.x
       originY = pose.position.y

       for waypoint in waypoints:

           shift_x = waypoint.pose.pose.position.x - originX
           shift_y = waypoint.pose.pose.position.y - originY

           x = shift_x * cos(0 - yaw) - shift_y * sin(0 - yaw)
           y = shift_x * sin(0 - yaw) + shift_y * cos(0 - yaw)

           x_vals.append(x)
           y_vals.append(y)

       return x_vals, y_vals	
		