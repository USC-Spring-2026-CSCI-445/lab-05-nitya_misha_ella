#!/usr/bin/env python3
from typing import Optional, Tuple
from argparse import ArgumentParser
import math
import queue

import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion


# PID controller class for both linear and angular control
class PIDController:
    """
    Generates control action taking into account instantaneous error (proportional action),
    accumulated error (integral action) and rate of change of error (derivative action).
    """

    def __init__(self, kP, kI, kD, kS, u_min, u_max):
        assert u_min < u_max, "u_min should be less than u_max"
        # Initialize PID variables here
        ######### Your code starts here #########
        self.kP = kP
        self.kI = kI
        self.kD = kD
        self.kS = kS
        self.u_min = u_min
        self.u_max = u_max
        #more?
        self.t_prev = 0.0
        self.e_prev = 0.0
        self.integral = 0.0

        ######### Your code ends here #########

    def control(self, err, t):
        # computer PID control action here
        ######### Your code starts here #########
        dt = t - self.t_prev
        
        #compute derivative
        if self.t_prev == 0.0 or dt <= 0:
            derivative = 0.0
        else:
            derivative = (err - self.e_prev)/dt

        
        #compute integral
        if dt > 0:
            self.integral += err * dt

        #clamp integral
        self.integral = max(-self.kS, min(self.integral, self.kS))

        #compute u 
        u = self.kP*err + self.kI*self.integral + self.kD*derivative
        u = max(self.u_min, min(u, self.u_max))

        self.t_prev = t
        self.e_prev = err

        return u
        ######### Your code ends here #########


# PD controller class
class PDController:
    """
    Generates control action taking into account instantaneous error (proportional action)
    and rate of change of error (derivative action).
    """

    def __init__(self, kP, kD, kS, u_min, u_max):
        assert u_min < u_max, "u_min should be less than u_max"
        # Initialize PD variables here
        ######### Your code starts here #########
        self.kP = kP
        #self.kI = kI
        self.kD = kD
        self.kS = kS
        self.u_min = u_min
        self.u_max = u_max
        self.t_prev = 0.0
        self.e_prev = 0.0
        #more?
        
        ######### Your code ends here #########

    def control(self, err, t):
        dt = t - self.t_prev
        # Compute PD control action here
        ######### Your code starts here #########
        #compute derivative
        if self.t_prev == 0.0 or dt <= 0:
            derivative = 0.0
        else:
            derivative = (err - self.e_prev)/dt

        
        u = self.kP * err + self.kD * derivative
        u = max(self.u_min, min(u, self.u_max))
        self.t_prev = t
        self.e_prev = err
        return u

        ######### Your code ends here #########


# Class for controlling the robot to reach a goal position
class GoalPositionController:
    def __init__(self, goal_position):
        rospy.init_node("goal_position_controller", anonymous=True)

        # Subscriber to the robot's current position (assuming you have Odometry data)
        self.odom_sub = rospy.Subscriber("/odom", Odometry, self.odom_callback)

        # Publisher for robot's velocity command
        self.vel_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)

        self.goal_position = goal_position
        self.current_position = None

        # TODO: define PID controllers for linear and angular velocities
        ######### Your code starts here #########
        kP = 2.0
        kD = 0.03
        kI = 0.01
        kS = 0.4
        u_min = -1.5
        u_max = 1.5

        self.angular_controller = PIDController(kP, kI, kD, kS, u_min, u_max)
       
        self.v0 = 0.2 #base forward velocity
        
        ######### Your code ends here #########

    def odom_callback(self, msg):
        # Extracting current position from Odometry message
        pose = msg.pose.pose
        orientation = pose.orientation
        _, _, theta = euler_from_quaternion([orientation.x, orientation.y, orientation.z, orientation.w])

        self.current_position = {"x": pose.position.x, "y": pose.position.y, "theta": theta}

    def calculate_error(self) -> Optional[Tuple]:
        if self.current_position is None:
            return None

        # Calculate error in position and orientation
        ######### Your code starts here #########
        
        dx = self.goal_position["x"] - self.current_position["x"]
        dy = self.goal_position["y"] - self.current_position["y"]
        
        #dist = sqrt(dx^2 + dy^2)
        distance_error = math.sqrt(dx**2+dy**2)
        
        #angle = arctan(dy/dx)
        goal_angle = math.atan2(dy, dx)
        current_angle = self.current_position["theta"]

        angle_error = goal_angle - current_angle
        ######### Your code ends here #########

        # Ensure angle error is within -pi to pi range
        if angle_error > math.pi:
            angle_error -= 2 * math.pi
        elif angle_error < -math.pi:
            angle_error += 2 * math.pi

        return distance_error, angle_error

    def control_robot(self):
        rate = rospy.Rate(10)  # 10 Hz
        ctrl_msg = Twist()
        while not rospy.is_shutdown():
            error = self.calculate_error()

            if error is None:
                continue
            distance_error, angle_error = error

            # Calculate control commands using linear and angular PID controllers and stop if close enough to goal
            ######### Your code starts here #########
            #ctrl_msg.linear.x = forward, backward
            #ctrl_msg.angular.z = rotation
            
                        
            t = rospy.get_time()
            
            
            #if error is small
            if abs(distance_error) < 0.05:
                ctrl_msg.linear.x = 0.0
                ctrl_msg.angular.z = 0.0
            else:
                omega = self.angular_controller.control(angle_error, t) #calls the control method in the PID controller class that internally computes ω = kP*error + kI*integral + kD*derivative
                ctrl_msg.angular.z = omega
                ctrl_msg.linear.x = self.v0
            
            #publish
            self.vel_pub.publish(ctrl_msg)

            ######### Your code ends here #########

            rate.sleep()


# Class for controlling the robot to reach a goal position
class GoalAngleController:
    def __init__(self, goal_angle):
        rospy.init_node("goal_angle_controller", anonymous=True)

        # Subscriber to the robot's current position (assuming you have Odometry data)
        self.odom_sub = rospy.Subscriber("/odom", Odometry, self.odom_callback)

        # Publisher for robot's velocity command
        self.vel_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)

        self.goal_angle = goal_angle
        self.current_position = None

        # define PID controller angular velocity
        ######### Your code starts here #########
        
        
         # Reduced gains for real hardware (less noisy than sim)
        kP = 0.8
        kD = 0.03
        kI = 0.01
        kS = 0.4
        u_min = -1.5
        u_max = 1.5

        self.angular_controller = PIDController(kP, kI, kD, kS, u_min, u_max)
        ######### Your code ends here #########

    def odom_callback(self, msg):
        # Extracting current position from Odometry message
        pose = msg.pose.pose
        orientation = pose.orientation
        _, _, theta = euler_from_quaternion([orientation.x, orientation.y, orientation.z, orientation.w])

        self.current_position = {"x": pose.position.x, "y": pose.position.y, "theta": theta}

    def calculate_error(self) -> Optional[float]:
        if self.current_position is None:
            return None

        # Calculate error in orientation
        ######### Your code starts here #########
        current_angle = self.current_position["theta"] #current position encodes x, y, theta
        angle_error = self.goal_angle - current_angle
        ######### Your code ends here #########

        # Ensure angle error is within -pi to pi range
        if angle_error > math.pi:
            angle_error -= 2 * math.pi
        elif angle_error < -math.pi:
            angle_error += 2 * math.pi
        return angle_error

    def control_robot(self):
        rate = rospy.Rate(10)  # 10 Hz
        ctrl_msg = Twist()
        while not rospy.is_shutdown():
            angle_error = self.calculate_error()

            if angle_error is None:
                continue

            # TODO: Calculate control commands using angular PID controller and stop if close enough to goal
            ######### Your code starts here #########
            #ctrl_msg.linear.x = forward, backward
            #ctrl_msg.angular.z = rotation
            
            #no forward or backward anytime
            ctrl_msg.linear.x = 0.0
            
            t = rospy.get_time()
            
            
            #if error is small
            if abs(angle_error) < 0.01:
                ctrl_msg.angular.z = 0.0
            else:
                omega = self.angular_controller.control(angle_error, t) #calls the control method in the PID controller class that internally computes ω = kP*error + kI*integral + kD*derivative

                ctrl_msg.angular.z = omega
            
            #publish
            self.vel_pub.publish(ctrl_msg)
            
            ######### Your code ends here #########

            rate.sleep()


if __name__ == "__main__":

    parser = ArgumentParser()
    parser.add_argument("--goal_x", type=float, help="Goal x-coordinate")
    parser.add_argument("--goal_y", type=float, help="Goal y-coordinate")
    parser.add_argument("--goal_angle", type=float, help="Goal orientation in radians")
    parser.add_argument("--mode", type=str, required=True, help="Mode of operation: 'position' or 'angle'")
    args = parser.parse_args()
    assert args.mode in {"position", "angle"}

    if args.mode == "position":
        assert isinstance(args.goal_x, float) and isinstance(args.goal_y, float)
        goal_pos = {"x": args.goal_x, "y": args.goal_y}
        controller = GoalPositionController(goal_pos)
    else:
        assert isinstance(args.goal_angle, float), f"expected float for --goal_angle, got {type(args.goal_angle)}"
        assert -math.pi <= args.goal_angle <= math.pi, f"--goal_angle should be in range [-pi, pi]"
        controller = GoalAngleController(args.goal_angle)

    try:
        controller.control_robot()
    except rospy.ROSInterruptException:
        pass
