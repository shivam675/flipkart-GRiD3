from __future__ import division, print_function
from math import pi, sqrt, sin, cos, atan2, fabs, radians
from diff_drive.pose import Pose
from simple_pid import PID
import rospy

class GoalController:
    """Finds linear and angular velocities necessary to drive toward
    a goal pose.
    """

    def __init__(self):
        self.kP = 3
        self.kA = 8
        self.kB = -1.5
        self.max_linear_speed = 1E9
        self.min_linear_speed = 0
        self.max_angular_speed = 1E9
        self.min_angular_speed = 0
        self.max_linear_acceleration = 1E9
        self.max_angular_acceleration = 1E9
        self.linear_tolerance = 0.025 # 2.5cm
        self.angular_tolerance = 3/180*pi # 3 degrees
        self.forward_movement_only = False
        self.pid_left_right = PID(1, 0, 0, 0)  # PID left right
        self.pid_speed = PID(1, 0, 0, 0)  # PID Forward Backward
        self.max_forward_heading = 8
        self.mid_pid_speed = 0.11
        self.mid_pid_turn = 0.0813

        ##PID parameters

        self.Kp_turn = rospy.get_param('~Kp_turn', 0.1)
        self.Kd_turn = rospy.get_param('~Kd_turn', 0.01)
        self.Ki_turn = rospy.get_param('~Ki_turn', 0.001)

        self.Kp_speed = rospy.get_param('~Kp_speed', 0.15)
        self.Kd_speed = rospy.get_param('~Kp_speed', 0.1)
        self.Ki_speed = rospy.get_param('~Kp_speed', 0.01)





    def set_constants(self, kP, kA, kB):
        self.kP = kP
        self.kA = kA
        self.kB = kB

    def set_max_linear_speed(self, speed):
        self.max_linear_speed = speed

    def set_min_linear_speed(self, speed):
        self.min_linear_speed = speed

    def set_max_angular_speed(self, speed):
        self.max_angular_speed = speed

    def set_min_angular_speed(self, speed):
        self.min_angular_speed = speed

    def set_max_linear_acceleration(self, accel):
        self.max_linear_acceleration = accel

    def set_max_angular_acceleration(self, accel):
        self.max_angular_acceleration = accel

    def set_linear_tolerance(self, tolerance):
        self.linear_tolerance = tolerance

    def set_angular_tolerance(self, tolerance):
        self.angular_tolerance = tolerance

    def set_forward_movement_only(self, forward_only):
        self.forward_movement_only = forward_only

    def get_goal_distance(self, cur, goal):
        if goal is None:
            return 0
        diffX = cur.x - goal.x
        diffY = cur.y - goal.y
        return sqrt(diffX*diffX + diffY*diffY)

    def at_goal_position(self, cur, goal):
        if goal is None:
            return True
        d = self.get_goal_distance(cur, goal)
        ##dTh = abs(self.normalize_pi(cur.theta - goal.theta))
        ##return d < self.linear_tolerance and dTh < self.angular_tolerance
        return d< self.linear_tolerance

    def at_goal_orientation(self,cur,goal):
        if goal is None:
            return True
        dTh = abs(self.normalize_pi(cur.theta - goal.theta))
        return dTh < self.angular_tolerance       


    def get_velocity_PID(self,cur,goal,dT, goToPos):

        #print("current position x: {}, y:{} , theta: {}".format(cur.x,cur.y,cur.theta))
        
        desired = Pose()
        goal_heading = atan2(goal.y - cur.y, goal.x - cur.x)
        d = self.get_goal_distance(cur, goal)
        detected = int(rospy.get_param('bot_detected',1))

        if(detected and goToPos): 

            self.pid_speed.tunings = (self.Kp_speed, self.Kd_speed, self.Ki_speed)  # depends on the robot configuration
            if d > 5:  # I-term anti windup
                self.pid_speed.Ki = 0
            self.pid_speed.limitsUpper = (-self.max_linear_speed, self.max_linear_speed)  # depends on the robot configuration
            self.pid_speed.limitsLower = (-self.mid_pid_speed ,self.mid_pid_speed)
            self.pid_speed.sample_time = dT  # update every 0.01 seconds
            self.pid_speed.setpoint = 0

            if fabs(goal_heading - cur.theta) < radians(self.max_forward_heading):
                desired.xVel = -self.pid_speed(d)
            else :
                desired.xVel = 0


            self.pid_left_right.tunings = (self.Kp_turn, self.Kd_turn, self.Ki_turn)
            if(fabs(goal_heading - cur.theta)) > radians(5):
                self.pid_left_right.Ki = 0

            self.pid_left_right.limitsUpper = (-self.max_angular_speed,self.max_angular_speed)
            if (fabs(desired.xVel>self.mid_pid_speed)):
                self.pid_left_right.limitsLower = (0,0)
            else:
                self.pid_left_right.limitsLower = (-self.mid_pid_turn,self.mid_pid_turn)

            self.pid_left_right.sample_time = dT
            self.pid_left_right.setpoint = goal_heading
            desired.thetaVel = self.pid_left_right(cur.theta)

        elif(detected and not goToPos):
            self.pid_left_right.tunings = (self.Kp_turn, self.Kd_turn, self.Ki_turn)
            self.pid_left_right.limitsUpper = (-self.max_angular_speed,self.max_angular_speed)
            self.pid_left_right.limitsLower = (-self.mid_pid_turn,self.mid_pid_turn)
            if(fabs(goal_heading - cur.theta)) > radians(5):
                self.pid_left_right.Ki = 0

            self.pid_left_right.sample_time = dT
            self.pid_left_right.setpoint = goal.theta
            desired.thetaVel = self.pid_left_right(cur.theta)  


        else:
            desired.xVel = 0
            desired.thetaVel = 0            




        ##print("xvel: {}, theta_vel {}".format(desired.xVel,desired.thetaVel))
        return desired








    def get_velocity(self, cur, goal, dT):
        desired = Pose()

        goal_heading = atan2(goal.y - cur.y, goal.x - cur.x)
        a = -cur.theta + goal_heading

        # In Automomous Mobile Robots, they assume theta_G=0. So for
        # the error in heading, we have to adjust theta based on the
        # (possibly non-zero) goal theta.
        theta = self.normalize_pi(cur.theta - goal.theta)
        b = -theta - a

        # rospy.loginfo('cur=%f goal=%f a=%f b=%f', cur.theta, goal_heading,
        #               a, b)

        d = self.get_goal_distance(cur, goal)
        if self.forward_movement_only:
            direction = 1
            a = self.normalize_pi(a)
            b = self.normalize_pi(b)
        else:
            direction = self.sign(cos(a))
            a = self.normalize_half_pi(a)
            b = self.normalize_half_pi(b)

        # rospy.loginfo('After normalization, a=%f b=%f', a, b)

        if abs(d) < self.linear_tolerance:
            desired.xVel = 0
            desired.thetaVel = self.kB * theta
        else:
            desired.xVel = self.kP * d * direction
            desired.thetaVel = self.kA*a + self.kB*b

        # Adjust velocities if X velocity is too high.
        if abs(desired.xVel) > self.max_linear_speed:
            ratio = self.max_linear_speed / abs(desired.xVel)
            desired.xVel *= ratio
            desired.thetaVel *= ratio

        # Adjust velocities if turning velocity too high.
        if abs(desired.thetaVel) > self.max_angular_speed:
            ratio = self.max_angular_speed / abs(desired.thetaVel)
            desired.xVel *= ratio
            desired.thetaVel *= ratio

        # TBD: Adjust velocities if linear or angular acceleration
        # too high.

        # Adjust velocities if too low, so robot does not stall.
        if abs(desired.xVel) > 0 and abs(desired.xVel) < self.min_linear_speed:
            ratio = self.min_linear_speed / abs(desired.xVel)
            desired.xVel *= ratio
            desired.thetaVel *= ratio
        elif desired.xVel==0 and abs(desired.thetaVel) < self.min_angular_speed:
            ratio = self.min_angular_speed / abs(desired.thetaVel)
            desired.xVel *= ratio
            desired.thetaVel *= ratio

        return desired

    def normalize_half_pi(self, alpha):
        alpha = self.normalize_pi(alpha)
        if alpha > pi/2:
            return alpha - pi
        elif alpha < -pi/2:
            return alpha + pi
        else:
            return alpha

    def normalize_pi(self, alpha):
        while alpha > pi:
            alpha -= 2*pi
        while alpha < -pi:
            alpha += 2*pi
        return alpha

    def sign(self, x):
        if x >= 0:
            return 1
        else:
            return -1
