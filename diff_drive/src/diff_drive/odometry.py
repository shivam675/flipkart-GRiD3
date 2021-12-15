from __future__ import division
from math import pi, sin, cos
from diff_drive.encoder import Encoder
from diff_drive.pose import Pose
import rospy
import tf

class Odometry:
    """Keeps track of the current position and velocity of a
    robot using differential drive.
    """

    def __init__(self, orig_tag_id,robot_id):  ##takes in all the transforms and publishes odometry
        #self.leftEncoder = Encoder()
        #self.rightEncoder = Encoder()
        self.pose = Pose()
        self.lastTime = 0
        self.robot_object_id = robot_id
        self.orig_tag_id = orig_tag_id



    #def setWheelSeparation(self, separation):
        #self.wheelSeparation = separation

   # def setTicksPerMeter(self, ticks):
        #self.ticksPerMeter = ticks
        
    #def setEncoderRange(self, low, high):
        #self.leftEncoder.setRange(low, high)
        #self.rightEncoder.setRange(low, high)

    def setTime(self, newTime):
        self.lastTime = newTime
        
    #def updateLeftWheel(self, newCount):
        #self.leftEncoder.update(newCount)

    #def updateRightWheel(self, newCount):
        #self.rightEncoder.update(newCount)

    def updatePose(self, newTime):
        listener = tf.TransformListener()
        """Updates the pose based on the accumulated encoder ticks
        of the two wheels. See https://chess.eecs.berkeley.edu/eecs149/documentation/differentialDrive.pdf
        for details.

        v2 
        added pose update using aruco/apriltag transform
        """

        print("updated odom")
        trans= [0,0,0]
        rot=[0,0,0]
        try: 
            #(trans,rot) = listener.lookupTransform(self.orig_tag_id, self.robot_object_id, rospy.Time(0))
            listener.waitForTransform(self.orig_tag_id, self.robot_object_id, rospy.Time(0), rospy.Duration(3))
            (trans,rot) = listener.lookupTransform(self.orig_tag_id,self.robot_object_id , rospy.Time(0))

        except (tf2_ros.LookupException):
            print("tf lookupException")
            rospy.set_param('bot_detected', 0)
            pass
        except (tf2_ros.ConnectivityException):
            print("tf connectivity exception")
            rospy.set_param('bot_detected', 0)
            pass
        except(tf2_ros.ExtrapolationException):
            print("tf extrapolation exception")
            rospy.set_param('bot_detected', 0)
            pass

        ##remove all components except yaw

        orient_euler = tf.transformations.euler_from_quaternion(rot)
        orient_only_yaw = tf.transformations.quaternion_from_euler(0,0,orient_euler[2])


        #leftTravel = self.leftEncoder.getDelta() / self.ticksPerMeter
        #rightTravel = self.rightEncoder.getDelta() / self.ticksPerMeter
        #deltaTime = newTime - self.lastTime

        #deltaTravel = (rightTravel + leftTravel) / 2
        #deltaTheta = (rightTravel - leftTravel) / self.wheelSeparation

        #if rightTravel == leftTravel:
            #deltaX = leftTravel*cos(self.pose.theta)
            #deltaY = leftTravel*sin(self.pose.theta)
        #else:
            #radius = deltaTravel / deltaTheta

            # Find the instantaneous center of curvature (ICC).
            #iccX = self.pose.x - radius*sin(self.pose.theta)
            #iccY = self.pose.y + radius*cos(self.pose.theta)

           # deltaX = cos(deltaTheta)*(self.pose.x - iccX) \
                #- sin(deltaTheta)*(self.pose.y - iccY) \
                #+ iccX - self.pose.x

            #deltaY = sin(deltaTheta)*(self.pose.x - iccX) \
                #+ cos(deltaTheta)*(self.pose.y - iccY) \
                #+ iccY - self.pose.y

        self.pose.x = trans[0]                         ##x and y pose wrt map
        self.pose.y = trans[1]

        self.pose.theta = orient_euler[2]    ##yaw component                                         ##(self.pose.theta + deltaTheta) % (2*pi)
        self.pose.xVel = 0.
        self.pose.yVel = 0
        self.pose.thetaVel = 0.

        self.lastTime = newTime

    def getPose(self):
        return self.pose;

    def setPose(self, newPose):
        self.pose = newPose
