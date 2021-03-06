= diff_drive -- Differential-Drive Controller
:imagesdir: ./images

This package implements ROS nodes to control and monitor a differential-drive robot. Modified for the flipkart Grid challenge. Original package from:
https://github.com/merose/diff_drive

-Idris Salim

The package
is intended as a lighter-weight solution than the ROS controller framework, albeit with lower
performance since it is written in Python. If you need tight, real-time control, you may want
to look at link:http://wiki.ros.org/ros_controllers[ros_controllers],
a C++ package which includes a differential-drive controller that is integrated with
link:http://wiki.ros.org/ros_control[ros_control] and
link:http://wiki.ros.org/controller_manager[controller_manager]. Those controllers are designed
to integrate with hardware in the same process, rather than using topics. Instead, this package
expects to publish the desired motor speeds using standard ROS messages.

== Supplied Nodes

* `diff_drive_controller` -- Converts from twist to wheel velocities for motors.Not used for pure aruco implementation (Use for encoders)

* `diff_drive_odometry` -- Publishes odometry from wheel encoder data. For AruCo mode, uses Tfs published by fiducials package.

* `diff_drive_go_to_goal` -- Moves the robot to a goal position. Added PID controller option that uses AruCo Tfs.

* `diff_drive_mock_robot` -- Implements a mock differential drive robot, for testing.

The nodes in this package are designed with these considerations:

== Demo

Run the AruCo/ Tagslam package. You can also run Opencv Aruco and convert the image co-ordinates to transforms.

Run the diff_drive_odometry node with arguments: 

1)No. Of bots
2)Origin tag ID (fiducial/tagslam)
3)Robot tag Ids (fiducial/tagslam)



example: ~$ rosrun diff_drive diff_drive_odometry 2 'fiducial_102' 'fiducial_107' 'fiducial_106'

(2 robots, origin tagID and robot IDs)








=== 1. diff_drive_odometry

Listens to AruCo transforms and publishes odometry
==== Published Topics

`~odom` -- (nav_msgs/Odometry)::
The robot odometry -- the current robot pose.

==== Subscribed Topics
/tf

==== Parameters


`~rate` (double, default 10.0)::
The rate at which the `tf` and `odom` topics are published (Hz).

`~timeout` (double, default 0.2)::
The amount of time to continue publishing desired wheel rates after receiving a twist message (seconds).
If set to zero, wheel velocities will be sent only when a new twist message is received.

`~base_frame_id` (string, default: "base_link")::
The name of the base frame of the robot. 

`~odom_frame_id` (string, default: "odom")::
The name of the odometry reference frame. 

=== 3. diff_drive_go_to_goal

Listens for new goal poses and computes velocities needed to achieve the goal.

==== Published Topics

`~distance_to_goal` (std_msgs/Float32)::
Distance to the goal position (meters).

`~cmd_vel` (geometry_msgs/Twist)::
Desired linear and angular velocity to move toward the goal pose.

==== Subscribed Topics

`~goal` (geometry_msgs/Pose)::
Desired goal pose.

==== Parameters

`~rate` (float, default: 10)::
Rate at which to publish desired velocities (Hz).

`~goal_linear_tolerance` (float, default: 0.1)::
The distance from the goal at which the robot is assumed to have accomplished the goal position (meters).

`~goal_angular_tolerance` (float, default: 0.087)::
The difference between robot angle and goal pose angle at which the robot is assumed to have
accomplished the goal attitude (radians). Default value is approximately 5 degrees.

`~max_linear_velocity` (float, default: 0.2)::
The maximum linear velocity toward the goal (meters/second).

`~max_angular_velocity` (float, default: 1.5)::
The maximum angular velocity (radians/second).

`~max_linear_acceleration` (float, default: 4.0)::
The maximum linear acceleration (meters/second^2).

`~forwardMovementOnly` (boolean, default: true)::
If true, only forward movement is allowed to achieve the goal position.
If false, the robot will move backward to the goal if that is the most
direct path.



==for continous path function==

`~Kp` (float, default: 3.0)::
Linear distance proportionality constant. Higher values make the robot accelerate more quickly toward the goal and decelerate less quickly.

`~Ka` (float: default: 8.0)::
Proportionality constant for angle to goal position. Higher values make the robot turn more quickly toward the goal.

`~Kb` (float: default: -1.5)::
Proportionality constant for angle to goal pose direction. Higher values make the robot turn more quickly toward the goal pose direction. This value should be negative, per _Autonomous Mobile Robots_.

The control law for determining the linear and angular velocity to move toward the goal works as follows. Let _d_ be the distance to the goal. Let _a_ be the angle between the robot heading and the goal position, where left is positive. Let _b_ be the angle between the goal direction and the final pose angle, where left is positive. Then the robot linear and angular velocities are calculated like this:

    v = Kp * d
    w = Ka*a + Kb*b

See _Autonomous Mobile Robots, Second Edition_ by Siegwart et. al., section 3.6.2.4. In this code, when the robot
is near enough to the goal, _v_ is set to zero, and _w_ is simply _Kb*b_.

To ensure convergence toward the goal, _K~p~_ and _K~a~_ must be positive, _K~b~_ must be negative, and _K~a~_
must be greater than _K~p~_. To ensure robust convergence, so that the robot never changes direction,
_K~a~_ - 5/3*_K~b~_ - 2/pi*_K~p~_ must be greater than zero.


==for PID method==
parameters are defined in file goal_controller.py


launch example:

~$rosrun diff_drive diff_drive_go_to_goal sim robot1

==parameters==

sim: for simulation mode
irl: for arena mode

robotID: robot ID of the running bot.

