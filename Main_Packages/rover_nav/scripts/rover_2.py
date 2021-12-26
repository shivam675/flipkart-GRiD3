#!/usr/bin/python3
# from threading import TIMEOUT_MAX
import rospy
from geometry_msgs.msg import PoseStamped
import tf
import sys
# from threading import Thread
from rover_nav.srv import get_task, path_service
from nav_msgs.srv import GetPlan
import rospkg
import yaml

path = rospkg.RosPack()
path = path.get_path('rover_nav')


class rover_two:
    def __init__(self, name='rover_2') -> None:
        self.rover_name = name
        self.package_id = None
        self.package_name = None
        # self.chute_name = None
        self.chute_name = 'mumbai'
        self.dock_station_name = None
        self.odom_x = None
        self.odom_y = None
        self.return_status = 'Done'
        self.location_dict = {}

    def service_server(self):
        rospy.Service('/schedule/rover_two',get_task ,self.get_task_callback)
        rospy.spin()
    
    def get_task_callback(self, req):
        self.odom_pose = PoseStamped()
        self.start_pose = PoseStamped()
        self.goal_pose = PoseStamped()
        self.get_odom()


        self.package_id = req.package_id
        self.package_name = req.package_name
        self.chute_name = req.chute_name
        self.dock_station_name = req.dock_station_name
        self.path_one = None
        self.path_two = None

        self.process_request()
        # return self.return_status
    
    def read_locations(self):
        with open(path + '/info/locations.yaml') as file:
            data = yaml.load(file, yaml.FullLoader)
            # print(data['chutes']['mumbai']['px'])
            self.location_dict = data

    def process_request(self):

        req_1 = GetPlan()
        req_2 = GetPlan()

        self.odom_pose.header.frame_id = "map"
        self.start_pose.header.frame_id = "map"
        self.goal_pose.header.frame_id = "map"

        self.odom_pose.header.stamp = rospy.Time(0)
        self.start_pose.header.stamp = rospy.Time(0)
        self.goal_pose.header.stamp = rospy.Time(0)

        self.odom_pose.header.seq = 0
        self.start_pose.header.seq = 0
        self.goal_pose.header.seq = 0

        self.odom_pose.pose.position.x = self.odom_x
        self.odom_pose.pose.position.y = self.odom_y

        self.start_pose.pose.position.x = self.location_dict['chutes'][self.chute_name]['px']
        self.start_pose.pose.position.y = self.location_dict['chutes'][self.chute_name]['py']

        self.goal_pose.pose.position.x = self.location_dict['docks'][self.dock_station_name]['px']
        self.goal_pose.pose.position.y = self.location_dict['docks'][self.dock_station_name]['px']

        ###################################
        req_1.start = self.odom_pose
        req_1.goal = self.start_pose
        req_1.tolerance = 0.1
        ###################################
        req_2.start = self.start_pose
        req_2.goal = self.goal_pose
        req_1.tolerance = 0.1
        ###################################

        self.get_plan = rospy.ServiceProxy('/move_base/make_plan', GetPlan)
        self.path_one = self.get_plan(req_1.start, req_1.goal, req_1.tolerance)
        self.path_two = self.get_plan(req_2.start, req_2.goal, req_2.tolerance)
    
    def get_odom(self):
        t = tf.TransformListener()
        now = rospy.Time.now()
        (trans,rot) = t.lookupTransform('robot2/base_link', 'map', time=now)
        self.odom_x, self.odom_y, _ = trans


    def send_paths(self):
        rospy.set_param('robot1/is_running', True)
        sendpath = rospy.ServiceProxy('robot2/exec_path', path_service)
        result_1 = sendpath(path = self.path_one)
        rospy.loginfo('sending path to the rover_script : ' + str(result_1))
        result_2 = sendpath(path = self.path_two)
        rospy.loginfo('sending path to the rover_script : ' + str(result_2))


if __name__ == '__main__':
    rospy.init_node('rover_2_node', anonymous=True)
    g = rover_two()
    g.read_locations()
    g.service_server()
