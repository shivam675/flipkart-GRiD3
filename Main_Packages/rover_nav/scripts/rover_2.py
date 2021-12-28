#!/usr/bin/python3
from rospkg.rospack import RosStack
import rospy
from geometry_msgs.msg import PoseStamped
import tf
import sys
# from threading import Thread
from nav_msgs.msg import Odometry, Path
from rover_nav.srv import get_task, path_service
from nav_msgs.srv import GetPlan
import rospkg
import yaml
from std_msgs.msg import Bool

path = rospkg.RosPack()
path = path.get_path('rover_nav')


class rover_one:
    def __init__(self, name='rover_2'):
        self.rover_name = name
        self.package_id = None
        self.package_name = None
        # self.chute_name = None
        self.chute_name = 'mumbai'
        self.dock_station_name = None
        self.odom_x = None
        self.odom_y = None
        self.return_status = 'Done'
        self.location_induct_one = {}
        self.location_induct_two = {}
        self.pub_locations = rospy.Publisher('location_publisher', PoseStamped, queue_size=10)
        self.sendpath = rospy.Publisher('robot2/exec_path', Path, queue_size=2)
        self.dock_dict = {'dock_one': 'induct_station_1', 'dock_two': 'induct_station_2'}

    def service_server(self):
        print('Rospy server started')
        rospy.Service('/schedule/rover_two',get_task ,self.get_task_callback)
        rospy.spin()

    def get_task_callback(self, req):
        # rospy.loginfo('Request Received !')

        rospy.set_param('robot2/can_run', False)
        print('requesting odom')
        self.get_odom()


        self.package_id = req.package_id
        self.package_name = req.package_name
        self.chute_name = req.chute_name
        self.dock_station_name = req.dock_station_name
        
        # rospy.set_param(self.dock_dict[self.dock_station_name], False)

        print('request_recieved')
        self.process_request()
        # 
        return self.return_status
    
    def read_locations(self):
        with open(path + '/info/location_for_induct_one.yaml') as file:
            data = yaml.load(file, yaml.FullLoader)
            # print(data['chutes']['mumbai']['px'])
            self.location_induct_one = data
        
        with open(path + '/info/location_for_induct_two.yaml') as file:
            data = yaml.load(file, yaml.FullLoader)
            # print(data['chutes']['mumbai']['px'])
            self.location_induct_two = data
            
        # while not rospy.is_shutdown():
        for i in self.location_induct_one['chutes']:
            # print(self.location_dict['chutes'][i]['px'])
            # pass
            point = PoseStamped()
            point.header.frame_id = 'map'
            point.pose.position.x = self.location_induct_one['chutes'][i]['px']
            point.pose.position.y = self.location_induct_one['chutes'][i]['py']
            self.pub_locations.publish(point)
            rospy.sleep(0.3)
        
        for i in self.location_induct_two['chutes']:
            # print(self.location_dict['chutes'][i]['px'])
            # pass
            point = PoseStamped()
            point.header.frame_id = 'map'
            point.pose.position.x = self.location_induct_two['chutes'][i]['px']
            point.pose.position.y = self.location_induct_two['chutes'][i]['py']
            self.pub_locations.publish(point)
            rospy.sleep(0.3)

    def process_request(self):

        req_1 = GetPlan()
        req_2 = GetPlan()

        self.odom_pose = PoseStamped()
        self.start_pose = PoseStamped()
        self.goal_pose = PoseStamped()

        rospy.sleep(0.4)

        self.odom_pose.header.frame_id = "map"
        self.start_pose.header.frame_id = "map"
        self.goal_pose.header.frame_id = "map"

        self.odom_pose.pose.position.x = self.odom_x
        self.odom_pose.pose.position.y = self.odom_y

        if self.dock_station_name == 'dock_one' :
            self.start_pose.pose.position.x = self.location_induct_one['docks'][self.dock_station_name]['px']
            self.start_pose.pose.position.y = self.location_induct_one['docks'][self.dock_station_name]['py']

            self.goal_pose.pose.position.x = self.location_induct_one['chutes'][self.chute_name]['px']
            self.goal_pose.pose.position.y = self.location_induct_one['chutes'][self.chute_name]['py']
        else:
            self.start_pose.pose.position.x = self.location_induct_two['docks'][self.dock_station_name]['px']
            self.start_pose.pose.position.y = self.location_induct_two['docks'][self.dock_station_name]['py']

            self.goal_pose.pose.position.x = self.location_induct_two['chutes'][self.chute_name]['px']
            self.goal_pose.pose.position.y = self.location_induct_two['chutes'][self.chute_name]['py']
        print('messages initiated')

        print(self.odom_x, self.odom_y)
        print(self.start_pose.pose.position.x, self.start_pose.pose.position.y)
        print(self.goal_pose.pose.position.x, self.goal_pose.pose.position.y)
        rospy.loginfo('Requesting Path')
        self.get_plan = rospy.ServiceProxy('/move_base/make_plan', GetPlan)
        self.path_one = self.get_plan.call(self.odom_pose, self.start_pose, 0.3)
        rospy.sleep(1)
        self.path_two = self.get_plan.call(self.start_pose, self.goal_pose, 0.3)
        rospy.sleep(3)

        self.send_paths()


    
    def get_odom(self):
        x = rospy.wait_for_message('/robot2/odom', Odometry, timeout=20)
        # p = Odometry()
        self.odom_x, self.odom_y = x.pose.pose.position.x, x.pose.pose.position.y

    def send_paths(self):
        # sending path from current position to induct station
        self.sendpath.publish(self.path_one.plan)
        # waiting for sucess string when path execution is done
        self.result_1 = rospy.wait_for_message('/waiting_for_response_two', Bool, timeout=None)
        rospy.loginfo('sending path to the rover_script : ' + str(self.result_1))
        
        # The rover has reached the induct station and waiting for package to be loaded
        ########### LOAD PACKAGE ###########
        rospy.sleep(15)
        ###################################

        # saying to schedular that induct station is free and can accept another request
        rospy.set_param('is_free/' + self.dock_dict[self.dock_station_name], True)

        # Rover now starts traversing from induct station to chute pose
        self.sendpath.publish(self.path_two.plan)

        # Waiting until exec script resopnds with the string that execution is done
        self.result_2 = rospy.wait_for_message('/waiting_for_response_two', Bool, timeout=None)
        rospy.loginfo('sending path to the rover_script : ' + str(self.result_2))
        rospy.sleep(1)

        # Says to schedualr script that rover is free to run
        rospy.set_param('robot2/can_run', True)
        print('Done')

if __name__ == '__main__':
    rospy.init_node('rover_2_node', anonymous=True)
    g = rover_one()
    g.read_locations()
    g.service_server()
