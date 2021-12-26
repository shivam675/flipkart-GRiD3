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

path = rospkg.RosPack()
path = path.get_path('rover_nav')


class rover_one:
    def __init__(self, name='rover_1') -> None:
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
        self.pub_locations = rospy.Publisher('location_publisher', PoseStamped, queue_size=10)

    def service_server(self):
        print('Rospy server started')
        rospy.Service('/schedule/rover_one',get_task ,self.get_task_callback)
        rospy.spin()

    def get_task_callback(self, req):
        # rospy.loginfo('Request Received !')

        rospy.set_param('robot1/is_running', False)
        print('requesting odom')
        self.get_odom()


        self.package_id = req.package_id
        self.package_name = req.package_name
        self.chute_name = req.chute_name
        self.dock_station_name = req.dock_station_name
        # self.path_one = None
        # self.path_two = None
        # m = Path()
        # m.poses

        print('request_recieved')
        self.process_request()
        # 
        return self.return_status
    
    def read_locations(self):
        with open(path + '/info/locations.yaml') as file:
            data = yaml.load(file, yaml.FullLoader)
            # print(data['chutes']['mumbai']['px'])
            self.location_dict = data
            
        # while not rospy.is_shutdown():
        for i in self.location_dict['chutes']:
            # print(self.location_dict['chutes'][i]['px'])
            # pass
            point = PoseStamped()
            point.header.frame_id = 'map'
            point.pose.position.x = self.location_dict['chutes'][i]['px']
            point.pose.position.y = self.location_dict['chutes'][i]['py']
            self.pub_locations.publish(point)
            rospy.sleep(0.3)
        # print(self.location_dict)

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

        # self.odom_pose.header.stamp = rospy.Time.now()
        # self.start_pose.header.stamp = rospy.Time.now()
        # self.goal_pose.header.stamp = rospy.Time.now()

        # self.odom_pose.header.seq = 0
        # self.start_pose.header.seq = 0
        # self.goal_pose.header.seq = 0

        self.odom_pose.pose.position.x = self.odom_x
        self.odom_pose.pose.position.y = self.odom_y

        self.start_pose.pose.position.x = self.location_dict['docks'][self.dock_station_name]['px']
        self.start_pose.pose.position.y = self.location_dict['docks'][self.dock_station_name]['py']

        self.goal_pose.pose.position.x = self.location_dict['chutes'][self.chute_name]['px']
        self.goal_pose.pose.position.y = self.location_dict['chutes'][self.chute_name]['py']

        print('messages initiated')
        ###################################
        # req_1.start = self.odom_pose
        # req_1.goal = self.start_pose
        # req_1.tolerance = 0.2
        ###################################
        # req_2.start = self.start_pose
        # req_2.goal = self.goal_pose
        # req_2.tolerance = 0.2
        ###################################
        # print(self.odom_pose)
        # print(self.start_pose)
        print(self.odom_x, self.odom_y)
        print(self.start_pose.pose.position.x, self.start_pose.pose.position.y)
        print(self.goal_pose.pose.position.x, self.goal_pose.pose.position.y)
        rospy.loginfo('Requesting Path')
        self.get_plan = rospy.ServiceProxy('/move_base/make_plan', GetPlan)
        self.path_one = self.get_plan.call(self.odom_pose, self.start_pose, 0.3)
        rospy.sleep(1)
        self.path_two = self.get_plan.call(self.start_pose, self.goal_pose, 0.3)
        rospy.sleep(3)
        rospy.set_param('robot1/is_running', True)
        # print(self.path_one)
        # print(self.path_two)
        # print(len(self.path_two.plan.poses))

    
    def get_odom(self):
        x = rospy.wait_for_message('/robot1/odom', Odometry, timeout=20)
        # p = Odometry()
        self.odom_x, self.odom_y = x.pose.pose.position.x, x.pose.pose.position.y

    def send_paths(self):
        rospy.set_param('robot1/is_running', True)
        sendpath = rospy.ServiceProxy('robot1/exec_path', path_service)
        result_1 = sendpath(path = self.path_one)
        rospy.loginfo('sending path to the rover_script : ' + str(result_1))
        result_2 = sendpath(path = self.path_two)
        rospy.loginfo('sending path to the rover_script : ' + str(result_2))


if __name__ == '__main__':
    rospy.init_node('rover_1_node', anonymous=True)
    g = rover_one()
    g.read_locations()
    g.service_server()
