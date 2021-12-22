#!/usr/bin/python3
from os import close, stat
# from threading import TIMEOUT_MAX
import rospy
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import Twist, PoseStamped
from math import frexp, pow, atan2, sqrt
from nav_msgs.msg import Odometry, Path
from tf.transformations import euler_from_quaternion
import sys
from threading import Thread
import matplotlib.pyplot as plt
import math
from nav_msgs.srv import GetPlan

show_animation = False
# show_animation = True


class Dijkstra:

    def __init__(self, ox, oy, resolution, robot_radius):
        """
        Initialize map for a star planning

        ox: x position list of Obstacles [m]
        oy: y position list of Obstacles [m]
        resolution: grid resolution [m]
        rr: robot radius[m]
        """

        self.min_x = None
        self.min_y = None
        self.max_x = None
        self.max_y = None
        self.x_width = None
        self.y_width = None
        self.obstacle_map = None

        self.resolution = resolution
        self.robot_radius = robot_radius
        self.calc_obstacle_map(ox, oy)
        self.motion = self.get_motion_model()

    class Node:
        def __init__(self, x, y, cost, parent_index):
            self.x = x  # index of grid
            self.y = y  # index of grid
            self.cost = cost
            self.parent_index = parent_index  # index of previous Node

        def __str__(self):
            return str(self.x) + "," + str(self.y) + "," + str(
                self.cost) + "," + str(self.parent_index)

    def planning(self, sx, sy, gx, gy):
        """
        dijkstra path search

        input:
            s_x: start x position [m]
            s_y: start y position [m]
            gx: goal x position [m]
            gx: goal x position [m]

        output:
            rx: x position list of the final path
            ry: y position list of the final path
        """

        start_node = self.Node(self.calc_xy_index(sx, self.min_x),
                               self.calc_xy_index(sy, self.min_y), 0.0, -1)
        goal_node = self.Node(self.calc_xy_index(gx, self.min_x),
                              self.calc_xy_index(gy, self.min_y), 0.0, -1)

        open_set, closed_set = dict(), dict()
        open_set[self.calc_index(start_node)] = start_node
        print(open_set, start_node)
        while 1:
            c_id = min(open_set, key=lambda o: open_set[o].cost)
            current = open_set[c_id]

            # show graph
            if show_animation:  # pragma: no cover
                plt.plot(self.calc_position(current.x, self.min_x),
                         self.calc_position(current.y, self.min_y), "xc")
                # for stopping simulation with the esc key.
                plt.gcf().canvas.mpl_connect(
                    'key_release_event',
                    lambda event: [exit(0) if event.key == 'escape' else None])
                if len(closed_set.keys()) % 10 == 0:
                    plt.pause(0.001)

            if current.x == goal_node.x and current.y == goal_node.y:
                print("Find goal")
                goal_node.parent_index = current.parent_index
                goal_node.cost = current.cost
                break

            # Remove the item from the open set
            del open_set[c_id]

            # Add it to the closed set
            closed_set[c_id] = current

            # expand search grid based on motion model
            for move_x, move_y, move_cost in self.motion:
                node = self.Node(current.x + move_x,
                                 current.y + move_y,
                                 current.cost + move_cost, c_id)
                n_id = self.calc_index(node)

                if n_id in closed_set:
                    continue

                if not self.verify_node(node):
                    continue

                if n_id not in open_set:
                    open_set[n_id] = node  # Discover a new node
                else:
                    if open_set[n_id].cost >= node.cost:
                        # This path is the best until now. record it!
                        open_set[n_id] = node

        rx, ry = self.calc_final_path(goal_node, closed_set)

        return rx, ry

    def calc_final_path(self, goal_node, closed_set):
        # generate final course
        rx, ry = [self.calc_position(goal_node.x, self.min_x)], [
            self.calc_position(goal_node.y, self.min_y)]
        parent_index = goal_node.parent_index
        while parent_index != -1:
            n = closed_set[parent_index]
            rx.append(self.calc_position(n.x, self.min_x))
            ry.append(self.calc_position(n.y, self.min_y))
            parent_index = n.parent_index

        return rx, ry

    def calc_position(self, index, minp):
        pos = index * self.resolution + minp
        return pos

    def calc_xy_index(self, position, minp):
        return round((position - minp) / self.resolution)

    def calc_index(self, node):
        return (node.y - self.min_y) * self.x_width + (node.x - self.min_x)

    def verify_node(self, node):
        px = self.calc_position(node.x, self.min_x)
        py = self.calc_position(node.y, self.min_y)

        if px < self.min_x:
            return False
        if py < self.min_y:
            return False
        if px >= self.max_x:
            return False
        if py >= self.max_y:
            return False

        if self.obstacle_map[node.x][node.y]:
            return False

        return True

    def calc_obstacle_map(self, ox, oy):

        self.min_x = round(min(ox))
        self.min_y = round(min(oy))
        self.max_x = round(max(ox))
        self.max_y = round(max(oy))

        self.x_width = round((self.max_x - self.min_x) / self.resolution)
        self.y_width = round((self.max_y - self.min_y) / self.resolution)

        # obstacle map generation
        self.obstacle_map = [[False for _ in range(self.y_width)]
                             for _ in range(self.x_width)]
        for ix in range(self.x_width):
            x = self.calc_position(ix, self.min_x)
            for iy in range(self.y_width):
                y = self.calc_position(iy, self.min_y)
                for iox, ioy in zip(ox, oy):
                    d = math.hypot(iox - x, ioy - y)
                    if d <= self.robot_radius:
                        self.obstacle_map[ix][iy] = True
                        break

    @staticmethod
    def get_motion_model():
        # dx, dy, cost
        motion = [[1, 0, 1],
                  [0, 1, 1],
                  [-1, 0, 1],
                  [0, -1, 1],
                  [-1, -1, math.sqrt(2)],
                  [-1, 1, math.sqrt(2)],
                  [1, -1, math.sqrt(2)],
                  [1, 1, math.sqrt(2)]]

        return motion


class dijkstra_ros:

    def __init__(self):
        self.start_x = None
        self.start_y = None
        self.trans_x = None
        self.trans_y = None
        self.start_pose_x_mtrs = None
        self.start_pose_y_mtrs = None
        # self.rate = rospy.Rate(10)
        self.frame_id = 'map'
        self.pose_x = 0
        self.pose_y = 0
        self.theta = 0
        self.waypoints = []
        self.map_msg = rospy.wait_for_message("/map", OccupancyGrid, timeout=None)
        # path message object
        # self.path = Path()
        # self.path.header.frame_id = 'map'
        # self.path.header.stamp = rospy.Time.now()
        # path waypoint list
        self.path_list = []
        self.path_publisher = rospy.Publisher('/path', Path, queue_size=10)
        # self.path_publisher = rospy.Publisher('/move_base/NavfnROS/plan', Path, queue_size=10)
        # 2D graph initializer
        self.d2_graph = []
        self.map_info = self.map_msg.info
        self.map_res = self.map_msg.info.resolution
        self.map_height = self.map_msg.info.height
        self.map_width = self.map_msg.info.width
        self.occupancy_threshold = 0.65
        self.list_of_pixel_coordinates = []
        self.turn_counter = 0
        
        # ########## Call functions already ###########33

        self.one_to_two_d_graph()
        self.gen_obstracle_poses()



    def one_to_two_d_graph(self):
        """
        This function converts one dimentional map.data array to a
        nested array of dimentions map.info.width X map.info.height
        so map looks somewhat like following
        [
            [0, 0, 0, 0, -1],
            [0, 1, 0, 0, 0],
            [0, 0, 1, 0, 0],
            [0, 1, 0, 0, 0],
            [0, 0, 0, -1, 0]

        ]

        map coordinate axis
            (0,0)
            +---------------x
            |               |
            |               |
            |               |
            |               |
            |               |
            y---------------+

        """
        one_d_graph = self.map_msg.data

        while one_d_graph:
            self.d2_graph.append(one_d_graph[:self.map_msg.info.width])
            one_d_graph = one_d_graph[self.map_msg.info.width:]

        # reversing the map graph array to normalize the image
        self.d2_graph = self.d2_graph[::-1]

    def gen_obstracle_poses(self):
        self.ox , self.oy = [], [] 
        for i in range(len(self.d2_graph)):
            for j in range(len(self.d2_graph[0])):
                if self.d2_graph[i][j] >= 0.65:   # 0.65 is obstracle threshold
                    self.ox.append(j)
                    self.oy.append(i)
        print('Done generating obs poses')



    def transform_coordinates_to_grid(self, x_mtrs,  y_mtrs):   
        '''
        NOTE: This function takes x,y values in meters
         and transforms it to 2D graph coordinates
        '''
        self.trans_x = self.map_width/2
        self.trans_y = self.map_height/2

        return self.trans_x + x_mtrs/self.map_res, self.trans_y - y_mtrs/self.map_res

    def transform_coordinates_to_rviz(self, x_px, y_px):
        '''
        NOTE: This function takse x,y values in index
        of grid map and transforms it into Rviz mtr coordinates
        '''
        self.trans_x = self.map_width/2
        self.trans_y = self.map_height/2

        x_m, y_m = self.map_res*(x_px - self.trans_x), self.map_res*(self.trans_y - y_px)
        return x_m, y_m

    def publish_path(self, x, y):
        '''
        NOTE: This function is publishes the path with
        respect to rviz coordinates in meters
        '''

        msg = Path()
        msg.header.frame_id = "map"
        list_ = msg.poses
        for i, j in zip(x[::-1], y[::-1]):
            # print(i,j)
            x_mtrs, y_mtrs = self.transform_coordinates_to_rviz(i,j)
            pose_msg = PoseStamped()
            pose_msg.header.frame_id = "map"
            msg.header.stamp = rospy.Time.now()
            pose_msg.pose.position.x = round(x_mtrs,2)
            pose_msg.pose.position.y = round(y_mtrs, 2)    
            # print(pose_msg)
            list_.append(pose_msg)
        rospy.loginfo("No of Points : " + str(len(msg.poses)))
        self.path_publisher.publish(msg)
        return msg



    def initiate_dijkstra(self, x_start, y_start, x_goal, y_goal):
        start_time = rospy.get_rostime()
        rospy.loginfo("Inititated ...")

        # start and goal position
        # print(ox,oy)
        sx, sy = self.transform_coordinates_to_grid(x_start, y_start)  # [m]
        gx, gy = self.transform_coordinates_to_grid(x_goal, y_goal) # [m]
        print(sx,sy,gx,gy)
        # grid_size = 0.2 # [m]  ###  BEST for navigation
        grid_size = 1  # [m]  ###  BEST for navigation
        robot_radius = 5 # [m]

        if show_animation:  # pragma: no cover
            plt.plot(self.ox, self.oy, ".k")
            plt.plot(sx, sy, "og")
            plt.plot(gx, gy, "xb")
            plt.grid(True)
            plt.axis("equal")


        dijkstra = Dijkstra(self.ox, self.oy, grid_size, robot_radius)
        rx, ry = dijkstra.planning(sx, sy, gx, gy)
        path = self.publish_path(rx, ry)
        end_time = rospy.get_rostime()
        rospy.logwarn("Time taken to plan : " + str(round(end_time.secs - start_time.secs, 3)))
        if show_animation:  # pragma: no cover
            plt.plot(rx, ry, "-r")
            plt.pause(1)
            plt.show()
        return path




    def path_server(self):
        rospy.loginfo('Started Listening !')
        s = rospy.Service('get_plan', GetPlan, requesting_path)
        s.spin()

    def requesting_path(self,req):
        rospy.loginfo("Received Nav Request !")
        # p = PoseStamped()
        start_x, start_y = req.start.pose.position.x,req.start.pose.position.y
        goal_x, goal_y = req.goal.pose.position.x, req.goal.pose.position.y
        # p.pose.position.
        rospy.loginfo('start pose > x: ' + str(start_x) + ' y: ' + str(start_y))
        rospy.loginfo('goal pose > x: ' + str(goal_x) + ' y: ' + str(goal_y))

        path = self.initiate_dijkstra(start_x, start_y, goal_x, goal_y)
        # rospy.loginfo(req.start.pose.position.x,req.start.pose.position.y)
        return path

    def test_dijkstra(self):
        path = self.initiate_dijkstra(0, -0.5, 0, 0.5)


if __name__ == '__main__':
    rospy.init_node("dijkstra_algo", anonymous=True)
    # path_server()
    k = dijkstra_ros()
    k.test_dijkstra()
    # rospy.spin_once()
    sys.exit()