#!/usr/bin/python3

import rospy
import csv
from threading import Thread
from rospy.timer import sleep
from rover_nav.srv import get_task, path_service
import rospkg

path = rospkg.RosPack()
path = path.get_path('rover_brain')

class schedualar:
    def __init__(self) -> None:
        self.rover_1_status = None
        self.rover_2_status = None
        self.keep_checking_status = True
        self.open_list_one = []
        self.open_list_two = []
        self.fields = []
        self.close_list = []
        self.package_count = 0
        self.induct_station_one = None
        self.induct_station_two = None
        self.rover_status_dict = {}
        self.free_rover = ''
        rospy.set_param('robot1/can_run', True)
        rospy.set_param('robot2/can_run', True)
        rospy.set_param('is_free/induct_station_1', True)
        rospy.set_param('is_free/induct_station_2', True)
        pass


    def thread_for_service(self):
        self.t = Thread(target = self.check_who_is_free)
        self.t.start()

    def check_who_is_free(self):
        while self.keep_checking_status:
            self.rover_status_dict['rover_one'] = rospy.get_param('robot1/can_run', default=False)
            # self.rover_status_dict['rover_one'] = rospy.get_param('robot1/can_run', default=False)
            self.rover_status_dict['rover_two'] = rospy.get_param('robot2/can_run', default=False)
            ####################### 
            # Check which induct is free
            #######################
            self.induct_station_one = rospy.get_param('is_free/induct_station_1', default = False)
            # self.induct_station_one = rospy.get_param('induct_station_1', default = False)
            self.induct_station_two = rospy.get_param('is_free/induct_station_2', default = False)
            print(self.rover_status_dict)
            print(self.induct_station_one, self.induct_station_two)
            rospy.sleep(0.4)


    def encode_goals(self, Shipment, Induct_Station,Destination):
        '''
        int32 package_id
        string package_name
        string chute_name
        string dock_station_name
        '''
        message = get_task()
        message.package_id = self.package_count
        message.package_name = Shipment
        message.chute_name = Destination
        message.dock_station_name = Induct_Station

        return message

    def get_free_rover(self):
        for j in self.rover_status_dict:
            if self.rover_status_dict[j]:
                self.free_rover = j
                break
            else:
                self.free_rover = None


    def plan_for_induct_one(self):
        while self.open_list_one:
            rospy.sleep(0.5)
            try:
                self.get_free_rover()
                # rospy.loginfo('new package initiated')
                if self.free_rover != None and self.induct_station_one:
                    rospy.set_param('is_free/induct_station_1', False)
                    print('Got Free Rover for Induct 1 : {}'.format(self.free_rover))
                    data_list = self.open_list_one.pop(0)
                    mes = self.encode_goals(data_list[0], 'dock_one', data_list[2])
                    exec_srv = rospy.ServiceProxy('/schedule/' + self.free_rover, get_task)
                    exec_srv(mes.package_id, mes.package_name, mes.chute_name, mes.dock_station_name)
                    self.package_count += 1
            except TypeError:
                pass


    def plan_for_induct_two(self):
        while self.open_list_two:
            rospy.sleep(1.2)
            try:
                self.get_free_rover()
                # rospy.loginfo('new package initiated')
                if self.free_rover != None and self.induct_station_two:
                    rospy.set_param('is_free/induct_station_2', False)
                    print('Got Free Rover for Induct 2 : {}'.format(self.free_rover))
                    data_list = self.open_list_two.pop(0)
                    mes = self.encode_goals(data_list[0], 'dock_two', data_list[2])
                    exec_srv = rospy.ServiceProxy('/schedule/' + self.free_rover, get_task)
                    exec_srv(mes.package_id, mes.package_name, mes.chute_name, mes.dock_station_name)
                    self.package_count += 1
            except TypeError:
                pass
    

    def thread_for_plan_one(self):
        self.t_one = Thread(target = self.plan_for_induct_one)
        self.t_one.start()

    def thread_for_plan_two(self):
        self.t_two = Thread(target = self.plan_for_induct_two)
        self.t_two.start()


    def read_csv(self):
        with open(path + '/scripts/data/data.csv', 'r') as csvfile:
            csvreader = csv.reader(csvfile)
            self.fields = next(csvreader)
            # print(csvreader)
            for row in csvreader:
                if row[1] == '1':
                    self.open_list_one.append(row)
                else:
                    self.open_list_two.append(row)
        rospy.loginfo("Number of Packages for Induct 1 : " + str(len(self.open_list_one)))
        rospy.loginfo("Number of Packages for Induct 2 : " + str(len(self.open_list_two)))

if __name__ == '__main__':
    rospy.init_node('main_schedular', anonymous=True)
    k = schedualar()
    rospy.sleep(0.5)
    k.read_csv()
    k.thread_for_service()
    k.thread_for_plan_one()
    rospy.loginfo('1 main thread and 2 sub threads running')
    # k.plan_for_induct_two()
    k.thread_for_plan_two()
    rospy.loginfo('All Done')
