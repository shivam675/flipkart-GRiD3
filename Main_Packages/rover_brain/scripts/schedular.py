#!/usr/bin/python3

from codecs import encode
import rospy
import csv
from threading import Thread
from rover_nav.srv import get_task, path_service
import rospkg

path = rospkg.RosPack()
path = path.get_path('rover_brain')

class schedualar:
    def __init__(self) -> None:
        self.rover_1_status = None
        self.rover_2_status = None
        self.keep_checking_status = True
        self.open_list = []
        self.fields = []
        self.close_list = []
        self.package_count = 0
        self.induct_station_one = None
        self.induct_station_two = None
        pass


    def thread_for_service(self):
        self.t = Thread(target = self.check_who_is_free)
        self.t.start()

    def check_who_is_free(self):
        while self.keep_checking_status:
            if rospy.get_param('robot1/is_running', default=False):
                self.rover_1_status = True
            if rospy.get_param('robot2/is_running', default=False):
                self.rover_2_status = True
            if rospy.get_param('induct_station_1', default = False):
                self.induct_station_one = True
            if rospy.get_param('induct_station_1', default = False):
                self.induct_station_two = True
            rospy.sleep(0.5)


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


    def plan(self):
        while self.open_list:

            if self.open_list[0][1] == '1':
                if self.rover_1_status and self.induct_station_one:
                    data_list = self.open_list.pop(0)
                    mes = self.encode_goals(data_list[0], data_list[1], data_list[2])
                    exec_srv = rospy.ServiceProxy('/schedule/rover_one', get_task)
                    exec_srv(mes)

            if self.open_list[0][1] == '2':
                if self.rover_1_status and self.induct_station_two:
                    data_list = self.open_list.pop(0)
                    mes = self.encode_goals(data_list[0], data_list[1], data_list[2])
                    exec_srv = rospy.ServiceProxy('/schedule/rover_two', get_task)
                    exec_srv(mes)

            rospy.sleep(0.5)



    def read_csv(self):
        with open('data/data.csv', 'r') as csvfile:
            csvreader = csv.reader(csvfile)
            self.fields = next(csvreader)
            # print(csvreader)
            for row in csvreader:
                self.open_list.append(row)


if __name__ == '__main__':
    k = schedualar()
    k.read_csv()
    k.thread_for_service()
    k.plan()
