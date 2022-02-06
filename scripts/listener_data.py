#!/usr/bin/env python

import rospy
import math
from std_msgs.msg import Float32
from self_balancing_robot.msg import imu_pose

class plotter():
    def __init__(self):
        # preparamos un archivo donde volcar los datos
        self.f=open('./data_log.txt','w')

        self.phi = 0
        self.phi_dot = 0
        self.actuation = 0

        rospy.Subscriber("/sbr/pose", imu_pose , self.callback_imu)
        rospy.Subscriber("/sbr/actuation", Float32 , self.callback_motors)

    def callback_imu(self, data):
        self.phi=data.phi
        self.phi_dot=data.phi_dot

    def callback_motors(self, data):
        self.actuation=data.data

    def save_data(self):
        self.f.write('%.2f\t%.2f\t%.2f\n' % (self.phi, self.phi_dot, self.actuation))

    def close(self):
        self.f.close


if __name__=='__main__':
    rospy.init_node('data',anonymous=True)
    plot=plotter()
    r = rospy.Rate(10)
    while not rospy.is_shutdown():
        plot.save_data()
        r.sleep()
    plot.close()