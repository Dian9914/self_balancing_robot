#!/usr/bin/env python
import rospy
import math
from std_msgs.msg import Float32
from self_balancing_robot.msg import imu_pose

class PID_controller():
    def __init__(self):
        rospy.Subscriber("/sbr/pose", imu_pose , self.callback)
        # Controlador
        self.Ik=0
        self.ek=0
        self.ekk=0
        self.uk=0
        #PID parameters
        self.T=0.02
        self.Kp=35
        self.Td=0.05
        self.Ti=600
        # Parametros de funcionamiento
        self.sat=1000


    def callback(self,data):
        self.ek=-data.phi

    def control(self):       
        r = rospy.Rate(20)
        while not rospy.is_shutdown() and not (self.ek >= 45 or self.ek <= -45):           
            self.Ik=self.Ik+self.ek
            self.uk=self.Kp*(self.ek+(self.T/self.Ti)*self.Ik+(self.Td/self.T)*(self.ek-self.ekk))
            self.ekk=self.ek
            pub.publish(self.uk)
            # Saturation
            if self.uk > self.sat: 
                self.uk = self.sat
            elif self.uk < -self.sat:
                self.uk = -self.sat 
            pub.publish(self.uk)
            r.sleep()
        pub.publish(0)

if __name__=='__main__':
    rospy.init_node('control',anonymous=True)
    pub = rospy.Publisher('/sbr/actuation', Float32, queue_size=10)
    control=PID_controller()
    control.control()
        
            