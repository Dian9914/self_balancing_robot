#!/usr/bin/env python
import rospy
import math
import numpy as np
from std_msgs.msg import Float32
from self_balancing_robot.msg import imu_pose

class LQR_controller():
    def __init__(self):
        # Variables de estado
        self.phi=0
        self.phi_dot=0
        self.uk=0
        self.xk=np.array([[0],[0],[0],[0]],dtype=float)
        self.Ik=0
        # Parametros de control
        self.Ki=1.0098
        self.KLQR=np.array([-185.8177, -17.6151, -2.4795, self.Ki])
        # Parametros de funcionamiento
        self.sat=10000

        # Inicializamos el subscriber para leer del topic pose
        rospy.Subscriber("/sbr/pose", imu_pose , self.callback)

    def callback(self,data):
        # Guardamos los datos del estado que tomamos del topic
        self.phi=data.phi
        self.phi_dot=data.phi_dot

    def control(self):
        # La actuacion se publicara a 50Hz
        r = rospy.Rate(50)
        # Si el robot se inclina demasiado, el control se detiene y el programa termina
        while not rospy.is_shutdown() and not (self.phi >= 90 or self.phi <= -90):
            # Calculamos la accion de control
            self.xk[0]=[self.phi]
            self.xk[1]=[self.phi_dot]
            self.xk[2]=[self.uk]
            self.xk[3]=[self.Ik]
            self.uk=np.dot(-self.KLQR,self.xk)
            self.Ik=self.Ik-self.phi
            # Saturation
            if self.uk > self.sat: 
                self.uk = self.sat
            elif self.uk < -self.sat:
                self.uk = -self.sat
            # Publicamos la accion de control
            pub.publish(self.uk)
            r.sleep()
        pub.publish(0)
        
         


if __name__=='__main__':
    rospy.init_node('control',anonymous=True)
    pub = rospy.Publisher('/sbr/actuation', Float32, queue_size=10)
    control=LQR_controller()
    control.control()
        
            