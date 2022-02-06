#!/usr/bin/env python
import qwiic_icm20948
import math
import rospy
from std_msgs.msg import Float32
from self_balancing_robot.msg import imu_pose

class readIMU():
    # Objeto encargado de manejar la IMU
    def __init__(self,delta_t):
        # Inicializacion de la IMU
        self.IMU = qwiic_icm20948.QwiicIcm20948()
        # Banderas
        self.start_flag=True
        # Parametros
        self.delta_t=delta_t
        # Parametros Kalman filter
        self.Ak=1
        self.Hk=1
        self.Pest=0
        self.Ppred=0
        self.Vk=1
        self.Wk=1
        self.dgyro=0.5
        self.daccel=0.25
        self.Q=self.dgyro**2
        self.R=self.daccel**2
        # Parametros data translation
        g = 9.81
        self.accel_translation = 2*g/32767
        self.gyro_translation = (500.0/32767)


    def initialize(self):
        # Metodo encargado de inicializar el sensor y comprobar que la conexion es correcta
        # Si la conexion esta mal, devuelve False
        if self.IMU.connected == False:
            rospy.loginfo("The Qwiic ICM20948 device isn't connected to the system. Please check your connection")
            return False 
        self.IMU.begin()
        self.IMU.setFullScaleRangeAccel(qwiic_icm20948.gpm2)
        self.IMU.setFullScaleRangeGyro(qwiic_icm20948.dps500)
        return True

    def close(self):
        self.f.close()
        return True
       
    def read_roll(self):
        # Metodo encargado de leer los datos de la IMU, traducirlos y filtrarlos, devolviendo el roll del robot
        if self.IMU.dataReady(): # Si hay datos disponibles
            self.IMU.getAgmt() # read all axis and temp from sensor, note this also updates all instance variables
            #######################################################
            # Data translation from signed 16 bit int to real units
            a_y=float(self.IMU.ayRaw)*self.accel_translation
            a_z=float(self.IMU.azRaw)*self.accel_translation
            g_x=float(self.IMU.gxRaw)*self.gyro_translation
            g_x=g_x
            # Get the IMU orientation
            try:
                self.roll_accel=math.atan(-a_z/a_y)*(180.0/math.pi)
            except ZeroDivisionError:
                self.roll_accel=self.roll_gyro

            if self.start_flag: # El punto inicial de la integracion se tomara segun los valores leidos por el acelerometro
                self.roll_gyro=self.roll_accel
                self.roll_est=self.roll_accel
                self.start_flag = False
            self.roll_gyro=self.roll_gyro+g_x*self.delta_t

            # Apply kalman filter
            # prediction
            self.roll_pred=self.roll_est+g_x*self.delta_t
            self.Ppred=self.Ak*self.Pest*self.Ak+self.Wk*self.Q*self.Wk
            # correction
            self.K=self.Ppred*self.Hk*((self.Hk*self.Ppred*self.Hk+self.Vk*self.R*self.Vk)**(-1))
            self.roll_est=self.roll_pred+self.K*(self.roll_accel-self.roll_pred)
            self.Pest=(1-self.K*self.Hk)*self.Ppred


            return self.roll_est, g_x

        #######################################################

if __name__ == '__main__':
    # Parametros
    freq=50
    delta_t=1/freq
    # Inicializacion del nodo y el publisher
    pub = rospy.Publisher('/sbr/pose', imu_pose, queue_size=10)
    pose = imu_pose()
    rospy.init_node('data_collection')
    r = rospy.Rate(freq) # 100hz
    # Inicializacion de la imu
    imu=readIMU(delta_t)    
    if imu.initialize():  
        # Bucle infinito
        while not rospy.is_shutdown():
            phi, phi_dot = imu.read_roll() # Lectura y procesamiento de los datos
            pose.phi=phi
            pose.phi_dot=phi_dot
            rospy.loginfo(rospy.get_caller_id() + "Im reading phi: %.2f and phi_dot: %.2f", phi, phi_dot)
            pub.publish(pose) # Se publica el valor obtenido 
            r.sleep()
