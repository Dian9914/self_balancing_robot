#!/usr/bin/env python
import RPi.GPIO as GPIO
import rospy
import math
from std_msgs.msg import Float32
from time import sleep

class stepper():
    def __init__(self, DIR, STEP, POSITION):
        # Direction pin from controller
        self.DIR = DIR
        # Step pin from controller
        self.STEP = STEP
        # 0/1 used to signify clockwise or counterclockwise.
        if POSITION=='left':
            self.CW = 0
            self.CCW = 1
        else:
            self.CW = 1
            self.CCW = 0
        
        # Setup pin layout on PI
        GPIO.setmode(GPIO.BOARD)

        # Establish Pins in software
        GPIO.setup(self.DIR, GPIO.OUT)
        GPIO.setup(self.STEP, GPIO.OUT)

        # Set the first direction you want it to spin
        GPIO.output(self.DIR, self.CW)

        # Flag for the steps
        self.HL=True
        self.change_DIR=False

    def set_direction(self,speed):
        if speed < 0:
            GPIO.output(self.DIR, self.CW)
        else:
            GPIO.output(self.DIR, self.CCW)
        self.change_DIR=True
        
    def step(self):
        if not self.change_DIR:
            if self.HL:
                GPIO.output(self.STEP,GPIO.HIGH)
                self.HL=False
            else:
                GPIO.output(self.STEP,GPIO.LOW)
                self.HL=True 
        else:           
            self.change_DIR=False

            
class Controller:
    def __init__(self):
        # Variables
        self.speed=0
        self.Frec=10
        self.sat=1000
        self.sleep_d=0.1
        #Flag
        self.Stop=True
        # Motor object
        self.right_motor=stepper(8,10,'left')
        self.left_motor=stepper(11,12,'right')


    def callback(self,data):
        #rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)
        self.speed=data.data
        # Saturation
        if self.speed > self.sat: 
            self.speed = self.sat
        elif self.speed < -self.sat:
            self.speed = -self.sat
        # Set the spin direction for each motor 
        self.right_motor.set_direction(self.speed)
        self.left_motor.set_direction(self.speed)
        # Calculate the frecuency needed to reach that speed
        self.Frec=2.0*(1600.0/360.0)*abs(self.speed)
        #rospy.loginfo(rospy.get_caller_id() + " SPEED: %s", data.data)
        if self.Frec==0:
            self.Stop=True
        else:
            self.Stop=False
            self.sleep_d=1/self.Frec

    
    def sub_stepper(self):
        rospy.Subscriber("/sbr/actuation", Float32 , self.callback)
        
    
    def motor_control(self):
        try:
            while not rospy.is_shutdown():
                if not self.Stop:
                    self.left_motor.step()
                    self.right_motor.step()
                sleep(self.sleep_d)
        except KeyboardInterrupt:
            print("cleanup")
            GPIO.cleanup()    


if __name__=='__main__':
    rospy.init_node('sub_stepper',anonymous=True)
    control=Controller()
    control.sub_stepper()
    control.motor_control()

    