#!/usr/bin/env python3
import RPi.GPIO as GPIO


# import the library
from RpiMotorLib import RpiMotorLib

#GPIO pins 
GPIO_pins = (-1, -1) # Microstep Resolution MS1-MS2 -> GPIO Pin
direction= 8       # Direction -> GPIO Pin
step = 10    # Step -> GPIO Pin

# Declare an named instance of class, pass GPIO-PINs
mymotortest = RpiMotorLib.A3967EasyNema(direction, step, GPIO_pins)

# call the function, pass the arguments, In this example
# we move 200 steps in full mode(one revolution) after an initdelay
# of 50mS with step delay of 5mS, Clockwise direction and verbose output on. 

mymotortest.motor_move(.005, 200 , False, True, "Full", .05)

# good practise to cleanup GPIO at some point before exit
GPIO.cleanup()