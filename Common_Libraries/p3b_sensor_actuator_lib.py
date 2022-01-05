"""
This code represents the library that students will reference to complete DESIGN STUDIO activities for Project 3
All code is currently set up to run on a raspberry pi ONLY.

Libaries were split into two due to the backend operation of the simulation. P3a_lib.py shall be used in the lab and
P3b_lib.py in the design studio.

During the design studio, it is assumed that students have already gone through the activity of determining the bin ID
and bottle mass. Given the variability in solutions that students may come up with, the intent of this library is to
first provide them with bin IDs and bottle masses as that was already been evaluated, and to ensure that all students have similar starting points.

Items to mention to students
1. Rotation limits base +/- 175 deg, shoulder +/- 90 deg, elbow +90 -80 deg, wrist +/-170, gripper 0(open)-1(close)
2. P3a_lib.py shall be used in the lab and P3b_lib.py in the design studio
"""

# Import all required libraries
import sys
sys.path.append('../')

#import numpy as np
import time
import os
import math

#Change GPIO 21 into SCLK requried for the ADC
import subprocess
subprocess.run("sudo pigpiod", capture_output=True, shell=True)
subprocess.run("pigs m 21 4", capture_output=True, shell=True)
subprocess.run("sudo killall pigpiod", capture_output=True, shell=True)

# Probably don't need these below:
"""
from Common_Libraries.postman import postman
from Common_Libraries.modular_comm import comm_modular_container
# from Common_Libraries.quanser_sim_lib import rotarytable_sim, QArm_sim, QBot2e_sim, CameraUI
from Common_Libraries.quanser_sim_lib import *

from Common_Libraries.modular_comm import comm_modular_server
from array import *
"""
import random
import board
import busio
import digitalio
from adafruit_motorkit import MotorKit
from adafruit_motor import stepper
import RPi.GPIO as GPIO
from Adafruit_MCP3008 import MCP3008
#initialize the QuanserSim environment

#QIL = postman(18001) #Establish communication
loop_counter = 0
servo_speed = 0.15
interval = 0.2

# Servo Table Constants
# Container properties
empty_plastic_container = 9.25 # empty mass of plastic container in g
empty_metal_container = 15.0 # empty mass of metal container in g
empty_paper_container = 10.0 # empty mass of paper container in g

# QBot Constants
wheel_to_wheel_distance = 0.235
camera_bumper_depth = 0.156
row = 385
col = 319

#Pins
#set GPIO Pins
GPIO_TRIGGER = 23
GPIO_ECHO = 24
GPIO_LDR = 12
GPIO_S2 = 13
GPIO_S3 = 16
GPIO_OUT = 19

GPIO_left = 18
GPIO_right = 27
#ADC Pins
CLK  = 21
MISO = 9
MOSI = 10
CS   = 22

class mount:
    def __init__(self):
        
        # activated actuator
        self.actuator = MotorKit(i2c=board.I2C())
        self.actuator.stepper1.release()
        self.actuator.motor3.throttle = 0
        
        self.stepper_motor_activated = False
        self.linear_actuator_activated = False
        self.linear_actuator_time_counter = 0 # used to keep track of time since the linear actuator doesn't have an encoder
        self.linear_actuator_max_time = 5 # (50mm/10mm/s) Assume that the actuator moves at 10mm/s (0.4 times full speed)
        self.actuation_time = 0

        maximum_travel_speed = 25.0
        desired_travel_speed = 10.0  # desired rate is 10mm/s. Max in data sheets is 25 mm/s no load
        self.throttle = desired_travel_speed / maximum_travel_speed

        # activated Sensors
        self.ultrasonic_sensor_activated = False
        self.hall_sensor_activated = False
        self.ir_sensor_activated = False
        self.ldr_sensor_activated = False

        self.color_sensor_activated = False
        
        self.photoelectric_sensor_activated = False

        # Sensor ranges.
        self.ultrasonic_sensor_range = 2.5 # Adafruit HC-SR04, source = datasheet. 250 cm
        self.hall_sensor_range = 0.50 # 50 cm
        self.ir_sensor_range = 0.50 # 50 cm
        self.color_sensor_range = 0.50 # 50 cm
        self.photoelectric_sensor_range = 0.50 # 50 cm

    # Takes an input file from the modelling sub-team that contains time and angle data.
    # Recommended file type is a .txt file without headers i.e. string characters identifying
    # the time and angle columns. It is assumed that the first column is time (s) and the
    # second column is the angle (deg)
    def process_file(self, filename):
        rotation_time = []
        rotation = []
        with open(filename, "r") as f:
            for line in f:
                line = line.strip()
                line_pair = line.split("\t")  # assuming translation and rotation coordinates are spearated by \t

                rotation_time.append(float(line_pair[0]))
                rotation.append(float(line_pair[1]))
        return rotation_time, rotation

    # -----------------------------------------------------------------------------------------------
    # Activates the stepper motor
    def activate_stepper_motor(self):
        self.stepper_motor_activated = True
        print("Stepper motor activated.")

    # Deactivates the stepper motor
    def deactivate_stepper_motor(self):
        self.stepper_motor_activated = False
        self.actuator.stepper1.release()

        #self.reset_box()
        print("Stepper motor deactivated.")

    # Activates the linear actuator
    def activate_linear_actuator(self):
        self.linear_actuator_activated = True
        print("Linear actuator activated.")

    # Deactivates the linear actuator
    def deactivate_linear_actuator(self):
        self.linear_actuator_activated = False
        self.actuator.motor3.throttle = 0
        print("Linear actuator deactivated.")

        #WHAT HAPPENS IF NOT MOVED ALL THE WAY BACK HOME. NEED TO CONSIDER THAT! IF STATEMENT TO SAY MOVE THE ACTUATOR BACK HOME?
        self.linear_actuator_time_counter = 0 # used to keep track of time since the linear actuator doesn't have an encoder
        self.actuation_time = 0

    def rotate_stepper_cw(self,rotation_time):
        if self.stepper_motor_activated:
            stepper_speed = 90  # Desired speed in deg/s i.e. 15 RPM
            deg_per_step = 1.8 # Motor specification (200 steps/revolutions)
            deg = stepper_speed * rotation_time
            steps = deg/deg_per_step

            for i in range(int(steps)): 
                self.actuator.stepper1.onestep(direction=stepper.BACKWARD)  # both coils are partially powered for smooth motion
                time.sleep(rotation_time/steps)
        else:
            print("Stepper motor not activated")

    def rotate_stepper_ccw(self,rotation_time):
        if self.stepper_motor_activated:
            stepper_speed = 90  # Desired speed in deg/s i.e. 15 RPM
            deg_per_step = 1.8 # Motor specification (200 steps/revolutions)
            deg = stepper_speed * rotation_time
            steps = deg/deg_per_step

            for i in range(int(steps)):
                self.actuator.stepper1.onestep(direction=stepper.FORWARD)  # both coils are partially powered for smooth motion
                time.sleep(rotation_time/steps)
        else:
            print("Stepper motor not activated")

    def linear_actuator_out(self,actuation_time):
        #self.actuator.motor3.throttle = self.throttle
        #time.sleep(1)
        if self.linear_actuator_activated:
            self.actuation_time = actuation_time
            if (self.linear_actuator_time_counter + self.actuation_time) <= self.linear_actuator_max_time:
                self.linear_actuator_time_counter = self.linear_actuator_time_counter + self.actuation_time
                print(self.linear_actuator_time_counter)
                self.actuator.motor3.throttle = self.throttle
                time.sleep(self.actuation_time)
                self.actuator.motor3.throttle = 0
            else:
                print("Specified time will cause the linear actuator to exceed maximum extension or the actuator is at maximum position")
        else:
            print("Linear actuator not activated")

    def linear_actuator_in(self,actuation_time):
        #self.actuator.motor3.throttle = -self.throttle
        #time.sleep(4)
        if self.linear_actuator_activated:
            self.actuation_time = actuation_time
            if (self.linear_actuator_time_counter - self.actuation_time) >= 0:
                self.linear_actuator_time_counter = self.linear_actuator_time_counter - self.actuation_time
                print(self.linear_actuator_time_counter)
                
                self.actuator.motor3.throttle = -self.throttle
                time.sleep(self.actuation_time)
                self.actuator.motor3.throttle = 0
            else:
                print("Specified actuation time will cause the linear actuator to exceed minimum extension or the actuator is at minimum position")
        else:
            print("Linear actuator not activated")
    #-------------------------------------------------------------------
    def map_value(self, x, in_min, in_max, out_min, out_max):
        return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min
    # ------------------------------------------------------------------------------------------------------------------------

    # Available sensors for activation
    # Note: All sensors take a bin_id input in order to get the properties of that particular bin (i.e. the target location)
    # This bin_id is an output from the load container function that they are responsible for developing.
    # To take a reading, you activate a sensor, take the reading, then deactivate the sensor.
    # Hall, IR, ldr, color and photoelectric sensors take a duration as well in order to put some randomness in their readings based on an interval (0.2) for the specified duration
    # They also have a range in which a high reading will be given. Ranges are summarized in the intialize method.
    # Students will need to calculate the average using the calc avg function they developed during the Lab.
    def activate_ultrasonic_sensor(self):
        #set GPIO direction (IN / OUT)
        GPIO.setup(GPIO_TRIGGER, GPIO.OUT)
        GPIO.setup(GPIO_ECHO, GPIO.IN)
        self.ultrasonic_sensor_activated = True
        print("Ultrasonic sensor activated")

    def deactivate_ultrasonic_sensor(self):
        self.ultrasonic_sensor_activated = False
        print("Ultrasonic sensor deactivated")

    # Outputs a distance from the specified bin from the qbot's bumper to the front face of the bin.
    def read_ultrasonic_sensor(self):
        if self.ultrasonic_sensor_activated == True:
            # set Trigger to HIGH
            GPIO.output(GPIO_TRIGGER, True)
         
            # set Trigger after 0.01ms to LOW
            time.sleep(0.00001)
            GPIO.output(GPIO_TRIGGER, False)
         
            StartTime = time.time()
            StopTime = time.time()
         
            # save StartTime
            while GPIO.input(GPIO_ECHO) == 0:
                StartTime = time.time()
         
            # save time of arrival
            while GPIO.input(GPIO_ECHO) == 1:
                StopTime = time.time()
         
            # time difference between start and arrival
            TimeElapsed = StopTime - StartTime
            # multiply with the sonic speed (34300 cm/s)
            # and divide by 2, because there and back
            distance = (TimeElapsed * 34300) / 2
         
            return round(distance,2)
        else:
            print("Ultrasonic sensor not activated")

    def activate_hall_sensor(self):
        
        self.hall_sensor_activated = True
        
        print("Hall sensor activated")

    def deactivate_hall_sensor(self):
        self.hall_sensor_activated = False
        print("Hall sensor deactivated")

    # Outputs high voltage readings for a specific duration if the specified bin is metallic and the box is within the sensor's range.
    def read_hall_sensor(self, duration):
        if self.hall_sensor_activated == True:
            reading = []
            elapsed_time = 0
            start_time = time.time()
            mcp = MCP3008(clk=CLK, cs=CS, miso=MISO, mosi=MOSI)
            while elapsed_time < duration:
                time.sleep(interval) 
                digital = mcp.read_adc(5)
                analog = 5-digital*5.0/1023.0
                reading.append(round(analog,1))
                elapsed_time = time.time() - start_time
            return reading
        else:
            print("Hall sensor not activated")

    def activate_ir_sensor(self):
        self.ir_sensor_activated = True
        print("Active IR sensor activated")

    def deactivate_ir_sensor(self):
        self.ir_sensor_activated = False
        print("Active IR sensor deactivated")

    # Outputs high voltage readings for a specific duration if the specified bin is within proximity to the QBot and the box is within sensor's range.
    def read_ir_sensor(self, duration):
        if self.ir_sensor_activated == True:
            reading = []
            elapsed_time = 0
            start_time = time.time()
            mcp = MCP3008(clk=CLK, cs=CS, miso=MISO, mosi=MOSI)
            while elapsed_time < duration:
                time.sleep(interval) 
                digital = mcp.read_adc(3)
                analog = digital*5.0/1023.0
                reading.append(round(analog,1))
                elapsed_time = time.time() - start_time
            return reading
        else:
            print("Active IR sensor not activated")

    def activate_ldr_sensor(self):
        self.ldr_sensor_activated = True
        print("LDR sensor activated")


    def deactivate_ldr_sensor(self):
        self.ldr_sensor_activated = False
        print("LDR sensor deactivated")

    # Outputs high voltage readings for a specific duration if light is sensed around the QBot.
    def read_ldr_sensor(self):
        if self.ldr_sensor_activated == True:
            GPIO.setup(GPIO_LDR, GPIO.OUT)
            GPIO.output(GPIO_LDR, GPIO.LOW)
            time.sleep(0.1)

            #Change the pin back to input
            GPIO.setup(GPIO_LDR, GPIO.IN)
            
            count = 0
            while (GPIO.input(GPIO_LDR) == GPIO.LOW):
                count += 1
            reading = self.map_value(count,800,1300,1,0)
            return round(reading,2)
        else:
            print("LDR sensor not activated")

    # Unlike the other activate sensor methods, here you have to specify the color you want to detect.
    def activate_color_sensor(self):
        GPIO.setup(GPIO_OUT,GPIO.IN, pull_up_down=GPIO.PUD_UP)
        GPIO.setup(GPIO_S2,GPIO.OUT)
        GPIO.setup(GPIO_S3,GPIO.OUT)
        self.color_sensor_activated = True

    def deactivate_color_sensor(self):
        self.color_sensor_activated = False
        print("Color sensor deactivated")

    # Outputs high voltage readings for a specific duration if the specified bin's color is red, and the box is within sensor's range.
    def read_color_sensor(self):
        if self.color_sensor_activated == True:
            
            NUM_CYCLES = 10
            GPIO.output(GPIO_S2,GPIO.LOW)
            GPIO.output(GPIO_S3,GPIO.LOW)
            time.sleep(0.3)
            start = time.time()
            for impulse_count in range(NUM_CYCLES):
              GPIO.wait_for_edge(GPIO_OUT, GPIO.FALLING)
            duration = time.time() - start      #seconds to run for loop
            red  = round((NUM_CYCLES / duration),2)   #in Hz
            red_RGB = round(self.map_value(red,200,600,0,1),1)

            GPIO.output(GPIO_S2,GPIO.HIGH)
            GPIO.output(GPIO_S3,GPIO.HIGH)
            time.sleep(0.3)
            start = time.time()
            for impulse_count in range(NUM_CYCLES):
              GPIO.wait_for_edge(GPIO_OUT, GPIO.FALLING)
            duration = time.time() - start
            green = round((NUM_CYCLES / duration),2)
            green_RGB = round(self.map_value(green,120,600,0,1),1)

            GPIO.output(GPIO_S2,GPIO.LOW)
            GPIO.output(GPIO_S3,GPIO.HIGH)
            time.sleep(0.3)
            start = time.time()
            for impulse_count in range(NUM_CYCLES):
              GPIO.wait_for_edge(GPIO_OUT, GPIO.FALLING)
            duration = time.time() - start
            blue = round((NUM_CYCLES / duration),2)
            blue_RGB = round(self.map_value(blue,160,800,0,1),1)

            return [red, green, blue],[red_RGB, green_RGB, blue_RGB]
                    
        else:
            print("Color sensor not activated")


