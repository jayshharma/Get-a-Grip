# This code represents the library that students will reference to complete activities in for Project 0
#
# All code is currently set up to run on a raspberry pi ONLY.

import math
import sys
import time
sys.path.append('../')

from Common_Libraries.QBot2e_Lib import *
from Common_Libraries.quanser_image_lib import * # Has CV and numpy imported already

from Common_Libraries.ultrasonic_calibration import *

wheel_to_wheel_distance = 0.235
camera_bumper_depth = 0.156
row = 385
col = 319

import RPi.GPIO as GPIO
import board

# Ultrasonic Sensor
GPIO_TRIGGER = 23 #8
GPIO_ECHO = 24 #25

# Connections to ADC for the IR Sensor
from Adafruit_MCP3008 import MCP3008
CLK = 11
MISO = 9
MOSI = 10
CS = 22

class qbot:
 
    def __init__(self, speed):
        self.bot = QBot2e() # Initialize qbot hardware
        self.bot.reset()
        
        self.max_speed = 100
        self.speed = speed
        self.turn = 0

        GPIO.setup(GPIO_TRIGGER, GPIO.OUT)
        GPIO.setup(GPIO_ECHO, GPIO.IN)
    
    # Moves the bot for the specified time in seconds.    
    def forward_time(self, time):
        distance = self.speed * time
        self.bot.move_time(distance,0,time)

    # Moves the bot for the specified distance in meters.
    def forward_distance(self,distance):
            time = distance/self.speed
            self.bot.move_time(distance,0,time)

    # Moves the bot until the specified distance in meters from an object is reached i.e. the threshold.
    # Distance is measured from the bot's bumper to the object within vicinity.
    def travel_forward(self,threshold):
        d = round(self.depth(),3) # initial depth measurement in meters. From bumper to measured object.
        #print(d)
        if d >= 0.125 and threshold >= 0.125:
            
            while threshold < d:
                self.set_wheel_speed([self.speed,self.speed])
                print("Depth (m): ", d)
                d = round(self.depth(),3)

            # Sto QBot
            self.bot.halt()
            print("Depth (m): ",d)
        else:
            self.bot.halt() # unnecessary, but just in case.
            print("Threshold will cause the bot to crash into the object.")

    # Rotates the bot by the specified degrees clockwise.
    def rotate(self, degree):
        time = 1.0
        rad = math.radians(-degree) # To ensure clockwise rotation for a positive degree input similar to the simulation.
        self.bot.move_time(0,rad,time)
        
    # Returns the values of the left and right IR sensors connected to the bot.
    def line_following_sensors(self):
        mcp = MCP3008(clk=CLK, cs=CS, miso=MISO, mosi=MOSI)
        left_ir_analog = mcp.read_adc(4)
        right_ir_analog = mcp.read_adc(3)
        ir_reading = [left_ir_analog, right_ir_analog]
        return ir_reading       

    # Returns the distance between the bumper and object in meters
    def depth(self):
        # Set Trig to Low to wait for the sensor to settle.
        #GPIO.output(GPIO_TRIGGER, False)
        #time.sleep(2)
        
        # Set Trigger to HIGH
        GPIO.output(GPIO_TRIGGER, True)

        # Set Trigger after 0.01ms to LOW
        time.sleep(0.00001)
        GPIO.output(GPIO_TRIGGER, False)

        StartTime = time.time()
        StopTime = time.time()

        # Save StartTime
        while GPIO.input(GPIO_ECHO) == 0:
            startTime = time.time()

        # Save time of arrival
        while GPIO.input(GPIO_ECHO) == 1:
            StopTime = time.time()

        # Time difference between start and arrival time
        TimeElapsed = StopTime - StartTime
        #Multiply with the sonic speed (34300 cm/s) and divide by 2, because it's there and back
        distance = (((TimeElapsed * 34300)/2.0) - us_sensor_offset + 7.4)/100 # Distance in meters
        # Not sure why there is a 7.3 cm error but it's there. Test for consistency with other QBots used in this activity.
        # Added 7.4 cm b/c it is the rough distance between the ultrasonic sensor and the bumper.
        return round(distance,3)

   # Sets the speed of the left and right wheel. input is a list containing the left wheel speed at index 0 and right wheel speed at index 1.
    def set_wheel_speed(self,speeds):
        if type(speeds) == list:
            self.bot.set_velocity([speeds[1],speeds[0]]) # This is done such that the referenced wheels are the same in the simulation and hardware
        else:
            print("Invalid input. Enter the left and right wheel speed as a list. e.g. [left_speed, right_speed].")

    def stop(self):
        self.bot.halt()
