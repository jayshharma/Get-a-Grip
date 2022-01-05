# This code represents the library that students will reference to complete activities in for Project 3
#
# All code is currently set up to run on a raspberry pi ONLY.

import math
import sys
import time
sys.path.append('../')

from Common_Libraries.quanser_sim_lib import QBot2e_sim, CameraUI
from Common_Libraries.postman import postman
from Common_Libraries.modular_comm import comm_modular_container

from Common_Libraries.quanser_image_lib import * # Has CV and nunmpy imported already

import random

wheel_to_wheel_distance = 0.235
camera_bumper_depth = 0.156
row = 385
col = 319

class qbot:
    def __init__(self, speed):
        QIL = postman(18001)
        comc = comm_modular_container()

        self.bot = QBot2e_sim(QIL, 0) #Initialize qbot sim
            
        self.max_speed = 100
        self.speed = speed
        self.turn = 0

        # Initialize camera for line following
        # Ensure that the default image file is in the same location of both the library and the file that will be run.
        self.camera_image = CameraUI()

        self.img_RGB = np.zeros((480,640,3), dtype = np.uint8) #initial image
        self.img_binary = crop_rect(self.img_RGB,[280,313],[448,480])

    # Moves the bot for the specified time in seconds.
    def forward_time(self, time):
        #Convert speed and turn to wheel speeds and command QBot
        delta = self.turn * wheel_to_wheel_distance
        left = self.speed - delta
        right = self.speed + delta
        velocity = [left,right] # velocity for the left and right wheel
        self.bot.move_time(velocity,time)

    # Moves the bot for the specified distance in meters.
    def forward_distance(self,distance):
        # Convert speed and turn to wheel speeds and command QBot
        delta = self.turn * wheel_to_wheel_distance
        left = self.speed - delta
        right = self.speed + delta
        velocity = [left, right]  # velocity for the left and right wheel
        time = distance/self.speed
        self.bot.move_time(velocity, time)

    # Moves the bot until the specified distance in meters from an object is reached i.e. the threshold.
    # Distance is measured from the bot's bumper to the object within vicinity.
    # Recommend to increase wall height to 22 cm.
    def travel_forward(self,threshold):
        
        d = round(self.bot.measure_depth(row, col) - camera_bumper_depth,3) # Initial depth measurement in meters. From bumper to measured object
        if d >= 0.125 and threshold >= 0.125:        
            # Continue to drive until threshold is reached
            while threshold < d:
                # Drive 
                self.bot.set_velocity([self.speed,self.speed])

                print ("Depth (m): ", d)
                d = round(self.bot.measure_depth(row, col) - camera_bumper_depth,3)
                #time.sleep(0.05)

            # Stop QBot
            self.bot.halt()
            print("Depth (m): ",d)
        else:
            self.bot.halt() # unnecessary, but just in case.
            print("Threshold will cause the bot to crash into the object.")        

    # Rotates the bot by the specified degrees clockwise.
    def rotate(self, degree):
        time = 1.0        
        rad = math.radians(degree)
        speed = ((wheel_to_wheel_distance / 2.0) * (rad / time))
        velocity = [speed, -speed]
        self.bot.move_time(velocity,time)

    # Returns the values of the left and right IR sensors.
    # Must angle the camera to -21.5 degrees
    def line_following_sensors(self):
        self.img_RGB = self.bot.get_new_RGB()

        self.img_binary = crop_rect(self.img_RGB,[280,313],[448,480])

        left_color = self.img_binary[10,0] #BGR color
        right_color = self.img_binary[10,32] #BGR color

        # Sensor readings
        if left_color[1] >= 220 and left_color[2] >= 220: # for a yellow line
            left_ir_sensor_reading = int(random.uniform(150,153)) # reflects the light. Based on values retrived from physical ir sensors.
        else:
            left_ir_sensor_reading = int(random.uniform(1022,1023))

        if right_color[1] >= 220 and right_color[2] >= 220: # for a yellow line
            right_ir_sensor_reading = int(random.uniform(150,152)) # reflects the light. Based on values retrived from physical ir sensors.
        else:
            right_ir_sensor_reading = int(random.uniform(1022,1023))

        ir_reading = [left_ir_sensor_reading,right_ir_sensor_reading]
        return ir_reading

    # Read and return how far the Q-Bot's bumper is from an object in meters e.g. the walls.
    # Suggest to start with camera at 0 degrees and decrease if a non-sensical reading is output.
    def depth(self):
        value = self.bot.measure_depth(row,col) - camera_bumper_depth
        return round(value,3)

    # Sets the speed of the left and right wheel. inputer is a list containing the left wheel speed at index 0 and right wheel speed at index 1.
    def set_wheel_speed(self,speeds):
        if type(speeds) == list:
            self.bot.set_velocity(speeds)
        else:
            print("Invalid input. Enter the left and right wheel speed as a list. e.g. [left_speed, right_speed].")

    def stop(self):
        self.bot.halt()
        
    # Used to establish continuous communication. Should only be used in the simulation
    def ping(self):
        self.bot.ping()
