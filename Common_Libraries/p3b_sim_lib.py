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
import numpy as np
import time
import os
import math

from Common_Libraries.postman import postman
from Common_Libraries.modular_comm import comm_modular_container
# from Common_Libraries.quanser_sim_lib import rotarytable_sim, QArm_sim, QBot2e_sim, CameraUI
from Common_Libraries.quanser_sim_lib import *

from Common_Libraries.modular_comm import comm_modular_server
from array import *

import random

#initialize the QuanserSim environment

QIL = postman(18001) #Establish communication
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

class servo_table:
    def __init__(self):
        self.my_table = rotarytable_sim(QIL)

        # Intialize all variables
        self.table_weight = 0
        self.proximity = False

        self.obj_number = 0

        self.container = []
        self.container_color = ""
        self.container_mass = 0.0

        self.material = ""
        self.bin_id = ""

# -----------------------------------------------------------------------------------------------
# These functions may not be necessary since we are not controlling the table during these activities. Leave out of the documentation
    def rotate_table_speed(self, speed):
        if float(speed) >= 0.0 and float(speed) <= 2.0:
            print("I am here")
            self.my_table.rotate_clockwise(speed)
        elif float(speed) > 2.0:
            print("Input speed is too fast. Enter a speed less than 2 m/s")
        elif float(speed) < 0.0:
            print("Input a positive speed.")
        else:
            print("Invalid input.")

    def rotate_table_angle(self, deg):
        if deg < 0:
            print("Input a positive angle.")
        else:
            self.my_table.command_rel_position_pid(deg)

    def stop_table(self):
        self.my_table.stop_table()

    # Continuous communications with the simulation environment
    def ping(self):
        self.my_table.ping()

# -----------------------------------------------------------------------------------------------

    # Function that outputs bin ID and bottle mass based on the number they input.
    # 1 = non contaminated plastic
    # 2 = non-contaminated metal, 3 = non-contaminated paper, 4 = contaminated plastic, 5 = contaminated metal
    # 6 = contaminated paper

    def container_properties(self, rand_number):
        if int(rand_number) >= 1 and int(rand_number) <= 6:
            self.obj_number = rand_number - 1

            # non-contaminated containers - a list containing the color of the object and the mass in grams
            obj1 = ['clear', 'plastic', empty_plastic_container, "Bin03"]
            obj2 = ['red', 'metal', empty_metal_container, "Bin01"]
            obj3 = ['blue', 'paper', empty_paper_container, "Bin02"]

            # contaminated containers - a list containing the color of the object and the mass in grams
            obj4 = ['clear', 'plastic', empty_plastic_container + random.uniform(5.0, 50.0), "Bin04"]
            obj5 = ['red', 'metal', empty_metal_container + random.uniform(5.0, 50.0), "Bin01"]
            obj6 = ['blue', 'paper', empty_paper_container + random.uniform(5.0, 50.0), "Bin04"]

            self.container = [obj1, obj2, obj3, obj4, obj5, obj6]
            self.material = self.container[self.obj_number][1]

            self.container_mass = round(self.container[self.obj_number][2],2)
            self.bin_id = self.container[self.obj_number][3]  # Added to output the bin ID.
            return self.material, self.container_mass, self.bin_id
        else:
            print("Enter a number between 1 and 6 (inclusive).")

    # dispense the bottle. Note: the container_properties method above needs to be called first to update the results
    def dispense_container(self):
        self.container_color = self.container[self.obj_number][0]

        if self.container_color == "red":
            color = [1, 0, 0]
        elif self.container_color == "clear":  # used to be "green"
            # color = [0,1,0]
            color = [0.86, 0.94, 0.94]
        elif self.container_color == "blue":
            color = [0, 0, 1]

        self.my_table.spawn_single_bottle(color, self.container_mass, self.material)
        time.sleep(1)

class qarm:
    def __init__(self):
        self.my_qarm = QArm_sim(QIL)
        self.tolerance = 0.01

# -----------------------------------------------------------------------------------------------
# Not necessarily useful to students. Leave it out of the documentation

    # Used to establish continuous communication
    def ping(self):
        self.my_qarm.ping()

# -----------------------------------------------------------------------------------------------

    def effector_position(self):
        x_pos, y_pos, z_pos = self.my_qarm.qarm_forward_kinematics(self.b, self.s, self.e, self.w)
        return x_pos, y_pos, z_pos

    def home(self):
        self.my_qarm.qarm_move(0, 0, 0, 0, 0, True, self.tolerance)
        self.b, self.s, self.e, self.w, self.g = 0, 0, 0, 0, 0
        time.sleep(0.1)

    def rotate_base(self, deg):
        b = self.b + math.radians(deg)
        if abs(b) > math.radians(175):
            print("Invalid Angle. Base does not rotate beyond +/- 175 degrees")
        else:
            self.b = b
            self.my_qarm.qarm_move_base(self.b, True, self.tolerance)

    def rotate_shoulder(self, deg):
        s = self.s + math.radians(deg)
        if abs(s) > math.radians(90):
            print("Invalid Angle. Shoulder does not rotate beyond +/- 90 degrees")
        else:
            self.s = s
            self.my_qarm.qarm_move_shoulder(self.s, True, self.tolerance)

    def rotate_elbow(self, deg):
        e = self.e + math.radians(deg)
        if e > math.radians(90) or e < math.radians(-80):
            print("Invalid Angle. Elbow does not rotate beyond +90 or -80 degrees")
        else:
            self.e = e
            self.my_qarm.qarm_move_elbow(self.e, True, self.tolerance)

    def rotate_wrist(self, deg):
        w = self.w + math.radians(deg)
        if abs(w) > math.radians(170):
            print("Invalid Angle. Wrist does not rotate beyond +/- 170 degrees")
        else:
            self.w = w
        self.my_qarm.qarm_move_wrist(self.w, True, self.tolerance)

    def control_gripper(self, value):
        if abs(value) <= 55 and math.degrees(self.g + math.radians(value)) >= 0 and math.degrees(
                self.g + math.radians(value)) < 56:
            self.g = self.g + math.radians(value)
            self.my_qarm.qarm_move_gripper(self.g, True, self.tolerance)

    def move_arm(self, x, y, z):
        self.b, self.s, self.e = self.my_qarm.qarm_inverse_kinematics(x, y, z)
        self.my_qarm.qarm_move(self.b, self.s, self.e, self.w, self.g, True, self.tolerance)

class qbot:
    def __init__(self, speed):
        self.bot = QBot2e_sim(QIL)#Initialize qbot sim
        self.max_speed = 100
        self.speed = speed
        self.turn = 0

        # Camera image number
        self.camera_image = 0.0

        # activated actuator
        self.stepper_motor_activated = False
        self.linear_actuator_activated = False

        # activated Sensors
        self.ultrasonic_sensor_activated = False
        self.hall_sensor_activated = False
        self.ir_sensor_activated = False
        self.ldr_sensor_activated = False

        self.color_sensor_activated = False

        self.photoelectric_sensor_activated = False

        # Sensor ranges.
        self.ultrasonic_sensor_range = 2.5 # Adafruit HC-SR04, source = datasheet. 250 cm
        self.hall_sensor_range = 0.25 # 25 cm
        self.ir_sensor_range = 0.25 # 25 cm
        self.ldr_sensor_range = 0.25 # 25 cm
        self.color_sensor_range = 0.25 # 25 cm
        self.photoelectric_sensor_range = 0.25 # 25 cm

        # initialize variable for line following capabilities
        self.lost_line = 0

    def forward_time(self, time):
        # Convert speed and turn to wheel speeds and command QBot
        delta = self.turn * wheel_to_wheel_distance
        left = self.speed - delta
        right = self.speed + delta
        velocity = [left, right]  # velocity for the left and right wheel
        self.bot.move_time(velocity, time)

    def forward_distance(self, distance):
        # Convert speed and turn to wheel speeds and command QBot
        delta = self.turn * wheel_to_wheel_distance
        left = self.speed - delta
        right = self.speed + delta
        velocity = [left, right]  # velocity for the left and right wheel
        time = distance / self.speed
        self.bot.move_time(velocity, time)

    def travel_forward(self, threshold):
        # Initial depth measurement in meters
        d = round(self.bot.measure_depth(row, col) - camera_bumper_depth, 3)

        # Continue to drive until threshold is reached
        while threshold < d:
            # Drive
            self.bot.set_velocity([self.speed, self.speed])

            print("Depth (m): ", d)
            d = round(self.bot.measure_depth(row, col) - camera_bumper_depth, 3)
            # time.sleep(0.05)

        # Stop QBot
        self.bot.halt()

    def rotate(self, degree):
        time = 1.0
        rad = math.radians(degree)
        speed = ((wheel_to_wheel_distance / 2.0) * (rad / time))
        velocity = [speed, -speed]
        self.bot.move_time(velocity, time)

        # Read and return how far the Q-Bot's bumper is from an object e.g. the walls. Suggest to start with camera at 0 degrees and increment if
        # a non-sensical reading is output.

    def depth(self):
        value = self.bot.measure_depth(row, col) - camera_bumper_depth
        return round(value, 3)

# -----------------------------------------------------------------------------------------------
# Not necessarily useful to students. Leave it out of the documentation

    # Used to establish continuous communication. Should only be used in the simulation
    def ping(self):
        self.bot.ping()

# -----------------------------------------------------------------------------------------------

    # Takes an input file from the modelling sub-team that contains time and angle data.
    # Recommended file type is a .txt file without headers i.e. string characters identifying
    # the time and angle columns. It is assumed that the first column is time (s) and the
    # second column is the angle (deg)
    def process_file(self,filename):
        rotation_time=[]
        rotation=[]
        with open(filename,"r") as f:
            for line in f:
                line = line.strip()
                line_pair = line.split("\t") #assuming translation and rotation coordinates are spearated by \t

                rotation_time.append(float(line_pair[0]))
                rotation.append(float(line_pair[1]))
        return rotation_time,rotation

# -----------------------------------------------------------------------------------------------
# Used for internal calculations to reset the box's position and rotation.
# Do not include in the library documentation
    def reset_box(self):
        self.x,self.y,self.z,self.x_r,self.y_r,self.z_r = [0]*6
        self.bot._set_box_attitude()

# -----------------------------------------------------------------------------------------------
    # Activates the stepper motor
    def activate_stepper_motor(self):
        self.stepper_motor_activated = True
        self.reset_box()
        print("Actuator activated.")

    # Deactivates the stepper motor
    def deactivate_stepper_motor(self):
        self.stepper_motor_activated = False
        self.reset_box()
        print("Actuator deactivated.")

    # Activates the linear actuator
    def activate_linear_actuator(self):
        self.linear_actuator_activated = True
        self.reset_box()
        print("Actuator activated.")

    # Deactivates the linear actuator
    def deactivate_linear_actuator(self):
        self.linear_actuator_activated = False
        self.reset_box()
        print("Actuator deactivated.")

    # Rotates the container box angle to a specified angle position (Enter positive angles only)
    def rotate_box_cw(self, angle):
        if self.linear_actuator_activated is not self.stepper_motor_activated:
            if float(angle) >= 0.0 and float(angle) <= float(120):
                theta = math.radians(angle)
                self.bot._set_box_angle(theta)
            elif float(angle) < float(0):
                print("Enter a positive angle.")
            elif float(angle) > float(120):
                print("Angle is greater than 120 degrees")
            else:
                print("Invalid angle.")
        elif self.linear_actuator_activated is self.stepper_motor_activated:
            print("Both actuators are activated. Deactivate one.")
        else:
            print("Actuators is not activated.")

    # Rotates the container box angle to a specified angle position (Enter positive angles only)
    def rotate_box_ccw(self, angle):
        if self.linear_actuator_activated is not self.stepper_motor_activated:
            if float(angle) > 0.0 and float(angle) <= float(120):
                theta = math.radians(angle)
                self.bot._set_box_angle(theta)
            elif float(angle) == float(0):
                print("Box is at the minimum angle. Cannot rotate counterclockwise.")
            elif float(angle) < float(0):
                print("Enter a positive angle.")
            elif float(angle) > float(120):
                print("Angle is greater than 120 degrees")
            else:
                print("Invalid angle.")
        elif self.linear_actuator_activated is self.stepper_motor_activated:
            print("Both actuators are activated. Deactivate one.")
        else:
            print("Actuators is not activated.")

    # Dumps the containers along a generic pre-defined motion.
    def dump(self):
        if self.actuator_activated == True:
            dump = self.bot.dump()
            return dump
        else:
            print("Actuator is not activated.")

    def stop(self):
        self.bot.halt()

    # Gets the positon of the Qbot in the environment (towards the bins is y, towards the arm is x).
    def position(self):
        [self.bot_position_x, self.bot_position_y, self.bot_position_z] = self.bot.get_position()
        self.bot_position_x = round(self.bot_position_x,2)
        self.bot_position_y = round(self.bot_position_y,2)
        self.bot_position_z = round(self.bot_position_z,2)
        return self.bot_position_x, self.bot_position_y, self.bot_position_z

# ------------------------------------------------------------------------------------------------------------------------
    # DON'T INCLUDE IN THE LIBRARY DESCRIPTION.

    # Used to calculate the length, which is used to evaluate the distance
    def dotproduct(self, v1, v2):
        return sum((a * b) for a, b in zip(v1, v2))

    # Same as above
    def length(self,v):
        return math.sqrt(self.dotproduct(v, v))

    # Used to randomize the readings by adding simulated noise.i.e. Readings are taken at time interaval (0.2) for a specified duration.
    # Students will need to calculate the average reading.
    def sensor_readings(self,duration, lower_limit, upper_limit):
        reading = []
        elapsed_time = 0
        start_time = time.time()
        while elapsed_time < duration:
            reading.append(random.uniform(lower_limit, upper_limit))
            time.sleep(interval)
            elapsed_time = time.time() - start_time
        return reading

    # Used in the ultrasonic sensor method to calculate the distance; and in the hall, and IR sensor for range verification.
    def box_to_bin_distance(self,bin_id):
        [self.bot_position_x, self.bot_position_y,self.bot_position_z] = self.bot.get_position()  # Get intial bot reading. Always needs to be run or you would get a 0,0,0 reading.

        self.bin = bins()

        [self.bin_position_x, self.bin_position_y, self.bin_position_z] = self.bin.bin_position(bin_id)
        [self.bot_position_x, self.bot_position_y, self.bot_position_z] = self.bot.get_position()

        qbot_radius = 0.5 * wheel_to_wheel_distance
        bin_length = 0.3 # Verify with Quanser
        offset = 0.07 # The position of the bins from the centre of the yellow line is not exactly 0. It is 2 cm (1/2 thickness of the line) + 5 cm offset.

        distance = self.length([(self.bot_position_x - self.bin_position_x),(self.bot_position_y - self.bin_position_y )]) - 0.5*bin_length - offset - qbot_radius

        return distance

    #Used to map values from one range to another
    def map_value(self, x, in_min, in_max, out_min, out_max):
        return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min

    #Finds the bin closest to the qbot for the sensor to read
    def closest_bin(self):
        distances = []
        for i in range(1,5):
            bin_id = "Bin0"+str(i)
            distances.append(self.box_to_bin_distance(bin_id))
        ind = distances.index(min(distances))
        closest = "Bin0"+str(ind+1)
        return closest
            
# ------------------------------------------------------------------------------------------------------------------------

# Available sensors for activation
# Note: All sensors take a bin_id input in order to get the properties of that particular bin (i.e. the target location)
# This bin_id is an output from the load container function that they are responsible for developing.
# To take a reading, you activate a sensor, take the reading, then deactivate the sensor.
# Hall, IR, ldr, color and photoelectric sensors take a duration as well in order to put some randomness in their readings based on an interval (0.2) for the specified duration
# They also have a range in which a high reading will be given. Ranges are summarized in the intialize method.
# Students will need to calculate the average using the calc avg function they developed during the Lab.
    def activate_ultrasonic_sensor(self):
        self.ultrasonic_sensor_activated = True
        print("Ultrasonic sensor activated")

    def deactivate_ultrasonic_sensor(self):
        self.ultrasonic_sensor_activated = False
        print("Ultrasonic sensor deactivated")

    # Outputs a distance from the specified bin from the qbot's bumper to the front face of the bin.
    def read_ultrasonic_sensor(self):
        if self.ultrasonic_sensor_activated == True:
            bin_id = self.closest_bin()
            distance = self.box_to_bin_distance(bin_id)
            if distance <= self.ultrasonic_sensor_range:
                return round(distance,2)
            else:
                return 0
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
            bin_id = self.closest_bin()
            distance = self.box_to_bin_distance(bin_id)
            if distance <= self.hall_sensor_range:
                [self.bot_position_x, self.bot_position_y,self.bot_position_z] = self.bot.get_position()  # Get intial bot reading. Always needs to be run or you would get a 0,0,0 reading.
                self.bin = bins()

                r, g, b, self.metallic, roughness = self.bin.bin_properties(bin_id)

                if self.metallic != 0:#Hall sensor maxes out around 2.5V 
                    reading = self.sensor_readings(duration, 2.1, 2.5)
                else:
                    reading = self.sensor_readings(duration, 1.8, 2.0)
                    
            else:
                reading = self.sensor_readings(duration, 1.8, 2.0)
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
            bin_id = self.closest_bin()
            distance = self.box_to_bin_distance(bin_id)
            if distance <= self.ir_sensor_range:
                reading = self.sensor_readings(duration,4.5,5.0)
            else:
                reading = self.sensor_readings(duration,0.0,0.4)
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
    def read_ldr_sensor(self, duration):
        if self.ldr_sensor_activated == True:
            bin_id = self.closest_bin()
            distance = self.box_to_bin_distance(bin_id)
            if distance <= self.ldr_sensor_range:
                reading = random.uniform(0.6,1)
            else:
                reading = random.uniform(0.0,0.4)
            return round(reading,2)
        else:
            print("LDR sensor not activated")

    # Unlike the other activate sensor methods, here you have to specify the color you want to detect.
    def activate_color_sensor(self):
        self.color_sensor_activated = True
        print("Color sensor activated")
        
    def deactivate_color_sensor(self):
        self.color_sensor_activated = False
        print("Color sensor deactivated")

    # Outputs high voltage readings for a specific duration if the specified bin's color is red, and the box is within sensor's range.
    def read_color_sensor(self):
        if self.color_sensor_activated == True:
            bin_id = self.closest_bin()
            distance = self.box_to_bin_distance(bin_id)
            red_min = random.uniform(200, 250)
            red_max = random.uniform(600, 650)
            green_min = random.uniform(110,130)
            green_max = random.uniform(600, 650)
            blue_min = random.uniform(150,160)
            blue_max = random.uniform(800,850)
            if distance <= self.color_sensor_range:
                self.bin = bins()
                self.r, self.g, self.b, metallic_property, roughness = self.bin.bin_properties(bin_id)
                red = round(self.map_value(self.r,0,1,red_min,red_max),1)
                green = round(self.map_value(self.g,0,1,green_min,green_max),1)
                blue = round(self.map_value(self.b,0,1,blue_min,blue_max),1)
                reading = [round(self.r,2), round(self.g,2), round(self.b,2)],[red,green,blue]
            else:
                reading = [0,0,0],[round(red_min,2),round(green_min,2),round(blue_min,2)]
            return reading
        else:
            print("Color sensor not activated")

    
    # Returns IR sensor readings at the bottom of the Q-Bot. 1 = yellow line reading, 0 = no yellow line readings
    # Use readigns to create a follow line function. Suggest to position the camera at 21.5 degrees.
##    def line_following_sensors(self):
##        self.img_RGB = self.bot.get_new_RGB()
##
##        self.img_binary = crop_rect(self.img_RGB,[288,304],[448,464])
##
##        left_color = self.img_binary[8,4] #BGR color
##        right_color = self.img_binary[8,12] #BGR color
##
##        # Sensor readings
##        if left_color[1] >= 220 and left_color[2] >= 220: # for a yellow line
##            left_ir_sensor_reading = 1 # reflects the light
##        else:
##            left_ir_sensor_reading = 0
##
##        if right_color[1] >= 220 and right_color[2] >= 220: # for a yellow line
##            right_ir_sensor_reading = 1 # reflects the light
##        else:
##            right_ir_sensor_reading = 0
##
##        ir_reading = [left_ir_sensor_reading,right_ir_sensor_reading]
 
        
    def line_following_sensors(self):
        # Initialize camera
        #self.camera_image = CameraUI()

        image_buffer = self.bot.get_new_RGB()
        line_ctr = self.camera_image.process(image_buffer)
        max_speed = 1
        qbot_speed = 0
        turn = 0
        #print(line_ctr)

        if line_ctr != -1:
            self.lost_line = 0

            # Normalize the position of the line in the range (-1, 1)
            err = (320 - line_ctr) / 320 # Takes the curve path around the track

            # Calculate the offset for turning
            turn = err * 0.5
            # Slow down as the line approaches the edge of the FOV
            qbot_speed = max_speed * (1 - abs(err))

        else:
            # Stop the robot if the line is not found for 5 frames
            self.lost_line += 1
            if self.lost_line > 5 and max_speed != 0 :
                self.stop()
                print("Cannot find line, QBot stopped...")
                return [0,0]

        delta = turn * 0.235  # QBOT_DIAMETER
        left = qbot_speed - delta
        right = qbot_speed + delta
        #print(left,right)

        if abs(left-right)<=0.02 and left!=0 and right!=0:
            ir_reading = [1,1]
        elif left>right:
            ir_reading = [0,1]
        elif right>left:
            ir_reading = [1,0]
        else:
            ir_reading = [0,0]
        
        return ir_reading

    # Sets the speed of the left and right wheel. inputer is a list containing the left wheel speed at index 0 and right wheel speed at index 1.
    def set_wheel_speed(self,speeds):
        if type(speeds) == list:
            self.bot.set_velocity(speeds)
        else:
            print("Invalid input. Enter the left and right wheel speed as a list. e.g. [left_speed, right_speed].")

    def initialize_camera(self):
        self.camera_image = CameraUI()

        self.img_RGB = np.zeros((480,640,3), dtype = np.uint8) #initial image
        self.img_binary = crop_rect(self.img_RGB,[280,313],[448,480])
        #self.show_camera_image()

    def stop_camera(self):
        self.camera_image.destroy()

    def show_camera_image(self):
         cv2.startWindowThread()
         cv2.namedWindow('Camera Image 33 x 32', cv2.WINDOW_AUTOSIZE)
         cv2.imshow('Camera Image 33 x 32', self.img_binary)

    def follow_line(self,speed):
        # Initialize camera
        #self.camera_image = CameraUI()

        image_buffer = self.bot.get_new_RGB()
        line_ctr = self.camera_image.process(image_buffer)
        max_speed = speed
        qbot_speed = 0
        turn = 0
        #print(line_ctr)

        if line_ctr != -1:
            self.lost_line = 0

            # Normalize the position of the line in the range (-1, 1)
            err = (320 - line_ctr) / 320 # Takes the curve path around the track

            # Calculate the offset for turning
            turn = err * 0.5
            # Slow down as the line approaches the edge of the FOV
            qbot_speed = max_speed * (1 - abs(err))

        else:
            # Stop the robot if the line is not found for 5 frames
            self.lost_line += 1
            if self.lost_line > 5 and max_speed != 0 :
                self.stop()
                print("Cannot find line, QBot stopped...")

        delta = turn * 0.235  # QBOT_DIAMETER
        left = qbot_speed - delta
        right = qbot_speed + delta
        velocity = [left,right]
        #self.bot.set_velocity(velocity)
        return self.lost_line, velocity
# -----------------------------------------------------------------------------------------------
# Class used for internal use only. It was used to be used in conjuction with the sensors on the Q-Bot.
# Leave it out of the documentation

class bins:
    def __init__(self):

        self.metal_bin = smartbox_sim(QIL, 1)
        self.paper_bin = smartbox_sim(QIL, 2)
        self.plastic_bin = smartbox_sim(QIL, 3)
        self.garbage_bin = smartbox_sim(QIL, 4)

    # Returns the position of the specified bin.
    def bin_position(self,bin_id):
        if bin_id == "Bin01":
            [self.bin_position_x, self.bin_position_y, self.bin_position_z] = self.metal_bin.get_position()
        elif bin_id == "Bin02":
            [self.bin_position_x, self.bin_position_y, self.bin_position_z] = self.paper_bin.get_position()
        elif bin_id == "Bin03":
            [self.bin_position_x, self.bin_position_y, self.bin_position_z] = self.plastic_bin.get_position()
        elif bin_id == "Bin04":
            [self.bin_position_x, self.bin_position_y, self.bin_position_z] = self.garbage_bin.get_position()

        #print(self.bin_position_x, self.bin_position_y, self.bin_position_z)
        return self.bin_position_x, self.bin_position_y, self.bin_position_z

    # Returns the properties of the specified bin.
    def bin_properties(self,bin_id):
        if bin_id == "Bin01":
            self.r, self.g, self.b, self.metallic, self.roughness = self.metal_bin.get_surface_properties()
        elif bin_id == "Bin02":
            self.r, self.g, self.b, self.metallic, self.roughness = self.paper_bin.get_surface_properties()
        elif bin_id == "Bin03":
            self.r, self.g, self.b, self.metallic, self.roughness = self.plastic_bin.get_surface_properties()
        elif bin_id == "Bin04":
            self.r, self.g, self.b, self.metallic, self.roughness = self.garbage_bin.get_surface_properties()

        return self.r, self.g, self.b, self.metallic, self.roughness

# -----------------------------------------------------------------------------------------------

