# This code represents the library that students will reference to run the Q-Arm
# simulation for Project 2.
#
# Current code is set up to run on a raspberry pi but simple modification are required to
# allow for the code to run on a computer. A couple libraries will need to be installed
# here in order to prevent the students from having to install them.
#
# **** Note: After installing all the libraries on computer for a computer test run, an
# error was generated. So far, guaranteed execution is only when a Raspberry Pi is used.**** 

# Items to mention to students
# 1. Rotation limits base +/- 175 deg, shoulder +/- 90 deg, elbow +90 -80 deg, wrist +/-170, gripper 0(open)-1(close)
# 2. For the autoclaves, 0 = closed >0 = open

# Import all required libraries
import sys

sys.path.append('../')
import numpy as np
import time

import math

from Common_Libraries.qarm_lib import QArm, saturate
np.set_printoptions(formatter={'float': lambda x: "{0:0.2f}".format(x)})

actuator_enable = 4
actuator_input1 = 2
actuator_input2 = 3

# Imports for the EMG Sensors
import RPi.GPIO as GPIO
from Adafruit_MCP3008 import MCP3008

CLK = 11
MISO = 9
MOSI = 10
CS = 22

class qarm:
    # Manipulator parameters in meters:
    _L1 = 0.127
    _L2 = 0.3556
    _L3 = 0.4064

    def __init__(self):

        self.my_qarm = QArm()
        time.sleep(1)

        self.my_qarm.write_LEDs() # set the base led color.

        # Autoclave activation
        self.autoclaves_activated = False

        # EMG sensor activation
        self.my_emg_activated = False

        self.b, self.s, self.e, self.w, self.g = 0, 0, 0, 0, 0
        self.home()

        self.rotate_shoulder_tracker = 0

    def effector_position(self):
        self.my_qarm.read_config() # Updates the configurations based on the arms current position
        x_pos, y_pos, z_pos = self.arm_forward_kinematics(self.b, self.s, self.e, self.w)
        return round(x_pos,3), round(y_pos,3), round(z_pos,3)

    def home(self):
        self.move_arm_intermediate()
        self.rotate_wrist(math.degrees(-self.w))
        self.control_gripper(-self.g*30.0/0.9)

        self.b, self.s, self.e, self.w, self.g = 0, 0, 0, 0, 0.0
        time.sleep(0.1)
        self.my_qarm.read_config() # update the configuration

        self.rotate_shoulder_tracker = 0

    # Rotate Joints
    def rotate_base(self, deg):
        b = self.b + math.radians(deg)
        if abs(b) > math.radians(176): # added one degree to deal with rounding error that shows up in forward and inverse kinematic calculations.
            print("Invalid Angle. Base does not rotate beyond +/- 175 degrees")
        else:
            self.b = b
            self.my_qarm.goto_config(self.b, self.s, self.e, self.w, self.g)
            time.sleep(0.1)
            self.my_qarm.read_config()
        #print(self.b)
        
    def rotate_shoulder(self, deg):
        s = self.s + math.radians(deg)
        if s > math.radians(51) or s < math.radians(-91):
            print("Invalid Angle. Shoulder does not rotate beyond +50 or -90 degrees")
        else:
            self.s = s
            self.my_qarm.goto_config(self.b, self.s, self.e, self.w, self.g)
            time.sleep(0.1)
            self.my_qarm.read_config()

    def rotate_elbow(self, deg):
        e = self.e + math.radians(deg)
        if e > math.radians(91) or e < math.radians(-81):
            print("Invalid Angle. Elbow does not rotate beyond +90 or -80 degrees")
        else:
            self.e = e
            self.my_qarm.goto_config(self.b, self.s, self.e, self.w, self.g)
            time.sleep(0.1)
            self.my_qarm.read_config()

    def rotate_wrist(self, deg):
        w = self.w + math.radians(deg)
        if abs(w) > math.radians(171):
            print("Invalid Angle. Wrist does not rotate beyond +/- 170 degrees")
        else:
            self.w = w
            self.my_qarm.goto_config(self.b, self.s, self.e, self.w, self.g)
            time.sleep(0.1)
            self.my_qarm.read_config()

    # Control Gripper. Gripper moves between 0 - 30 degrees
    def control_gripper(self, deg):
        if abs(deg) <= 30 and ((self.g*30/0.9) + deg) >= 0 and ((self.g*30/0.9) + deg) < 31:
            self.g = self.g + ((0.9*deg)/30.0) # Conversion since the simulation uses degrees bu the hardware uses a number between 0 and 1
            self.my_qarm.goto_config(self.b, self.s, self.e, self.w, self.g)
            time.sleep(0.1)
            self.my_qarm.read_config()
        else:
            print("Please enter a value in between +/-30 degrees")

    # Open / Close the Autoclave. Takes values of True = open, False = close
    def control_red_autoclave(self, value):
        self.activate_autoclaves()  # Assumes that one would activate the actuator if they know they are plugged in.
        if self.autoclaves_activated == True:
            self.actuation_time = self.maximum_travel_length / self.desired_travel_speed
            # self.actuation_time = 10
            if value == True:
                if (self.linear_actuator_time_counter + self.actuation_time) <= self.linear_actuator_max_time:
                    self.linear_actuator_time_counter = self.linear_actuator_time_counter + self.actuation_time
                    print(self.linear_actuator_time_counter)

                    GPIO.output(actuator_input1, False)  # Black wire on input 1
                    GPIO.output(actuator_input2, True)  # White wire on input 2
                    self.pwm.ChangeDutyCycle(self.throttle * 100)
                    GPIO.output(actuator_enable, True)
                    # self.actuator.motor3.throttle = self.throttle
                    time.sleep(self.actuation_time)
                    GPIO.output(actuator_enable, False)
                    self.pwm.start(0)
                    # self.actuator.motor3.throttle = 0
                else:
                    print(
                        "Specified time will cause the linear actuator to exceed maximum extension or the actuator is at maximum position")
            elif value == False:
                if (self.linear_actuator_time_counter - self.actuation_time) >= 0:
                    self.linear_actuator_time_counter = self.linear_actuator_time_counter - self.actuation_time
                    print(self.linear_actuator_time_counter)
                    GPIO.output(actuator_input1, True)  # Black wire on input 1
                    GPIO.output(actuator_input2, False)  # White wire on input 2
                    self.pwm.ChangeDutyCycle(self.throttle * 100)
                    GPIO.output(actuator_enable, True)

                    # self.actuator.motor3.throttle = -self.throttle
                    time.sleep(self.actuation_time)
                    GPIO.output(actuator_enable, False)
                    self.pwm.start(0)
                    # self.actuator.motor3.throttle = 0
                else:
                    print(
                        "Specified actuation time will cause the linear actuator to exceed minimum extension or the actuator is at minimum position")

            else:
                print("Invalid entry. Enter True to open the Autoclave or enter False to close the Autoclave")
        else:
            print("Autoclave is not activated.")
        # GPIO.cleanup() # Clean up all GPIO channels

    def control_green_autoclave(self, value):
        self.activate_autoclaves()  # Assumes that one would activate the actuator if they know they are plugged in.
        if self.autoclaves_activated == True:
            self.actuation_time = self.maximum_travel_length / self.desired_travel_speed
            # self.actuation_time = 10
            if value == True:
                if (self.linear_actuator_time_counter + self.actuation_time) <= self.linear_actuator_max_time:
                    self.linear_actuator_time_counter = self.linear_actuator_time_counter + self.actuation_time
                    print(self.linear_actuator_time_counter)

                    GPIO.output(actuator_input1, False)  # Black wire on input 1
                    GPIO.output(actuator_input2, True)  # White wire on input 2
                    self.pwm.ChangeDutyCycle(self.throttle * 100)
                    GPIO.output(actuator_enable, True)
                    # self.actuator.motor3.throttle = self.throttle
                    time.sleep(self.actuation_time)
                    GPIO.output(actuator_enable, False)
                    self.pwm.start(0)
                    # self.actuator.motor3.throttle = 0
                else:
                    print(
                        "Specified time will cause the linear actuator to exceed maximum extension or the actuator is at maximum position")
            elif value == False:
                if (self.linear_actuator_time_counter - self.actuation_time) >= 0:
                    self.linear_actuator_time_counter = self.linear_actuator_time_counter - self.actuation_time
                    print(self.linear_actuator_time_counter)
                    GPIO.output(actuator_input1, True)  # Black wire on input 1
                    GPIO.output(actuator_input2, False)  # White wire on input 2
                    self.pwm.ChangeDutyCycle(self.throttle * 100)
                    GPIO.output(actuator_enable, True)

                    # self.actuator.motor3.throttle = -self.throttle
                    time.sleep(self.actuation_time)
                    GPIO.output(actuator_enable, False)
                    self.pwm.start(0)
                    # self.actuator.motor3.throttle = 0
                else:
                    print(
                        "Specified actuation time will cause the linear actuator to exceed minimum extension or the actuator is at minimum position")

            else:
                print("Invalid entry. Enter True to open the Autoclave or False enter  to close the Autoclave")
        else:
            print("Autoclave is not activated.")
        # GPIO.cleanup() # Clean up all GPIO channels

    def control_blue_autoclave(self, value):
        self.activate_autoclaves()  # Assumes that one would activate the actuator if they know they are plugged in.
        if self.autoclaves_activated == True:
            self.actuation_time = self.maximum_travel_length / self.desired_travel_speed
            # self.actuation_time = 10
            if value == True:
                if (self.linear_actuator_time_counter + self.actuation_time) <= self.linear_actuator_max_time:
                    self.linear_actuator_time_counter = self.linear_actuator_time_counter + self.actuation_time
                    print(self.linear_actuator_time_counter)

                    GPIO.output(actuator_input1, False)  # Black wire on input 1
                    GPIO.output(actuator_input2, True)  # White wire on input 2
                    self.pwm.ChangeDutyCycle(self.throttle * 100)
                    GPIO.output(actuator_enable, True)
                    # self.actuator.motor3.throttle = self.throttle
                    time.sleep(self.actuation_time)
                    GPIO.output(actuator_enable, False)
                    self.pwm.start(0)
                    # self.actuator.motor3.throttle = 0
                else:
                    print(
                        "Specified time will cause the linear actuator to exceed maximum extension or the actuator is at maximum position")
            elif value == False:
                if (self.linear_actuator_time_counter - self.actuation_time) >= 0:
                    self.linear_actuator_time_counter = self.linear_actuator_time_counter - self.actuation_time
                    print(self.linear_actuator_time_counter)
                    GPIO.output(actuator_input1, True)  # Black wire on input 1
                    GPIO.output(actuator_input2, False)  # White wire on input 2
                    self.pwm.ChangeDutyCycle(self.throttle * 100)
                    GPIO.output(actuator_enable, True)

                    # self.actuator.motor3.throttle = -self.throttle
                    time.sleep(self.actuation_time)
                    GPIO.output(actuator_enable, False)
                    self.pwm.start(0)
                    # self.actuator.motor3.throttle = 0
                else:
                    print(
                        "Specified actuation time will cause the linear actuator to exceed minimum extension or the actuator is at minimum position")

            else:
                print("Invalid entry. Enter True to open the Autoclave or enter False to close the Autoclave")
        else:
            print("Autoclave is not activated.")
        # GPIO.cleanup() # Clean up all GPIO channels

    # EMG Sensor Readings
    # EMG left is connected to channel 0 of the ADC.
    # EMG right is connected to channel 1 of the ADC.
    def emg_left(self):
        self.activate_emg_sensors()  # Assumes that one would activate the sensors if they know they are plugged in.
        if self.my_emg_activated == True:
            emg_left = self.mcp.read_adc(0) * 5.0 / 1023.0 * (1.0 / 5.0)
            # print(emg_left)
            return round(emg_left, 3)
        else:
            print("EMG sensors are not activated.")

    def emg_right(self):
        self.activate_emg_sensors() # Assumes that one would activate the sensors if they know they are plugged in.
        if self.my_emg_activated == True:
            emg_right = self.mcp.read_adc(1) * 5.0 / 1023.0 * (1.0 / 5.0)
            # print(emg_right)
            return round(emg_right, 3)
        else:
            print("EMG sensors are not activated.")

    # Move arm to target location based on cartesian coordinate inputs
    def move_arm(self,x,y,z):
        self.move_arm_intermediate() # first move to home position (the intermediate step)

        base, shoulder, elbow = self.arm_inverse_kinematics(x, y, z)
        # print(math.degrees(base))
        # print(math.degrees(shoulder))
        # print(math.degrees(elbow))

        self.rotate_base(math.degrees(base - self.b))
        #time.sleep(2)
        self.rotate_elbow(math.degrees(elbow - self.e))
        #time.sleep(2)
        self.rotate_shoulder(math.degrees(shoulder - self.s))
        #print("base, elbow, shoulder")
        self.b = base
        self.s = shoulder
        self.e = elbow

    # This must be done before turning off the arm. It moves the arm to a
    # suitable position for turning it off.
    def terminate_arm(self):
        self.move_arm_intermediate() # First move to the home position
        self.move_arm(0.05,0.00,0.29)

##################################################################################
# Background and debugging functions. DO NOT include in the library documentation.
##################################################################################
    def move_arm_intermediate(self):
        base, shoulder, elbow = self.arm_inverse_kinematics(0.406,0.0,0.483) # First move home without affecting the wrist or gripper
        # print(math.degrees(base))
        # print(math.degrees(shoulder))
        # print(math.degrees(elbow))
        if math.degrees(shoulder - self.s) >= 10:
            self.rotate_elbow(math.degrees(elbow) - math.degrees(self.e))
            # time.sleep(2)
            self.rotate_shoulder(math.degrees(shoulder) - math.degrees(self.s))
            # time.sleep(2)
            self.rotate_base(math.degrees(base) - math.degrees(self.b))
            # print("elbow, shoulder, base")
        else:
            self.rotate_shoulder(math.degrees(shoulder - self.s))
            # time.sleep(2)
            self.rotate_base(math.degrees(base - self.b))
            # time.sleep(2)
            self.rotate_elbow(math.degrees(elbow - self.e))
            # print("shoulder, base, elbow")
        time.sleep(0.5)
        self.b = base
        self.s = shoulder
        self.e = elbow

    # Calculate standard DH parameters
    # Taken from Quanser Simulation Library
    # Inputs:
    # a       :   translation  : along : x_{i}   : from : z_{i-1} : to : z_{i}
    # alpha   :      rotation  : about : x_{i}   : from : z_{i-1} : to : z_{i}
    # d       :   translation  : along : z_{i-1} : from : x_{i-1} : to : x_{i}
    # theta   :      rotation  : about : z_{i-1} : from : x_{i-1} : to : x_{i}
    # Outputs:
    # transformed       : transformation                   : from :     {i} : to : {i-1}
    def arm_dh(self, theta, d, a, alpha):
        # Rotation Transformation about z axis by theta
        a_r_z = np.array(
            [[math.cos(theta), -math.sin(theta), 0, 0],
             [math.sin(theta), math.cos(theta), 0, 0],
             [0, 0, 1, 0],
             [0, 0, 0, 1]])

        # Translation Transformation along z axis by d
        a_t_z = np.array(
            [[1, 0, 0, 0],
             [0, 1, 0, 0],
             [0, 0, 1, d],
             [0, 0, 0, 1]])

        # Translation Transformation along x axis by a
        a_t_x = np.array(
            [[1, 0, 0, a],
             [0, 1, 0, 0],
             [0, 0, 1, 0],
             [0, 0, 0, 1]])

        # Rotation Transformation about x axis by alpha
        a_r_x = np.array(
            [[1, 0, 0, 0],
             [0, math.cos(alpha), -math.sin(alpha), 0],
             [0, math.sin(alpha), math.cos(alpha), 0],
             [0, 0, 0, 1]])

        # For a transformation from frame {i} to frame {i-1}: transformed
        transformed = a_r_z @ a_t_z @ a_r_x @ a_t_x

        return transformed

    # Calculate end-effector position (x, y, z) using forward kinematics
    # Taken from Quanser Simulation Library
    # Input:    joint angles in rads
    # Output:   end-effector position (x, y, z) expressed in base frame {0}
    def arm_forward_kinematics(self, joint1, joint2, joint3, joint4):
        # Transformation matrices for all frames:
        # A{i-1}{i} = quanser_arm_dh(theta, d, a, alpha)

        A01 = self.arm_dh(joint1, self._L1, 0, -math.pi/2)
        A12 = self.arm_dh(joint2 - math.pi/2, 0, self._L2, 0)
        A23 = self.arm_dh(joint3, 0, 0, -math.pi/2)
        A34 = self.arm_dh(joint4, self._L3, 0, 0)

        A04 = A01 @ A12 @ A23 @ A34

        # Extract and return the x, y, z Position rounded to four decimal digits
        return round(A04[0, 3], 4), round(A04[1, 3], 4), round(A04[2, 3], 4)

    # Taken from Quanser Simulation Library
    def arm_inverse_kinematics(self,p_x, p_y, p_z):
        # Initialization
        q_base = 0
        q_shoulder = 0
        q_elbow = 0

        # Base angle:
        q_base = math.atan2(p_y, p_x)

        # Geometric definitions
        # Radial distance (R) projection onto the horizontal plane
        R = math.sqrt(p_x ** 2 + p_y ** 2)

        # Vertical offset within the verical plane from Frame 1 to End-Effector
        # Note: Frame 1 y-axis points downward (negative global Z-axis direction)
        Z = self._L1 - p_z

        # Distance from Frame 1 to End-Effector Frame
        Lambda = math.sqrt(R ** 2 + Z ** 2)

        # Angle of Lambda vector from horizontal plane (Frame 1)
        # Note: theta is measured about z-axis of Frame 1 so positive theta
        # rotates Lambda "down".
        theta = math.atan2(Z, R)

        # Based angle of the triangle formed by L2, L3 and Lambda
        # Computed using cosine law
        # Note: The sign of alpha determines whether it is elbow up (alpha < 0) or
        # elbow down (alpha > 0) configuration (i.e., consistent with Frame 1)
        alpha = math.acos(-(self._L3 ** 2 - self._L2 ** 2 - Lambda ** 2) / (2 * self._L2 * Lambda))

        # Solve for q_shoulder; elbow up solution
        q_shoulder = math.pi / 2 + (theta - alpha)

        # Solve for q_elbow, elbow up solution
        q_elbow = math.atan2(self._L2 - R * math.sin(q_shoulder) + Z * math.cos(q_shoulder),
                             R * math.cos(q_shoulder) + Z * math.sin(q_shoulder))

        # Return the joint angles in degrees
        return q_base, q_shoulder, q_elbow

    def current_reading(self):
        self.my_qarm.read_std()
        return self.my_qarm.measJointCurrent

# USED FOR DEBUGGING PURPOSES. OPEN AND CLOSE LINEAR ACTUATOR FULLY
    def linear_actuator(self,value):
        if value == True:
            GPIO.output(actuator_input1,False) # Black wire on input 1
            GPIO.output(actuator_input2, True) # White wire on input 2
            self.pwm.ChangeDutyCycle(self.throttle * 100)
            GPIO.output(actuator_enable,True)
            #self.actuator.motor3.throttle = self.throttle
            time.sleep(self.actuation_time)
            GPIO.output(actuator_enable,False)
            self.pwm.start(0)
        else:
            GPIO.output(actuator_input1,True) # Black wire on input 1
            GPIO.output(actuator_input2, False) # White wire on input 2
            self.pwm.ChangeDutyCycle(self.throttle * 100)
            GPIO.output(actuator_enable,True)
            #self.actuator.motor3.throttle = self.throttle
            time.sleep(self.actuation_time)
            GPIO.output(actuator_enable,False)
            self.pwm.start(0)

    def activate_emg_sensors(self):
        self.mcp = MCP3008(clk=CLK, cs=CS, miso=MISO, mosi=MOSI)
        self.my_emg_activated = True

        # Activates the mechanism for opening and closing the autoclaves
    def activate_autoclaves(self):
        # Linear actuator
        ##        self.actuator = MotorKit(i2c=board.I2C())
        ##        self.actuator.motor3.throttle = 0
        if self.autoclaves_activated == False:
            GPIO.setmode(GPIO.BCM)
            GPIO.setwarnings(False) # Added since it was giving a runtime warning for the setups below.
            # Set pins as output
            GPIO.setup(actuator_input1, GPIO.OUT)
            GPIO.setup(actuator_input2, GPIO.OUT)
            GPIO.setup(actuator_enable, GPIO.OUT)

            # setup pwm command and set the it to 0 duty
            self.pwm = GPIO.PWM(actuator_enable, 100)
            self.pwm.start(0)
            #self.autoclaves_activated = False

            self.linear_actuator_time_counter = 0  # used to keep track of time since the linear actuator doesn't have an encoder
            self.linear_actuator_max_time = 9  # (90mm/10mm/s) Assume that the actuator moves at 10mm/s (0.4 times full speed)
            self.actuation_time = 0

            self.maximum_travel_speed = 14.0  # based on chart, 5lbs linear acutator moves at a speed of 0.55 inches/second
            self.desired_travel_speed = 10.0  # desired rate is 10mm/s. Max in data sheets is 14 mm/s no load
            self.throttle = self.desired_travel_speed / self.maximum_travel_speed
            self.maximum_travel_length = 90  # ap
        self.autoclaves_activated = True
