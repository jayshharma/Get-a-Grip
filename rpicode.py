import random 
import time
import sys

sys.path.append('../')

from Common_Libraries.p2_sim_lib import *

import os
from Common_Libraries.repeating_timer_lib import repeating_timer

def update_sim ():
    try:
        arm.ping()
    except Exception as error_update_sim:
        print (error_update_sim)

arm = qarm()
update_thread = repeating_timer(2, update_sim)

##----------------------------------------------------------------

def find_target(containerID): # function for identifying container ID target location
    # first row: small red, small green, and small blue target locations respectively
    # second row: big red, big green, and big blue target locations respectively
    target = [[-0.604, 0.239, 0.439], [0.011, -0.65, 0.439], [-0.011, 0.65, 0.439],
             [-0.4, 0.166, 0.2], [0.0, -0.442, 0.211], [0.0, 0.455, 0.228]]

    if containerID == 1: #if container ID is 1, get coordinates for "small red"
        return target[0]
    
    elif containerID == 2: #if container ID is 2, get coordinates for "small green"
        return target[1]
    
    elif containerID == 3: #if container ID is 3, get coordinates for "small blue"
        return target[2]
    
    elif containerID == 4: #if container ID is 4, get coordinates for "big red"
        return target[3]
    
    elif containerID == 5: #if container ID is 5, get coordinates for "big green"
        return target[4]
    
    elif containerID == 6: #if container ID is 6, get coordinates for "big blue"
        return target[5]



def move_effector(target): # function for moving end effector depending emg muscle sensor values
    print("Waiting to move... (both arms up)")
    print("NOTE: MUST RETURN HOME BEFORE MOVING TO AUTOCLAVE\n")
    
    
    while True: 
        L_sensor = arm.emg_left()
        R_sensor = arm.emg_right()
        
        if L_sensor > 0.6 and R_sensor > 0.6: # executes function when both arms are up
            arm.move_arm(target[0], target[1], target[2])
            time.sleep(1.5)
            break


def control_gripper(position): # open and closes the gripper
    pick_up = [0.531, 0.0, 0.025]

    print("Waiting to open/close gripper ... (right arm up, left arm down)\n")    
    # keeps checking the control gripper condition until its true
    while True:
        L_sensor = arm.emg_left()
        R_sensor = arm.emg_right()
        
        if R_sensor > 0.6 and L_sensor == 0:
            # if at the pick_up position, close the gripper. Open if otherwise
            if position == pick_up:
                arm.control_gripper(32)
            
            else:
                arm.control_gripper(-32)
                
            break
            
def autoclave_bin(action, containerID): # opens or closes autoclave bin based on action parameter and containerID
    open_bin = True # autoclave should open when the "action" variable is equal to true

    print("Waiting to open/close autoclave bin... (left arm up, right arm down)\n")
    while True:
        L_sensor = arm.emg_left()
        R_sensor = arm.emg_right()
        
        if L_sensor > 0.6 and R_sensor == 0: # when the left arm is up and the right arm is fully extended open or close autoclave bin
            if action == open_bin: # the bin only opens when the action variable is set to true 
                if containerID == 4: # based on Container ID, function dictates what autoclave bin to open or close
                    arm.open_red_autoclave(True) 
            
                elif containerID == 5:
                    arm.open_green_autoclave(True) 

                elif containerID == 6:
                    arm.open_blue_autoclave(True)

                time.sleep(1.5)
            
            else: 
                if containerID == 4: 
                    arm.open_red_autoclave(False)
            
                elif containerID == 5:
                    arm.open_green_autoclave(False)

                elif containerID == 6:
                    arm.open_blue_autoclave(False)
                    
                time.sleep(1.5)
                
            break

def spawn_container(containerID, cycle_num): # function for spawning a random container
    print("SPAWNING CONTAINER",cycle_num, "\n")
    arm.spawn_cage(containerID)
    time.sleep(2)
    
def startup(): # function for starting program
    print("\nDONT FORGET TO RESET SIM")
    print("\nRUNNING PROGRAM IN...")
    for i in range(5, -1, -1):
        print(i)
        time.sleep(1)
    print("")

def main(): # main function for executing entire program
    ID_list = [1, 2, 3, 4, 5, 6]
    random.shuffle(ID_list) # shuffles ID_list to be in random order
    random_order = ID_list
    
    pick_up = [0.531, 0.0, 0.025]
    home = [0.406, 0.0, 0.483]
    
    open_bin = True # opens bin when open_bin variable is set to true
    close_bin = False # closes bin close_bin variable is set to false

    cycle_num = 1

    startup()

    # spawns containers based on the random order created by shuffling the original ID list
    for i in random_order:
        # indented block below follows program flow for one cycle
        containerID = i
        
        spawn_container(containerID, cycle_num)
        
        move_effector(pick_up)

        control_gripper(pick_up)

        drop_off = find_target(containerID)

        move_effector(home)

        if containerID == 4 or containerID == 5 or containerID == 6:
            autoclave_bin(open_bin, containerID)

        move_effector(drop_off)

        control_gripper(drop_off)

        move_effector(home)

        if containerID == 4 or containerID == 5 or containerID == 6:
            autoclave_bin(close_bin, containerID)

        cycle_num += 1

    print("beep boop im robot")

main()
