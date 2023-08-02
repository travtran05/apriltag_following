from motor_test import test_motor
import time
from pymavlink import mavutil
import socket
import select
import numpy as np
def arm_rov(mav_connection):
    """
    Arm the ROV, wait for confirmation
    """
    mav_connection.arducopter_arm()
    print("Waiting for the vehicle to arm")
    mav_connection.motors_armed_wait()
    print("Armed!")

def disarm_rov(mav_connection):
    """
    Disarm the ROV, wait for confirmation
    """
    mav_connection.arducopter_disarm()
    print("Waiting for the vehicle to disarm")
    mav_connection.motors_disarmed_wait()
    print("Disarmed!")

def run_motors_timed(mav_connection, seconds: int, motor_settings: list) -> None:
    """
    Run the motors for a set time
    :param mav_connection: The mavlink connection
    :param time: The time to run the motors
    :param motor_settings: The motor settings, a list of 6 values -100 to 100
    :return: None
    """
    start_time = time.time()
    while time.time()-start_time < seconds:
        for i in range(len(motor_settings)):
            test_motor(mav_connection=mav_connection, motor_id=i, power=motor_settings[i])
        time.sleep(0.2)

def forward_cw_spin():
    '''Makes robot go forward while spinning'''
    sec = 3.8325
    run_motors_timed(mav_connection, seconds=sec, motor_settings=[-100,-100 ,0 ,100, 0, 0])
    #print("command one")
    run_motors_timed(mav_connection, seconds=sec, motor_settings=[100,100,0,100, 0, 0])
    #print("command two")
    run_motors_timed(mav_connection, seconds=sec, motor_settings=[100,0 ,-100 ,-100, 0, 0])
    #print("command three")
    run_motors_timed(mav_connection, seconds=sec, motor_settings=[100,0, 100,100, 0, 0])
    #print("command four")
    run_motors_timed(mav_connection, seconds=0.2, motor_settings=[100,-100 ,-100 ,100, 0, 0])

def forward_ccw_spin():
    sec = 3.8325
    run_motors_timed(mav_connection, seconds=sec, motor_settings=[-100,-100 ,100 ,0, 0, 0])
    print("command one")
    run_motors_timed(mav_connection, seconds=sec, motor_settings=[100,100,100,0, 0, 0])
    print("command two")
    run_motors_timed(mav_connection, seconds=sec, motor_settings=[0,100 ,-100 ,-100, 0, 0])
    print("command three")
    run_motors_timed(mav_connection, seconds=sec, motor_settings=[0,100, 100,100, 0, 0])
    print("command four")
    run_motors_timed(mav_connection, seconds=0.2, motor_settings=[-100,100 ,100 ,-100, 0, 0])

def jump():
    run_motors_timed(mav_connection, seconds = 5, motor_settings=[0,0,0,0,-30,-30])
    print("About to jump")
    run_motors_timed(mav_connection, seconds = 1, motor_settings=[0,0,0,0,100,100])
#main
def pirouette(sec, power, times):
    sec = sec/(power/100)
    for i in range(times):
        run_motors_timed(mav_connection, seconds=sec, motor_settings=[-power,-power ,0 ,power, 0, 0])
        print("command one")
        run_motors_timed(mav_connection, seconds=sec, motor_settings=[power,power,0,power, 0, 0])
        print("command two")

        run_motors_timed(mav_connection, seconds=sec, motor_settings=[power,0 ,-power ,-power, 0, 0])
        print("command three")
        run_motors_timed(mav_connection, seconds=sec, motor_settings=[power,0, power,power, 0, 0])
        print("command four")
       
def makeBubbles(duration):
    run_motors_timed(mav_connection, seconds=duration, motor_settings=[0,0, 0 ,0, 100, 100])

def circle(Strafe,turn, power, time):
    StrafeVector = np.array([-Strafe,Strafe,-Strafe,Strafe])
    turnVector = np.array([turn,-turn,-turn,turn])

    ResultantVector = turnVector+StrafeVector
    scalingConstant = power/np.max(ResultantVector)
    
    ResultantVector = ResultantVector*scalingConstant
    #print (ResultantVector)
    run_motors_timed(mav_connection, seconds=time,motor_settings= ResultantVector.tolist())


def figureEight():
    
    circle(100,10, 50,20)
    run_motors_timed(mav_connection, seconds=20, motor_settings=[-50,50, -50 ,50, 0, 0])
    circle(100,-10, 50,26.5)
    run_motors_timed(mav_connection, seconds=27, motor_settings=[-50,50, -50 ,50, 0, 0])
if __name__ == "__main__":

    ####
    # Initialize ROV
    ####
    mav_connection = mavutil.mavlink_connection('udpin:0.0.0.0:14550')
    print("Waiting for connection")
    mav_connection.wait_heartbeat()
    # Arm the ROV and wait for confirmation
    arm_rov(mav_connection)

    print("ARMED")

    ####
    # Run choreography
    ####
    """
    Call sequence of calls to run_timed_motors to execute choreography
    Motors power ranges from -100 to 100
    """
    #forward_ccw_spin()
    #run_motors_timed(mav_connection,5,[100,100,-100,-100,0,0])
    #forward_cw_spin()
    #pirouette(5,100,1)
    figureEight()
    # stop
    run_motors_timed(mav_connection, seconds=5, motor_settings=[0, 0, 0, 0, 0, 0])
    print("stopped")
    ####
    # Disarm ROV and exit
    ####
    disarm_rov(mav_connection)
    print("disarmed")