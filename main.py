# Imports
from threading import Thread, Event
from time import sleep
from pid import PID
from video import Video
from bluerov_interface import BlueROV
from pymavlink import mavutil
from dt_apriltags import Detector
import cv2

# TODO: import your processing functions
from apriltag_detection import *
from lane_following import *
from lane_detection import *
from depthControl import *



# Create the video object
video = Video()
# Create the PID object
pid_vertical = PID(K_p=50, K_i=1, K_d=-27.5, integral_limit=100)
pid_horizontal = PID(K_p=0.1, K_i=0.0, K_d=0.01, integral_limit=1)
pid_horizontal_lf = PID(K_p=5, K_i=0.0, K_d=0.01, integral_limit=1)
pid_heading_lf = PID(K_p=0.1, K_i=0, K_d=-0.01, integral_limit=100)
# Create the mavlink connection
mav_comn = mavutil.mavlink_connection("udpin:0.0.0.0:14550")
# Create the BlueROV object
bluerov = BlueROV(mav_connection=mav_comn)

frame = None
frame_available = Event()
frame_available.set()

# Power values and followRobot boolean
vertical_power = 0
lateral_power = 0
yaw_power = 0
longitudinal_power = 0
followRobot = False
# lanesDetected = Falseå


def _get_frame():
    """
    Function: Gets the frame from the AUV and determines how to move based on if it detects a lane or apriltags
    """

    # Global variables for frame, power, and robot following
    global frame, vertical_power, lateral_power, yaw_power, longitudinal_power, followRobot

    # Sets the values for the PID controllers for the vertical and horizontal directions
    vertical_pid = PID(1, 0, 0, 100)
    horizontal_pid = PID(1, 0, 0, 100)

    # Creates a detector that can be used to detect the apriltags on the robot and on the walls
    at_detector = Detector(families='tag36h11',
                    nthreads=1,
                    quad_decimate=1.0,
                    quad_sigma=0.0,
                    refine_edges=1,
                    decode_sharpening=0.25,
                    debug=0)
    
    # Keeps waiting until a video frame is available
    while not video.frame_available():
        print("Waiting for frame...")
        sleep(0.01)

    # bluerov.set_rc_channel(9, 1100) #turns lights off if we need to
    # Tries to process a frame and adjust the robot's movement accordingly
    try:
        while True:
            if video.frame_available():
                frame = video.frame() # Initializes the frame
                
                #print(frame.shape)
                
                """
                tags, tag_z = detect_tag(frame, at_detector) # detects and returns all apriltags in the given frame
                print("Got frame")
                # Checks if any apriltags have been detected
                if len(tags) > 0:
                    # Sets lanesDetected to False so depth control isn't activated
                    #lanesDetected = False
                    # Sets followRobot variable to True so the robot will track the robot's apriltag and move towards it
                    followRobot = True
                    # Utilizies the last tag found
                    center_tags = tags[-1]
                    print("Got tag")
                    # Gets the horizontal and vertical outputs for the PID to make thruster adjustments and move towards the other AUV's apriltag
                    horizontal_output, vertical_output = PID_tags(frame.shape, center_tags[0], center_tags[1], horizontal_pid, vertical_pid)
                    # Draws the arrows in terms of where the apriltag is in relation to the center of the AUV's camera
                    img = drawOnImage(frame, center_tags, horizontal_output, vertical_output)
                    
                    # Sets vertical and lateral/horizontal power for the thrusters
                    vertical_power = vertical_output
                    lateral_power = horizontal_output

                    # Goes straight forward if the apriltag is already in the center of the AUV's camera
                    if(vertical_power < 0.1 and lateral_power < 0.1):
                       #pass
                       longitudinal_power = 20
                    # Sets longitudinal/forward thruster magnitudes to 0 so it doesn't go forward
                    else:
                        longitudinal_power = 0
                    
                    # Writes image so we can see the apriltag the AUV detected in VS Code
                    # cv2.imwrite("ROV_frame.jpg", img)

                    # Turns off thrusters and flashes lights on and off if the AUV is in "shooting" range of the other AUV
                    if (tag_z<0.1):
                        vertical_power = 0
                        lateral_power = 0
                        longitudinal_power = 0

                        for i in range(10):
                            bluerov.set_rc_channel(9,1100)
                            time.sleep(0.1)
                            bluerov.set_rc_channel(9,1500)
                            time.sleep(0.1)

                # If no AUV's apriltag is detected, the AUV shuts off all thrusters and prepares to follow the lanes
                else:
                    followRobot = False
                    vertical_power = 0
                    lateral_power = 0
                    longitudinal_power = 0
                """
                
                followRobot = False
                # Checks if the AUV isn't following another robot and performs lane detection/following
                if(followRobot == False):
                    #frame = cv2.imread("ROV_frame.jpg")#halllucinate a img during testing


                    #turn
                    #print (img.shape)
                    # cropped_image = frame[180:360, 10:600]# crop the image cause theres garbage on the sides
                    cropped_image = frame[int(frame.shape[0]/2):,]
                    frame = cropped_image
                    # Creates a list of the lines that have been detected
                    line_list = detect_lines(frame, 40, 110, 3, 10, 10)
                    #print (line_list)

                    
                    # Tries to detect the lanes from any lines that have been found
                    
                    if(not (line_list is None)):
                        img = draw_lines(frame, line_list)
                        
                        
                        print("got line")
                        #print("attempting to pick a lane")
                        try:
                            pickedLane = PickLaneFromImage(frame)
                            draw_Single_lane(img,pickedLane)
                            center_intercept, center_slope = get_lane_center(cropped_image.shape[1],pickedLane)
                            #print(f"center_intercept:{center_intercept} center_slope:{center_slope}")
                            HorizontalDiff, AproxAUVAngle = recommend_direction(cropped_image.shape[1], center_intercept, center_slope)
                            #print (f"HorizontalDiff:{HorizontalDiff/40} AproxAUVAngle: {AproxAUVAngle/2}")
                            
                            # print(Horizontal)
                            if(abs(HorizontalDiff) < 30 and abs(AproxAUVAngle < 6)):
                                yaw_power = 0
                                lateral_power = 0
                                # Sets longitudinal power to 0 so the robot moves forward
                                longitudinal_power = 20
                                
                            else:
                                # Sets longitudinal power to 0 to make sure the AUV isn't moving straight at the same time the robot is realigning itself
                                longitudinal_power = 10
                                # Calculates the yaw and lateral thruster powers
                                yaw_power, lateral_power = lane_PID(frame.shape[0], AproxAUVAngle, HorizontalDiff/40, pid_heading_lf, pid_horizontal_lf)
                                yaw_power = int(yaw_power)
                                lateral_power = int(lateral_power)
                                print(f"yaw_power: {yaw_power}")
                                print(f"lateral_power: {lateral_power}")
                                # yaw_power, lateral_power = lane_PID(frame.shape[0], AproxAUVAngle, 0, pid_heading_lf, pid_horizontal_lf)
                            #lanesDetected = True
                        except:
                            pickedLane = None
                        
                        
                        # Tries to analyze the lanes and align the AUV with the center of the lane
                        
                        print("got lane")
                        

                       # frame = DrawLanes(img,pickedLane, img.shape[1])
                        cv2.imwrite("ROV_frame_lanes.jpg", img)
                        # Gets the x-intercept and the slope of the line running through the center of the lane
                       

                            # Writes the image so we can see the lanes the AUV detected in VS code
                            #cv2.imwrite("ROV_frame.jpg", img)
                        # Turns off thrusters and sets followRobot to False in case no lanes are detected
                    else:
                        followRobot = False
                        vertical_power = 0
                        lateral_power = 0
                        yaw_power = 0
                        longitudinal_power = 0
                    # Turns off thrusters and sets followRobot to False in case no lines/lanes are detected
                        
                    
    except KeyboardInterrupt:
        return


def depth_control():
    global pid_vertical, vertical_power, followRobot
    # mav = mavutil.mavlink_connection("udpin:0.0.0.0:14550")
    
    # mav = bluerov.mav_connection
    # if this doesn't work, then replace bluerov.mav_connection with mav_comm
    while True:
        if(followRobot == False): # remember to uncomment----------------------------------------------------------------
            print("Got into depth control loop")
            msg = bluerov.mav_connection.recv_match(type="SCALED_PRESSURE2", blocking=True)
            press_abs = msg.press_abs
            current_depth = press_to_depth(press_abs)
            error = 0.5 - current_depth
            output = pid_vertical(error)
            print(f"depth output:{output}")
            vertical_power = output
            #print(vertical_power)
            #bluerov.set_vertical_power(output)

def _send_rc():
    global vertical_power, lateral_power, yaw_power, longitudinal_power
    bluerov.set_rc_channels_to_neutral()
    mav_comn.set_mode(19)
    while True:
        # bluerov.disarm()
        bluerov.arm()

        # Sets the powers of the thrusters based on outputs for the PID controllers
        bluerov.set_vertical_power(int(vertical_power))
        bluerov.set_lateral_power(-int(lateral_power))
        bluerov.set_yaw_rate_power(int(yaw_power))
        bluerov.set_longitudinal_power(int(longitudinal_power))

# Start the video thread
video_thread = Thread(target=_get_frame)
video_thread.start()

# Start the depth control thread
#depth_thread = Thread(target=depth_control)
#depth_thread.start()

# Start the RC thread
rc_thread = Thread(target=_send_rc)
rc_thread.start()

# Main loop
try:
    while True:
        mav_comn.wait_heartbeat()
except KeyboardInterrupt:
    video_thread.join()
    #depth_thread.join()
    rc_thread.join()
    bluerov.disarm()
    print("Exiting...")