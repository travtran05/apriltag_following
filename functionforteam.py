import cv2
from lane_detection import *
from lane_following import *
def PickLaneFromImage(frame):
    """ take in image and spit out the lane that has been picked"""
    #global imgPixelHeight
    imgPixelHeight = frame.shape[0]
    lines = detect_lines(frame, 100, 110, 3,10,10)
    lanes = detect_lanes(lines)
    # print ("tried to detect lanes")
    pickedLane = pick_lane(lanes)
    return pickedLane

def DrawLanes(frame,pickedLane, imgPixelHeight):
    """visualize the lanes, and the draw em."""
    center_intercept, center_slope = get_lane_center(pickedLane)
    xPoint = pickedLane[0][2]
    yPoint = pickedLane[0][3]
    #draw the lane that has been detected
    frame = draw_Single_lane(frame, pickedLane, (255, 0, 0))
    #frame = draw_lines(frame, lines,(0, 255, 0))
    #draw the center lane line
    cv2.line(frame, (int(center_intercept), imgPixelHeight), (int(xPoint), int(yPoint)), (0,0,255), 3)
    return frame
    pass