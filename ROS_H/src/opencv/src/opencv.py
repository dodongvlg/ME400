#!/usr/bin/env python
import rospy
import cv2
import numpy as np
from std_msgs.msg import Float64MultiArray
from std_msgs.msg import Float64
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

bridge = CvBridge()

cam_distance = 0.25 # 250mm
f_mm = 3.67 / 1000
sensor_width = 4.8 / 1000
pixel_width = 1920
f_pixel = f_mm * pixel_width / sensor_width
disparity = 0
img_left = 0
img_right = 0


# b = [98, 131, 131, 255, 0, 255]  # high hue, low hue, high calue, low value, high sat, low sat
# bC = [1, 101, 68, 8, 0, 68, 1]	# parameter related to circles (dp,minDist,param1,param2,minRadius,maxRadius,BlurSize)
b = [95, 129, 159, 255, 0, 255]
bC = [1, 901, 1, 26, 0, 244, 7]

def getHSV(frame, l, m=[]):
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    LB = np.array(l[::2])
    UB = np.array(l[1::2])
    if m != []:
        LB_ = np.array(m[::2])
        UB_ = np.array(m[1::2])
        mask = cv2.inRange(hsv, LB, UB) + cv2.inRange(hsv, LB_, UB_)
    else:
        mask = cv2.inRange(hsv, LB, UB)
    res = cv2.bitwise_and(frame, frame, mask=mask)
    # return res#,cv2.bitwise_not(mask)
    return mask

def getHoughCircles(frame, l):
    if l[6] % 2 == 0: l[6] += 1
    frame = cv2.blur(frame, (l[6], l[6]))
    highlight = frame.copy()

    if len(frame.shape) == 3:
        frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    else:
        highlight = cv2.cvtColor(frame, cv2.COLOR_GRAY2BGR)
    circles = cv2.HoughCircles(frame, cv2.HOUGH_GRADIENT, l[0], l[1], param1=l[2], param2=l[3], minRadius=l[4], maxRadius=l[5])
    # print(circles)
    if circles is not None:
        return circles
    else:
        return np.array([], dtype=np.float32)

def drawCircles(frame,circlesList,textList=[]) :
    
    font = cv2.FONT_HERSHEY_SIMPLEX ;fontScale = 0.5 ;color = (255, 255, 255) ;thickness = 1
    highlight=frame.copy()

    for (circles,text) in zip(circlesList,textList):
        if circles is not None:
            circles = np.round(circles[0, :]).astype("int")
            for (x, y, r) in circles:
                highlight=cv2.circle(highlight, (x, y), r, (0, 255, 0), 4)
                highlight=cv2.rectangle(highlight, (x - 3, y - 3), (x + 3, y + 3), (0, 128, 255), -1)
                highlight=cv2.putText(highlight, text, (x,y), font, fontScale, color, thickness, cv2.LINE_AA) 
    return highlight

def left_circle_callback(data):
    global left_blue_ball
    left_blue_ball = np.array([], dtype=np.float32)
    img_left = bridge.imgmsg_to_cv2(data, "bgr8")
    left_blue_ball = getHoughCircles(getHSV(img_left, b), bC)
    # print("left", left_blue_ball)
    # cv2.imshow('blue',drawCircles(img_left,[left_blue_ball],["Blue ball"]))
    # cv2.waitKey(0)
    
    # left_blue_ball = getHoughCircles(getHSV(img_left, b), bC)
    
def right_circle_callback(data):
    global right_blue_ball
    right_blue_ball = np.array([], dtype=np.float32)
    img_right = bridge.imgmsg_to_cv2(data, "bgr8")
    right_blue_ball = getHoughCircles(getHSV(img_right, b), bC)
    # cv2.imshow('blue',drawCircles(img_right,[right_blue_ball],["Blue ball"]))
    # cv2.waitKey(0)
    # print("right", right_blue_ball)
    # img_right = bridge.imgmsg_to_cv2(data, "bgr8")
    # right_blue_ball = getHoughCircles(getHSV(img_right, b), bC)

if __name__ == '__main__':
    global left_blue_ball
    global right_blue_ball

    left_blue_ball = np.array([], dtype=np.float32)
    right_blue_ball = np.array([], dtype=np.float32)
    x_const = 0.88 # Consts for calibration
    y_const = 0.88


    dist_pub = rospy.Publisher("/ball_distance", Float64MultiArray, queue_size = 10)
    rospy.Subscriber("/camera_left/rgb/image_raw", Image, left_circle_callback)
    rospy.Subscriber("/camera_right/rgb/image_raw", Image, right_circle_callback)

    rospy.init_node("OpenCV_Node")

    rate = rospy.Rate(5)

    while not rospy.is_shutdown():
        distance = []
        pub_distance = Float64MultiArray()
        if ((left_blue_ball.size == 0) or (right_blue_ball.size == 0)):
            distance.append(-1.0)

        else:
            for c1 in left_blue_ball[0, :]:
                for c2 in right_blue_ball[0, :]:
                    if ((c2[0] - 30 < c1[0]) or (c1[0] < c2[0] + 30)):
                        if ((c2[1] - 5 < c1[1]) or (c1[1] < c2[1] + 5)):
                            x_dist = cam_distance * f_pixel / abs(c1[0] - c2[0]) * x_const
                            y_dist = - (x_dist * ((960 - c1[0]) + (960 - c2[0]))) / (2 * f_pixel) * y_const
                            distance.append(x_dist)
                            distance.append(y_dist)
        
            if (not distance):
                distance.append(-1.0)
        # print(distance)
        pub_distance.data = distance
        dist_pub.publish(pub_distance) ############# Publish blue ball distance from now
        print("publishing")
        rate.sleep()

