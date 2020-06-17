#!/usr/bin/env python
import rospy
import cv2
import numpy as np

from std_msgs.msg import Float64 ,Int64
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import math





'''
***required imports
Method 1 =>
    ( python 2.7) 
    sudo apt install python-pip  
    pip install scikit-image

    ( python 3.x )
    sudo apt install python3-pip
    pip3 install scikit-image

Method 2=>
    sudo apt-get install python3-matplotlib python3-numpy python3-pil python3-scipy python3-tk
    sudo apt-get install build-essential cython3
'''
###################################################
from skimage import measure
###################################################





bridge = CvBridge()
img_center = np.zeros((1,1,3),dtype=np.uint8)



######################pusblisher variables#########
regionFlag=Int64()
holderFlag=Int64()
xc=Int64()
orientation = Float64()
xcDocking = Float64()
###################################################
'''
CENTER CAMERA HSV PARAMS
'''  
blueMask=[98,131,131,255,0,255]
greenMask=[50,115,131,255,8,252]
redMask = [142,180,99,255,8,252];
redMask_=[0,4,99,255,8,252]
###################################################



def adjustOrientationValue(val):
    '''
    1.make north be the zero
    2.make the range from west(pi/2) to east(-pi/2)
    
    '''
    if val >= 0 : return val - math.pi/2
    else : return val + math.pi /2 
def selectSection(img,verticalStart=-1,verticalEnd=-1,horizontalStart=-1,horizontalEnd=-1,value = 0):
    #select part of an image as a percentage of height start and end
    # for example start = 0.5 , end = 1 means start from middle and end at the bottom of frame
    img_=img.copy()
    
    if len(img_.shape)==2 : y,x= img.shape
    elif len(img_.shape)==3 : y,x,z= img.shape
        
    vs,ve,hs,he = map(int,[verticalStart*y,verticalEnd*y,horizontalStart*x,horizontalEnd*x])
    
    if len(img_.shape) ==3 : 
        if verticalStart !=-1 : img_ [: vs,:] =[value,value,value]
        if verticalEnd !=-1 :img_ [ve:,:] = [value,value,value]
        if horizontalStart !=-1 :img_ [:,:hs] =[value,value,value]
        if horizontalEnd !=-1 :img_ [:,he:] =[value,value,value]
    elif len(img.shape) ==2 : 
        if verticalStart !=-1: img_ [: vs,:] =value
        if verticalEnd !=-1 :img_ [ve:,:] = value
        if horizontalStart !=-1 :img_ [:,:hs] =value
        if horizontalEnd !=-1 :img_ [:,he:] =value
    return img_
def getProp(binaryImage):
    label_img = measure.label(binaryImage)
    regions = measure.regionprops(binaryImage)
    if regions !=[] :return regions[0] 
    else: return []
def ballHolderState(img,thresh, redBallMask,blueBallMask):
    '''
    thresh = > minimum ratio of mask and ball intersection measured using white pixels
    outputs  
        0=> if no ball in holder
        1=> if blue ball in holder
        2=> if red ball in holder
    '''
    
    def compareHolder(img, x= 960 , y = 1300, radius = 450 ):
        yy,xx= img.shape
        f =np.zeros((yy,xx,3),dtype=np.uint8)
        mask = cv2.circle(f.copy(),(x,y ),radius,(255,255,255),-1);mask=mask[:,:,0]
        
        maskAndImg= cv2.bitwise_and(mask,img)
        
        maskWhitePixels = float(cv2.countNonZero(mask))
        maskAndImgWhitePixels= float(cv2.countNonZero(maskAndImg))
        return maskAndImgWhitePixels  / maskWhitePixels  

    ratioBlue= compareHolder(blueBallMask) ; 

    if ratioBlue >=thresh : 
        print("holder: blue ball ") ;
        return 1 
    
    else :
        ratioRed= compareHolder(redBallMask)
        if ratioRed >= thresh : 
            print("holder: red ball ") ;
            return 2
        else : 
            print("holder: no ball ")
            return 0
def regionState(img,thresh,redBallMask,blueBallMask):
    if cv2.countNonZero(redBallMask) > thresh  or cv2.countNonZero(blueBallMask) > 0 :
        print("---------------ball harvesting region------------------------------------")
        return 1
    
    print("---------------line tracing region------------------------------------")
    return 0
def getColorMask(img,p1,p2=[]):
    
    hsv=cv2.cvtColor(img,cv2.COLOR_BGR2HSV)
    
    LB=np.array(p1[::2]);  #Lower bound of HSV
    UB=np.array(p1[1::2])  #Upper bound of HSV
    
    #merge two bounds for case of red color
    if p2!=[]: 
        LB_=np.array(p2[::2]);
        UB_=np.array(p2[1::2]);
        mask=cv2.inRange(hsv,LB,UB) + cv2.inRange(hsv,LB_,UB_)
    
    else : 
        mask=cv2.inRange(hsv,LB,UB)
        
    res=cv2.bitwise_and(img,img,mask=mask)
    
    return mask
def getCentroidOrientation(img,start=0,end=1):
    '''
    start portion of image vertically
    end portion of image vertically
    for example start : 0.5 end :1 means get orientation of line from middle to bottom
    
    ***return -1 ,-1 if no line is found
    
    '''

    ret,thresh=cv2.threshold(img,80,255,cv2.THRESH_BINARY_INV);
    props = getProp(selectSection(thresh[:,:,0],start,end))
    
    if props !=[] :
        x,ori =  props.centroid[1]  , adjustOrientationValue(props.orientation)
        print ("line x centroid :" ,x )
        print("line orientation: ", ori)
        return x,ori
    else : 
        print ("line x centroid :" ,"not detected" )
        print ("line orientation: ", "not detected")
        return -1,-1
def getCentroidBall(img, thresh,blueBallMask):
    '''
    Get accurate position of ball at docking position
    
    '''
    
    
    def eccentricityThresh(regions,thresh):
        #output regions of full visible circle shape
        for r in regions:
            if r.eccentricity <= thresh :
                yield r

    def filterBallsByArea(regions,baseArea=189738):
        curRegion = None
        for r in regions :
            if  r.area >= thresh*0.7 :# and r.area <= baseArea*1.3 :
                if r.area > curRegion : curRegion=r
        if curRegion is None : return -2,-2,-2
        else : return curRegion.centroid[1],curRegion.centroid[0], math.sqrt((curRegion.area/math.pi))

    half = img.shape[1]/2

    labeled = measure.label(blueBallMask)
    regions = measure.regionprops(labeled)
    x1,y1,r1 = filterBallsByArea(eccentricityThresh (regions ,0.5))


    if x1 != -2 : 
        print("center camera detected blue ball in sight")
        return -(x1-half)/half 
    else : return -2


    #return regionMethod,houghMethod

    
def center_circle_callback(data) :
    global regionFlag,holderFlag,xc,orientation
    global blueMask , redMask ,redMask_
    
    
    '''
    Four tasks 
     1 - detect if the car is in line tracing[=0] or ball harvesting region[=1]   => regionState
     2 - detect if blue ball[=1] , red ball[=2] or not ball in holder [=0]        => ballHolderState
     3 - give orientation and horizontal centroid of line                         => getCentroidOrientation
     4 - give accurate position of ball when aligning in docking position         => getCentroidBall
    '''

    '''
    if region is line tracing =>
        get orientation and centroid of line
        if slope is detected =>
            set orientation=0 and move at full speed up
    
    else if region is ball harvesting =>
        get holder content ( no ball, blue ball , red ball)
        get accurate position of ball using center camera when aligning in docking position
    '''

    img_center = bridge.imgmsg_to_cv2(data, "bgr8")

    redBallMask = getColorMask(img_center,redMask,redMask_)
    blueBallMask = getColorMask(img_center,blueMask)


    
    if regionFlag.data !=1 : 
        regionFlag.data = regionState(img_center,5,redBallMask,blueBallMask)


    
    #line tracing region
    if regionFlag.data == 0 :

        xc2,orientation2= getCentroidOrientation(img_center,0,0.4)     #proprties of top half of view

        if xc2!=-1 :
            xc.data,orientation.data = getCentroidOrientation(img_center,0.6,0.98)
        
        else : 
            xc.data,orientation.data = getCentroidOrientation(img_center,0.81,0.99);
            print("move straight")
            orientation.data =0   #  if there is no line in top half then make orienttation =0 


    
    #ball harvesting region
    elif regionFlag.data == 1 :
        
        holderFlag.data = ballHolderState(img_center,
                                         1,
                                         selectSection(redBallMask,0.5,1),
                                         selectSection(blueBallMask,0.5,1) )  
        
        
        xcDocking.data = getCentroidBall(img_center,0.7 , blueBallMask)

    
    print("")
    print("---------------------------------------------------------------------------------")
    print("")

    
    
if __name__ == '__main__':
    left_blue_ball = np.array([], dtype=np.float32)
    right_blue_ball = np.array([], dtype=np.float32)

#   dist_pub = rospy.Publisher("/ball_distance", Float64, queue_size = 10)
#   rospy.Subscriber("/camera_left/rgb/image_raw", Image, left_circle_callback)
#   rospy.Subscriber("/camera_right/rgb/image_raw", Image, right_circle_callback)
    rospy.Subscriber("/camera_center/rgb/image_raw", Image,center_circle_callback )
    #rospy.Subscriber("/camera_left/rgb/image_raw", Image,left_circle_callback )
    #rospy.Subscriber("/camera_right/rgb/image_raw", Image,right_circle_callback )


    ####line tracing publishers######################################################
    regionFlag_pub = rospy.Publisher("/regionFlag", Int64, queue_size = 10)
    holderFlag_pub = rospy.Publisher("/holderFlag", Int64, queue_size = 10)
    xc_pub = rospy.Publisher("/line/centroid", Int64, queue_size = 10)
    orientation_pub = rospy.Publisher("/line/orientation", Float64, queue_size = 10)
    xcDocing_pub = rospy.Publisher("/docking/centroid",Float64, queue_size = 10)
    ##################################################################################
    
    rospy.init_node("OpenCV_Node2")

    rate = rospy.Rate(20)

 

while not rospy.is_shutdown():
  
  
  ########################line tracing code###################################################  
  #print(-xc.data+img_center.shape[1]//2,(orientation.data)*180/math.pi)  
  regionFlag_pub.publish(regionFlag)
  holderFlag_pub.publish(holderFlag)
  xc_pub.publish(xc)
  orientation_pub.publish(orientation)
  xcDocing_pub.publish(xcDocking)
  ############################################################################################
  
  
  
  rate.sleep()
