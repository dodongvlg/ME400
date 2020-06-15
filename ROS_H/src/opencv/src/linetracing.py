#!/usr/bin/env python
import rospy
import cv2
import numpy as np

'''
***import for line orienation and image comparison
windows =>
    pip install scikit-image
ubuntu =>
    sudo apt-get install python3-matplotlib python3-numpy python3-pil python3-scipy python3-tk
    sudo apt-get install build-essential cython3
'''

from skimage import measure
###################################################

from std_msgs.msg import Float64 ,Int64
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import math



bridge = CvBridge()

cam_distance = 0.25 # 250mm
f_mm = 3.67 / 1000
sensor_width = 4.8 / 1000
pixel_width = 1920
f_pixel = f_mm * pixel_width / sensor_width
disparity = 0
img_left = 0
img_right = 0
img_center = np.zeros((1080,1920,3),dtype=np.uint8)


cnt = 0

#################parameters for ball detection ########################
b = [98, 131, 131, 255, 0, 255]  
bC = [1, 101, 68, 8, 0, 68, 1]	# parameter related to circles (dp,minDist,param1,param2,minRadius,maxRadius,BlurSize)

i=0;
globalStart=0.6;
globalEnd=0.98

'''
CENTER CAMERA HSV PARAMS
'''  

regionFlag=Int64()
holderFlag=Int64()
xc=Int64()
orientation = Float64()
eigenImg = np.zeros((512,512,3),dtype=np.uint8)
eigenImg2= np.zeros((512,512,3),dtype=np.uint8)


#high hue, low hue, high calue, low value, high sat, low sat

#non holder ball harvesting color mask
blueMask=[98,131,131,255,0,255]
greenMask=[50,115,131,255,8,252]
redMask = [142,180,99,255,8,252];redMask_=[0,4,99,255,8,252]

'''
#holder in harvesting region color mask 
redMaskHolder = [132,180,145,255,140,170];
redMaskHolder_=[0,17,145,255,140,170]
blueMaskHolder=[92,133,133,255,184,255]
'''

'''
LEFT , RIGHT CAMERA HSV & HOUGH PARAMS

'''


def adjustOrientationValue(val):
    '''
    1.make north be the zero
    2.make the range from west(pi/2) to east(-pi/2)
    
    '''
    if val >= 0 : return val - math.pi/2
    else : return val + math.pi /2 


def compareMSE(imageA, imageB):
    m = mse(imageA, imageB)
    return m
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
def ballHolderState(img,thresh=1):
    '''
    thresh = > maximum range for displacement from center
    outputs  
        0=> if no ball in holder
        1=> if blue ball in holder
        2=> if red ball in holder
    '''
    
   
    def compareHolder(binaryImg, x= 960 , y = 1300, radius = 450 ):
        yy,xx= binaryImg.shape
        f =np.zeros((yy,xx,3),dtype=np.uint8)
        mask = cv2.circle(f.copy(),(x,y ),radius,(255,255,255),-1);mask=mask[:,:,0]
        
        #print(mask.shape)
        #print(img.shape)
        maskAndImg= cv2.bitwise_and(mask,binaryImg)
        
        
        maskWhitePixels = float(cv2.countNonZero(mask))
        maskAndImgWhitePixels= float(cv2.countNonZero(maskAndImg))
        print(maskAndImgWhitePixels  / maskWhitePixels)
        return maskAndImgWhitePixels  / maskWhitePixels   ,  maskAndImg 




    blueMask=[98,131,131,255,0,255]
    greenMask=[50,115,131,255,8,252]
    redMask = [142,180,99,255,8,252];redMask_=[0,4,99,255,8,252]

    redMaskHSV = selectSection(getColorMask(img.copy(),redMask,redMask_),0.5,1)
    blueMaskHSV = selectSection(getColorMask(img.copy(),blueMask),0.5,1)

    ratioBlue ,andMask = compareHolder(blueMaskHSV) ; 


    if ratioBlue >=thresh : print("true blue") ;return 1,blueMaskHSV,andMask 
    else :
        ratioRed,andMask = compareHolder(redMaskHSV)
        print("red ration", ratioRed,ratioRed >= thresh)
        if ratioRed >= thresh : 
            print("true red");
            return 2,redMaskHSV,andMask
        else : return 0, np.zeros((1,1,3),dtype=np.uint8),np.zeros((1,1,3),dtype=np.uint8)


    '''
    imgCenter=img.shape[1]//2

    ##check if red ball is in holder
    redBallMask = selectSection(getColorMask(img,rp,rp_),0.8,1,0.3,0.7)
    if cv2.countNonZero(redBallMask) > 0 :
        props = getProp(redBallMask)
        if abs(imgCenter-props.centroid[1]) < thresh : return 2
    
    ##check if blue ball is in holder
    blueBallMask = selectSection(getColorMask(img,bp),0.8,1,0.3,0.7)
    if cv2.countNonZero(blueBallMask) > 0 :
        props = getProp(blueBallMask)
        if abs(imgCenter-props.centroid[1]) < thresh : return 1
    
    #No ball in holder
    return 0
    '''


def regionState(img,thresh):
    '''
    input : 
        thresh => minimum number of colored pixel to declare it's ball region 
        bp,rp,rp_ => hsv parameters list of blue , red , red_
        
    outputs  
        0=> line tracing region
        1=> ball harvesting region
    '''
        
    ##check if red or blue ball is visible
    blueMask=[98,131,131,255,0,255]
    greenMask=[50,115,131,255,8,252]
    redMask = [142,180,99,255,8,252];redMask_=[0,4,99,255,8,252]


    redBallMask = getColorMask(img,redMask,redMask_)
    blueBallMask = getColorMask(img,blueMask)
    
    if cv2.countNonZero(redBallMask) > thresh  or cv2.countNonZero(blueBallMask) > 0 :
        return 1
    
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

def getCentroidOrientation(img,start=0,end=1,flag=0):
    '''
    start portion of image vertically
    end portion of image vertically
    for example start : 0.5 end :1 means get orientation of line from middle to bottom
    
    ***return -1 ,-1 if no line is found

    flag= 0 => no eigen image
    flag =1 => output eigen image
    
    '''

    ret,thresh=cv2.threshold(img,80,255,cv2.THRESH_BINARY_INV);
    props = getProp(selectSection(thresh[:,:,0],start,end))
    
    if props !=[] :
        x,ori =  props.centroid[1]  , adjustOrientationValue(props.orientation)
        if flag == 1 :  #Drawn image
            eigenImg= drawProps(props,selectSection(thresh[:,:,0],start,end)) 
            return x,ori,eigenImg
        elif flag ==0 :  #No image
            return x,ori,np.zeros((1,1,3),dtype=np.uint8)
    else : return -1,-1,np.zeros((1,1,3),dtype=np.uint8)




def drawProps(props,binaryImage):
    if props !=[]:
        image = cv2.cvtColor(binaryImage,cv2.COLOR_GRAY2BGR)
        y0, x0 = props.centroid
        ori = props.orientation
        x1 = x0 + math.cos(ori) * 0.5 * props.minor_axis_length
        y1 = y0 - math.sin(ori) * 0.5 * props.minor_axis_length
        x2 = x0 - math.sin(ori) * 0.5 * props.major_axis_length
        y2 = y0 - math.cos(ori) * 0.5 * props.major_axis_length
        minr, minc, maxr, maxc = props.bbox
        x0,y0,x1,y1,x2,y2 = map(int,[x0,y0,x1,y1,x2,y2])
        image = cv2.circle(image,(x0,y0),1,(0,0,255),-1)   #draw centroid
        image = cv2.line (image , (x0,y0),(x1,y1),(255,0,255),2)
        image = cv2.line (image , (x0,y0),(x2,y2),(255,0,0),2)
        #print(y0,x0,(orientation+ math.pi/2) *180 / math.pi)
        return image
    else :
        return binaryImage




    
def center_circle_callback(data) :
    global regionFlag,holderFlag,xc,orientation
    global blueMask , redMask , redMask_
    global blueMaskHolder , redMaskHolder , redMaskHolder_
    '''
    three tasks 
     1 - detect if the car is in line tracing[=0] or ball harvesting region[=1]
     2 - detect if blue ball[=1] , red ball[=2] or not ball in holder [=0]
     3 - give orientation and horizontal centroid of line 
    '''
    
    global i;

    img_center = bridge.imgmsg_to_cv2(data, "bgr8")
    i+=1
    
    #print(img_center.shape[1]
    #cv2.namedWindow("eig",cv2.WINDOW_NORMAL);cv2.namedWindow("eig2",cv2.WINDOW_NORMAL)

    if regionFlag.data !=1 : regionFlag.data = regionState(img_center,5)
    else : pass #print("now entering ball harvesting")

    key = cv2.waitKey(1) & 0xff
    if key == 27 : cv2.destroyAllWindows()
    
    #line tracing region
    if regionFlag.data == 0 :
        #define start and end of window of capturing######################
        
        #######################
        #cv2.namedWindow("raw",cv2.WINDOW_NORMAL)

        
        #orientation.data= adjustOrientationValue(orientation.data)
        
        xc2,orientation2,outputImage2 = getCentroidOrientation(img_center,0,0.4,1)

        '''
        if there is no line in top half then make orienttation =0 
        '''

        if xc2!=-1 :xc.data,orientation.data,outputImage = getCentroidOrientation(img_center,0.6,0.98,1)
        else : xc.data,orientation.data,outputImage = getCentroidOrientation(img_center,0.81,0.99,1);orientation.data =0

        #cv2.imshow("eig", outputImage) #this line shows the eigen vectors of bottom half - remove it after testing
        
        #cv2.imshow("eig2", outputImage2) #this line shows the eigen vectors of top half - remove it after testing

        
    
    #ball harvesting region
    elif regionFlag.data == 1 :
        holderFlag.data,holderImg,andMask = ballHolderState(img_center)  
        cv2.namedWindow("raw",cv2.WINDOW_NORMAL);cv2.namedWindow("holder",cv2.WINDOW_NORMAL);cv2.namedWindow("andMask",cv2.WINDOW_NORMAL)
        cv2.imshow("holder",holderImg)
        cv2.imshow("raw",img_center)
        cv2.imshow("andMask",andMask)

        key = cv2.waitKey(1)& 0xff
        if key == 27 : cv2.destroyAllWindows()
            # 0=>no ball , 1=>blueball , 2=>redball
            #publish holder state , region

    
    
if __name__ == '__main__':
    left_blue_ball = np.array([], dtype=np.float32)
    right_blue_ball = np.array([], dtype=np.float32)

#   dist_pub = rospy.Publisher("/ball_distance", Float64, queue_size = 10)
#   rospy.Subscriber("/camera_left/rgb/image_raw", Image, left_circle_callback)
#   rospy.Subscriber("/camera_right/rgb/image_raw", Image, right_circle_callback)
    rospy.Subscriber("/camera_center/rgb/image_raw", Image,center_circle_callback )
    
    ####line tracing publishers######################################################
    regionFlag_pub = rospy.Publisher("/regionFlag", Int64, queue_size = 10)
    holderFlag_pub = rospy.Publisher("/holderFlag", Int64, queue_size = 10)
    xc_pub = rospy.Publisher("/line/centroid", Int64, queue_size = 10)
    orientation_pub = rospy.Publisher("/line/orientation", Float64, queue_size = 10)
    ##################################################################################
    
    rospy.init_node("OpenCV_Node")

    rate = rospy.Rate(20)

 

while not rospy.is_shutdown():
  
  
  ########################line tracing code###################################################  
  #print(-xc.data+img_center.shape[1]//2,(orientation.data)*180/math.pi)  
  regionFlag_pub.publish(regionFlag)
  holderFlag_pub.publish(holderFlag)
  xc_pub.publish(xc)
  orientation_pub.publish(orientation)
  ############################################################################################
  
  
  
  rate.sleep()
