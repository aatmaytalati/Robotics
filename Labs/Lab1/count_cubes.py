import cv2 #importing cv2 -> 10 points
import numpy as np #numpy has alias np 

#TODO: Modify these values for yellow color range. Add thresholds for detecting green also.
#                         B,  G , R
yellow_lower = np.array([10, 168, 141])  # change in some rangee with given shadow in the image - TA Help
yellow_upper = np.array([60, 255, 255])  #    

# # Thresholds for Detecting Green color. 
green_lower = np.array([0,0,0])   #checked via using http://colorizer.org/ 
green_upper = np.array([175,255,60])   #checked via using http://colorizer.org/  
# Together these two makes hsv_lower and hsv_upper


#TODO: Change this function so that it filters the image based on color using the hsv range for each color.
def filter_image(img, hsv_lower, hsv_upper): #BGR 

# openCV documentations : http://opencv-python-tutroals.readthedocs.io/en/latest/py_tutorials/py_imgproc/py_colorspaces/py_colorspaces.html
 # And Class Lecture slides 54,59 from 8_31_Image_Processing_part1.pdf

    # Modify mask
    img = cv2.medianBlur(img, 25)


    #RGB -> HSV
    ImgBGR2HSV = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    
    # HSV image parameters -> HSV 
    # hsv_lower = cv2.cvtColor(hsv_lower, cv2.COLOR_BGR2HSV)[0][0]
    # hsv_upper = cv2.cvtColor(hsv_upper, cv2.COLOR_BGR2HSV)[0][0]
    # print (hsv_lower)
    # print (hsv_upper)

    mask = cv2.inRange(ImgBGR2HSV, hsv_lower, hsv_upper) #color Filtering 
    result = cv2.bitwise_and(ImgBGR2HSV,ImgBGR2HSV, mask = mask)
    
    #Debugging tool
    # cv2.imshow('img BGR', img)
    # cv2.imshow('img HSV', ImgBGR2HSV)
    # cv2.imshow('masking', mask)
    # cv2.imshow('result', result)
    # cv2.waitKey(0)
    # cv2.destroyAllWindows()

    #Function's Return Value
    return mask


    
#TODO: Change the parameters to make blob detection more accurate. Hint: You might need to set some parameters to specify features such as color, size, and shape. The features have to be selected based on the application. 
def detect_blob(mask):
    img = cv2.medianBlur(mask, 25) #blure uimage no more than 5

   # Set up the SimpleBlobdetector with default parameters with specific values.
    #params = None

    #Reference for Filters : StackOver Flow - https://stackoverflow.com/questions/37162704/detecting-blobs-on-image-with-opencv

    params = cv2.SimpleBlobDetector_Params()
    params.blobColor = 255
    
    
    #Filter By Color
    params.filterByColor = True
    # minArea maxArea not required here. 
   
    #Filter by Area
    params.filterByArea = True
    params.minArea = 255
    params.maxArea = 5000
    
    #Convexity
    params.filterByConvexity = True
    params.minConvexity = 0
    params.maxConvexity = 1
    
    # Circularity 
    params.filterByCircularity = True
    params.minCircularity = 0
    params.maxCircularity = 1
    
    #Interia <- not gonna work in this Lab1
    # params.filterByInteria = False
    # params.minInertiaRatio = 0
    # params.maxInteriaRatio = 1
    
    #Thresholds
    params.minThreshold = 0
    params.maxThreshold = 255
    
    #Bluring the Image
    blur = cv2.GaussianBlur(img,(5,5),0)
    median = cv2.medianBlur(img,5)
    blur2 = cv2.bilateralFilter(img,9,75,75)

    #ADD CODE HERE

    # builds a blob detector with the given parameters 
    detector = cv2.SimpleBlobDetector_create(params)

    # use the detector to detect blobs.
    keypoints = detector.detect(img)

    return len(keypoints)

    
def count_cubes(img):
    
    mask_yellow = filter_image(img, yellow_lower, yellow_upper)
    num_yellow = detect_blob(mask_yellow)

    #TODO: Modify to return number of detected cubes for both yellow and green (instead of 0)

    # Doing it for Green. Same code we see up there, we just need to put vars and parameters as green. 
    # We also modified 1st TODO as green. 
    mask_green = filter_image(img, green_lower, green_upper)
    num_green = detect_blob(mask_green)
    
    
    return num_yellow, num_green #returning green along with along with yellow. 