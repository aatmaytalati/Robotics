import cv2
import numpy as np

#TODO: Modify these values for yellow color range. Add thresholds for detecting green also.
yellow_lower = np.array([0, 0, 0])
yellow_upper = np.array([180, 255, 255])


#TODO: Change this function so that it filters the image based on color using the hsv range for each color.
def filter_image(img, hsv_lower, hsv_upper):

    # Modify mask
    mask = None
    return mask

    
#TODO: Change the parameters to make blob detection more accurate. Hint: You might need to set some parameters to specify features such as color, size, and shape. The features have to be selected based on the application. 
def detect_blob(mask):
    img = cv2.medianBlur(mask, 5)

   # Set up the SimpleBlobdetector with default parameters with specific values.
    params = None


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
    return num_yellow, 0

