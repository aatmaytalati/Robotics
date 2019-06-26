#!/usr/bin/env python3
#!c:/Python35/python3.exe -u
# Team Members: Kristian Kabbabe & Aatmay S. Talati
import asyncio
import sys
import cv2
import numpy as np
import cozmo
import time
import os
from cozmo.util import degrees, distance_mm, speed_mmps, Angle
from glob import glob

from find_cube import *

try:
    from PIL import ImageDraw, ImageFont
except ImportError:
    sys.exit('run `pip3 install --user Pillow numpy` to run this example')
def nothing(x):
    pass

boo = 1  # Green = 1, Yellow = 0
# 55.5cm away
YELLOW_LOWER = np.array([10, 168, 139])
YELLOW_UPPER = np.array([60, 255, 255])
GREEN_LOWER = np.array([0,0,0])
GREEN_UPPER = np.array([175, 255, 60])

UPPER = GREEN_UPPER*boo + YELLOW_UPPER*(not boo)
LOWER = GREEN_LOWER*boo + YELLOW_LOWER*(not boo)

# Define a decorator as a subclass of Annotator; displays the keypoint
class BoxAnnotator(cozmo.annotate.Annotator):

    cube = None

    def apply(self, image, scale):
        d = ImageDraw.Draw(image)
        bounds = (0, 0, image.width, image.height)

        if BoxAnnotator.cube is not None:

            #double size of bounding box to match size of rendered image
            BoxAnnotator.cube = np.multiply(BoxAnnotator.cube,2)

            #define and display bounding box with params:
            #msg.img_topLeft_x, msg.img_topLeft_y, msg.img_width, msg.img_height
            box = cozmo.util.ImageBox(BoxAnnotator.cube[0]-BoxAnnotator.cube[2]/2,
                                      BoxAnnotator.cube[1]-BoxAnnotator.cube[2]/2,
                                      BoxAnnotator.cube[2], BoxAnnotator.cube[2])
            cozmo.annotate.add_img_box_to_image(image, box, "green", text=None)
            BoxAnnotator.cube = None



async def run(robot: cozmo.robot.Robot):

    robot.world.image_annotator.annotation_enabled = True
    robot.world.image_annotator.add_annotator('box', BoxAnnotator)

    robot.camera.image_stream_enabled = True
    robot.camera.color_image_enabled = True
    robot.camera.enable_auto_exposure = True

    gain,exposure,mode = 2.5,3,1
    ang = 3
    angleView = 0
    aligned = 0
    count = 0
    try:
        while True:
            event = await robot.world.wait_for(cozmo.camera.EvtNewRawCameraImage, timeout=30)   #get camera image
            if event.image is not None:
                image = cv2.cvtColor(np.asarray(event.image), cv2.COLOR_BGR2RGB)

                if mode == 1:
                    robot.camera.enable_auto_exposure = True
                else:
                    robot.camera.set_manual_exposure(exposure,gain)

                #find the cube
                cube = find_cube(image, LOWER, UPPER)
                #print(cube)
                BoxAnnotator.cube = cube
                ################################################################
                # Todo: Add Motion Here
                ################################################################
                # await robot.set_lift_height(1.0).wait_for_completed()
                # await robot.set_head_angle(degrees(-17)).wait_for_completed()
                # await robot.drive_wheels(50, 30) # LEFT, RIGHT Wheels
 
    except KeyboardInterrupt:
        print("")
        print("Exit requested by user")
    except cozmo.RobotBusy as e:
        print(e)
    #cv2.destroyAllWindows()

if __name__ == '__main__':
    cozmo.run_program(run, use_viewer = True, force_viewer_on_top = True)
