# Most updated - FSM
# Kristian Kabbabe
# Aatmay S. Talati

import asyncio
import sys
import cv2
import numpy as np
import cozmo
import time
import os
from cozmo.util import degrees, distance_mm, speed_mmps, Angle
from glob import glob
# from find_cube import *
from transitions import Machine, State
from PIL import ImageDraw, ImageFont, Image

faceImages = []
faceTime = 50

imgs = []

async def LocateCube(robot: cozmo.robot.Robot):
	if robot is not None:
		await robot.say_text("Searching for Cube").wait_for_completed()
		await robot.display_oled_face_image(faceImages[0], faceTime, True).wait_for_completed()
		await robot.drive_wheels(50, -50) #Rotates in circle 
		cv2.imshow('image', imgs[0])
		print("press q to go to GetCloser")
		print("press p to QUIT the program")
		key = cv2.waitKey(25)

		sOut = robot.state
		if key == ord('q'):
			robot.foundCube()
			
		showTrans(sOut, robot.state)
		return robot, key


async def GetCloser(robot: cozmo.robot.Robot):
	if robot is not None:
		await robot.say_text("Found a Cube").wait_for_completed()
		await robot.display_oled_face_image(faceImages[1], faceTime, True).wait_for_completed()
		await robot.drive_wheels(10, 10)
		cv2.imshow('image', imgs[1])
		print("press q to go to LocateCube")
		print("press w to go to Stop")
		print("press e to go to Backup")
		print("press p to QUIT the program")
		key = cv2.waitKey(25)
			
		sOut = robot.state
		if key == ord('q'):
			robot.lostCube()
		elif key == ord('w'):
			robot.correctDistance()
		elif key == ord('e'):
			robot.tooClose()
			
		showTrans(sOut, robot.state)
		return robot, key

async def Backup(robot: cozmo.robot.Robot):
	if robot is not None:
		await robot.say_text("Opps. Too close").wait_for_completed()
		await robot.say_text("I am backing up").wait_for_completed()
		await robot.say_text("Beeeeeep..... Beeeeeep..... Beeeeeep").wait_for_completed()
		await robot.display_oled_face_image(faceImages[2], faceTime, True).wait_for_completed()
		await robot.drive_wheels(-10, -10)
		cv2.imshow('image', imgs[2])
		print("press q to go to LocateCube")
		print("press w to go to Stop")
		print("press e to go to GetCloser")
		print("press p to QUIT the program")
		key = cv2.waitKey(25)
			
		sOut = robot.state
		if key == ord('w'):
			robot.correctDistance()
		elif key == ord('e'):
			robot.tooFar()
		elif key == ord('q'):
			robot.lostCube()

		showTrans(sOut, robot.state)
		return robot, key

async def Stop(robot: cozmo.robot.Robot):
	if robot is not None:
		await robot.say_text("I made it! Yaaaaaaaaaay").wait_for_completed()
		await robot.display_oled_face_image(faceImages[3], faceTime, True).wait_for_completed()
		await robot.drive_wheels(0, 0)
		cv2.imshow('image', imgs[3])
		print("press q to go to LocateCube")
		print("press p to QUIT the program")
		key = cv2.waitKey(25)
			
		sOut = robot.state
		if key == ord('q'):
			robot.lostCube()

		showTrans(sOut, robot.state)
		return robot, key


def showTrans(outS, inS):
	print("[S] ", outS, "-->", inS)


switch = {	"LocateCube" : LocateCube,
			"GetCloser" : GetCloser,
			"Backup" : Backup,
			"Stop" : Stop}

async def run(robot: cozmo.robot.Robot):
	states = [	State(name = 'LocateCube', on_enter=[], on_exit=[]),
				State(name = 'GetCloser', on_enter=[], on_exit=[]),
				State(name = 'Backup', on_enter=[], on_exit=[]),
				State(name = 'Stop', on_enter=[], on_exit=[]),]
	transitions = [	{'trigger':'lostCube','source':['GetCloser','Backup','Stop'],'dest':'LocateCube'},
					{'trigger':'foundCube','source':'LocateCube','dest':'GetCloser'},
					{'trigger':'tooClose','source':'GetCloser','dest':'Backup'},
					{'trigger':'correctDistance','source':['GetCloser', 'Backup'],'dest':'Stop'},
					{'trigger':'tooFar','source':'Backup','dest':'GetCloser'},]
	initState = 'LocateCube'
	machine = Machine(model=robot, states=states, transitions=transitions, initial=initState, ignore_invalid_triggers=True)
	imgStates = [("images/locate.png", Image.BICUBIC),
				("images/getcloser.png", Image.BICUBIC),
				("images/backup.png", Image.BICUBIC),
				("images/stop.png", Image.BICUBIC)]
	for img, smp in imgStates:
		image = Image.open(img)
		resizeImg = image.resize(cozmo.oled_face.dimensions(), smp)
		image = cozmo.oled_face.convert_image_to_screen_data(resizeImg, invert_image=True)
		faceImages.append(image)
		image = cv2.imread(img, 0)
		imgs.append(image)

	while True:
		robot, key = await switch[robot.state](robot)
		if key == ord("p"):
			break
	


if __name__ == '__main__':
	cozmo.run_program(run, use_viewer = True, force_viewer_on_top = True)
