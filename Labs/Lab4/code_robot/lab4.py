
#author1: Kaley Findley
#author2: Hong Kim

from grid import *
from visualizer import *
import threading
from queue import PriorityQueue
import math
import cozmo

theta = 0

def astar(grid, heuristic):
	"""Perform the A* search algorithm on a defined grid

		Arguments:
		grid -- CozGrid instance to perform search on
		heuristic -- supplied heuristic function
	"""

	frontier = PriorityQueue()
	start = grid.getStart()
	frontier.put((0, (start, [start], 0)))
	while not frontier.empty():
		item = frontier.get()[1]
		node, actions, pathCost = item
		if isGoalState(grid, node):
			grid.setPath(actions)
			return actions
		if not node in grid.getVisited():
			grid.addVisited(node)
			for next_node, cost in grid.getNeighbors(node):
				if not next_node in grid.getVisited():
					new_actions = actions + [next_node]
					newPathCost = pathCost + cost
					score = newPathCost + heuristic(node, next_node, grid.getGoals())
					# print("NEXT NODE", next_node, cost)
					# print("Heuristic", heuristic(node, next_node, grid.getGoals()))
					frontier.put((score, (next_node, new_actions, newPathCost)))
	return []

def isGoalState(grid, node):
	return node in grid.getGoals()

def heuristic(current,next_node, goal):
	"""Heuristic function for A* algorithm

		Arguments:
		current -- current cell
		goal -- desired goal cell
	"""
	goalNode = goal[0]

	return math.sqrt((next_node[0] - goalNode[0])**2 + (next_node[1] - goalNode[1])**2) # Your code here


def helper(robot, grid):

	global theta
	while grid.getStart() != grid.getGoals()[0]:
		print("CURR: ", grid.getStart(), "GOAL: ", grid.getGoals())
		cube1 = robot.world.light_cubes[cozmo.objects.LightCube1Id]
		cube2 = robot.world.light_cubes[cozmo.objects.LightCube2Id]
		cube3 = robot.world.light_cubes[cozmo.objects.LightCube3Id]


		if cube1.is_visible:
			cubeX = math.ceil((1/25) * (cube1.pose.position.x))
			cubeY = math.ceil((1/25) * (cube1.pose.position.y))	
			faceangle = cube1.pose.rotation.angle_z.degrees
			grid.clearVisited()
			grid.clearGoals()
			print("ANGLE::::::: ", faceangle)
			if faceangle < 30 and faceangle > -30:
				print("Angle btw -30 and 30")
				print("robot angle", robot.pose_angle.degrees)
				cubeX -= 2
				cubeY += 2
				if robot.pose_angle.degrees < faceangle:
					theta = robot.pose_angle.degrees - faceangle
				else:
					theta = faceangle - robot.pose_angle.degrees
			elif faceangle <= -30 and faceangle >= -120:
				print("Angle btw -30 and -120")
				print("robot angle", robot.pose_angle.degrees)
				grid.addObstacle((cubeX, cubeY))
				xPlus = math.ceil((1/25) * (cube1.pose.position.x))
				yPlus = math.ceil((1/25) * (cube1.pose.position.y))
				print("Robot's X: ", robot.pose.position.x, " Robot's Y: ", robot.pose.position.y)
				print("X: ", xPlus, "Y: ", yPlus)	
				grid.addObstacle((xPlus, yPlus))
				grid.addObstacle((xPlus - 1, yPlus))
				grid.addObstacle((xPlus + 1, yPlus))
				grid.addObstacle((xPlus, yPlus - 1))
				grid.addObstacle((xPlus, yPlus + 1))
				grid.addObstacle((xPlus - 1, yPlus -1))
				grid.addObstacle((xPlus - 1, yPlus + 1))
				grid.addObstacle((xPlus + 1, yPlus - 1))
				grid.addObstacle((xPlus + 1, yPlus + 1))

				grid.addObstacle((xPlus - 2, yPlus))
				grid.addObstacle((xPlus - 2, yPlus - 1))
				grid.addObstacle((xPlus - 2, yPlus - 2))
				grid.addObstacle((xPlus - 2, yPlus + 1))
				grid.addObstacle((xPlus - 2, yPlus + 2))
				grid.addObstacle((xPlus - 1, yPlus + 2))
				grid.addObstacle((xPlus, yPlus + 2))
				grid.addObstacle((xPlus + 1, yPlus + 2))
				grid.addObstacle((xPlus + 2, yPlus + 2))
				grid.addObstacle((xPlus + 2, yPlus + 1))
				grid.addObstacle((xPlus + 2, yPlus))
				grid.addObstacle((xPlus + 2, yPlus - 1))
				grid.addObstacle((xPlus + 2, yPlus - 2))
				grid.addObstacle((xPlus + 1, yPlus - 2))
				grid.addObstacle((xPlus, yPlus - 2))
				grid.addObstacle((xPlus - 1, yPlus - 2))

				grid.addObstacle((xPlus + 3, yPlus + 2))
				grid.addObstacle((xPlus + 3, yPlus + 1))
				grid.addObstacle((xPlus + 3, yPlus))
				grid.addObstacle((xPlus + 3, yPlus - 1))
				grid.addObstacle((xPlus + 3, yPlus - 2))
				grid.addObstacle((xPlus + 3, yPlus - 3))
				grid.addObstacle((xPlus + 2, yPlus - 3))
				grid.addObstacle((xPlus + 1, yPlus - 3))
				grid.addObstacle((xPlus, yPlus - 3))
				grid.addObstacle((xPlus - 1, yPlus - 3))
				grid.addObstacle((xPlus - 2, yPlus - 3))

				grid.addObstacle((xPlus - 3, yPlus - 3))
				grid.addObstacle((xPlus - 3, yPlus - 2))
				grid.addObstacle((xPlus - 3, yPlus - 1))
				grid.addObstacle((xPlus - 3, yPlus))
				grid.addObstacle((xPlus - 3, yPlus + 1))
				grid.addObstacle((xPlus - 3, yPlus + 2))
				grid.addObstacle((xPlus - 3, yPlus + 3))

				grid.addObstacle((xPlus - 3, yPlus + 3))
				grid.addObstacle((xPlus - 2, yPlus - 3))
				grid.addObstacle((xPlus - 1, yPlus - 3))
				grid.addObstacle((xPlus, yPlus - 3))
				grid.addObstacle((xPlus + 1, yPlus - 3))
				grid.addObstacle((xPlus + 2, yPlus - 3))
				grid.addObstacle((xPlus + 3, yPlus - 3))


				cubeX += 4
				cubeY += 3
				if robot.pose_angle.degrees > faceangle:
					theta = faceangle - robot.pose_angle.degrees
				else:
					theta = robot.pose_angle.degrees - faceangle
							   
			elif faceangle >= 30 and faceangle < 90: 
				print("Angle btw 30 and 90")
				print("robot angle", robot.pose_angle.degrees)
				cubeX += 1
				cubeY -= 4
				if robot.pose_angle.degrees < faceangle:
					theta = faceangle - robot.pose_angle.degrees
				elif (faceangle < 60 and faceangle > 40) and (robot.pose_angle.degrees < 60 and robot.pose_angle.degrees > 40):
					theta = faceangle
				else:
					theta = robot.pose_angle.degrees - faceangle 
			elif faceangle >= 90 and faceangle < 120: 
				print("Angle btw 90 and 120")
				print("robot angle", robot.pose_angle.degrees)
				cubeY -= 3
				cubeX += 3
				if robot.pose_angle.degrees < faceangle:
					theta = faceangle - robot.pose_angle.degrees
				else:
					theta = robot.pose_angle.degrees - faceangle 
			elif faceangle >= 120 and faceangle < 180:
					print("Angle btw 120 and 179")
					print("robot angle", robot.pose_angle.degrees)
					cubeX += 6
					cubeY -= 3
					if robot.pose_angle.degrees < faceangle:
						theta = faceangle - robot.pose_angle.degrees
					else:
						theta = robot.pose_angle.degrees - faceangle

			elif faceangle >= -180 and faceangle < -120:
				print("Angle btw -120 and -180")
				grid.addObstacle((cubeX, cubeY))
				xPlus = math.ceil((1/25) * (cube1.pose.position.x))
				yPlus = math.ceil((1/25) * (cube1.pose.position.y))
				print("Robot's X: ", robot.pose.position.x, " Robot's Y: ", robot.pose.position.y)
				print("robot angle", robot.pose_angle.degrees)
				print("X: ", xPlus, "Y: ", yPlus)	
				grid.addObstacle((xPlus - 1, yPlus))
				grid.addObstacle((xPlus + 1, yPlus))
				grid.addObstacle((xPlus, yPlus - 1))
				grid.addObstacle((xPlus, yPlus + 1))
				grid.addObstacle((xPlus - 1, yPlus -1))
				grid.addObstacle((xPlus - 1, yPlus + 1))
				grid.addObstacle((xPlus + 1, yPlus - 1))
				grid.addObstacle((xPlus + 1, yPlus + 1))

				grid.addObstacle((xPlus - 2, yPlus))
				grid.addObstacle((xPlus - 2, yPlus - 1))
				grid.addObstacle((xPlus - 2, yPlus - 2))
				grid.addObstacle((xPlus - 2, yPlus + 1))
				grid.addObstacle((xPlus - 2, yPlus + 2))
				grid.addObstacle((xPlus - 1, yPlus + 2))
				grid.addObstacle((xPlus, yPlus + 2))
				grid.addObstacle((xPlus + 1, yPlus + 2))
				grid.addObstacle((xPlus + 2, yPlus + 2))
				grid.addObstacle((xPlus + 2, yPlus + 1))
				grid.addObstacle((xPlus + 2, yPlus))
				grid.addObstacle((xPlus + 2, yPlus - 1))
				grid.addObstacle((xPlus + 2, yPlus - 2))
				grid.addObstacle((xPlus + 1, yPlus - 2))
				grid.addObstacle((xPlus, yPlus - 2))
				grid.addObstacle((xPlus - 1, yPlus - 2))

				grid.addObstacle((xPlus + 3, yPlus + 2))
				grid.addObstacle((xPlus + 3, yPlus + 1))
				grid.addObstacle((xPlus + 3, yPlus))
				grid.addObstacle((xPlus + 3, yPlus - 1))
				grid.addObstacle((xPlus + 3, yPlus - 2))
				grid.addObstacle((xPlus + 3, yPlus - 3))
				grid.addObstacle((xPlus + 2, yPlus - 3))
				grid.addObstacle((xPlus + 1, yPlus - 3))
				grid.addObstacle((xPlus, yPlus - 3))
				grid.addObstacle((xPlus - 1, yPlus - 3))
				grid.addObstacle((xPlus - 2, yPlus - 3))

				grid.addObstacle((xPlus - 3, yPlus - 3))
				grid.addObstacle((xPlus - 3, yPlus - 2))
				grid.addObstacle((xPlus - 3, yPlus - 1))
				grid.addObstacle((xPlus - 3, yPlus))
				grid.addObstacle((xPlus - 3, yPlus + 1))
				grid.addObstacle((xPlus - 3, yPlus + 2))
				grid.addObstacle((xPlus - 3, yPlus + 3))

				grid.addObstacle((xPlus - 3, yPlus + 3))
				grid.addObstacle((xPlus - 2, yPlus - 3))
				grid.addObstacle((xPlus - 1, yPlus - 3))
				grid.addObstacle((xPlus, yPlus - 3))
				grid.addObstacle((xPlus + 1, yPlus - 3))
				grid.addObstacle((xPlus + 2, yPlus - 3))
				grid.addObstacle((xPlus + 3, yPlus - 3))

				cubeX += 5
				cubeY += 3
				if robot.pose_angle.degrees < faceangle:
					theta = faceangle - robot.pose_angle.degrees
				else:
					theta = robot.pose_angle.degrees - faceangle
			grid.addGoal((cubeX, cubeY))
		if cube2.is_visible:
			# print("X: ", cube2.pose.position.x, "Y: ", cube2.pose.position.y)
			# robot.pose = cozmo.util.Pose.define_pose_relative_this(cozmo.util.Pose(0,0,0, angle_z = cozmo.util.Angle(0), origin_id=0))
			# cozmo.util.Pose(0,0,0)
			xPlus = math.ceil((1/25) * (cube2.pose.position.x))
			yPlus = math.ceil((1/25) * (cube2.pose.position.y))
			print("Robot's X: ", robot.pose.position.x, " Robot's Y: ", robot.pose.position.y)
			print("X: ", xPlus, "Y: ", yPlus)	
			grid.addObstacle((xPlus - 1, yPlus))
			grid.addObstacle((xPlus + 1, yPlus))
			grid.addObstacle((xPlus, yPlus - 1))
			grid.addObstacle((xPlus, yPlus + 1))
			grid.addObstacle((xPlus - 1, yPlus -1))
			grid.addObstacle((xPlus - 1, yPlus + 1))
			grid.addObstacle((xPlus + 1, yPlus - 1))
			grid.addObstacle((xPlus + 1, yPlus + 1))

			grid.addObstacle((xPlus - 2, yPlus))
			grid.addObstacle((xPlus - 2, yPlus - 1))
			grid.addObstacle((xPlus - 2, yPlus - 2))
			grid.addObstacle((xPlus - 2, yPlus + 1))
			grid.addObstacle((xPlus - 2, yPlus + 2))
			grid.addObstacle((xPlus - 1, yPlus + 2))
			grid.addObstacle((xPlus, yPlus + 2))
			grid.addObstacle((xPlus + 1, yPlus + 2))
			grid.addObstacle((xPlus + 2, yPlus + 2))
			grid.addObstacle((xPlus + 2, yPlus + 1))
			grid.addObstacle((xPlus + 2, yPlus))
			grid.addObstacle((xPlus + 2, yPlus - 1))
			grid.addObstacle((xPlus + 2, yPlus - 2))
			grid.addObstacle((xPlus + 1, yPlus - 2))
			grid.addObstacle((xPlus, yPlus - 2))
			grid.addObstacle((xPlus - 1, yPlus - 2))

			grid.addObstacle((xPlus + 3, yPlus + 2))
			grid.addObstacle((xPlus + 3, yPlus + 1))
			grid.addObstacle((xPlus + 3, yPlus))
			grid.addObstacle((xPlus + 3, yPlus - 1))
			grid.addObstacle((xPlus + 3, yPlus - 2))
			grid.addObstacle((xPlus + 3, yPlus - 3))
			grid.addObstacle((xPlus + 2, yPlus - 3))
			grid.addObstacle((xPlus + 1, yPlus - 3))
			grid.addObstacle((xPlus, yPlus - 3))
			grid.addObstacle((xPlus - 1, yPlus - 3))
			grid.addObstacle((xPlus - 2, yPlus - 3))

			grid.addObstacle((xPlus - 3, yPlus - 3))
			grid.addObstacle((xPlus - 3, yPlus - 2))
			grid.addObstacle((xPlus - 3, yPlus - 1))
			grid.addObstacle((xPlus - 3, yPlus))
			grid.addObstacle((xPlus - 3, yPlus + 1))
			grid.addObstacle((xPlus - 3, yPlus + 2))
			grid.addObstacle((xPlus - 3, yPlus + 3))

			grid.addObstacle((xPlus - 3, yPlus + 3))
			grid.addObstacle((xPlus - 2, yPlus - 3))
			grid.addObstacle((xPlus - 1, yPlus - 3))
			grid.addObstacle((xPlus, yPlus - 3))
			grid.addObstacle((xPlus + 1, yPlus - 3))
			grid.addObstacle((xPlus + 2, yPlus - 3))
			grid.addObstacle((xPlus + 3, yPlus - 3))

		if cube3.is_visible:
			xPlus = math.ceil((1/25) * (cube3.pose.position.x))
			yPlus = math.ceil((1/25) * (cube3.pose.position.y))
			print("Robot's X: ", robot.pose.position.x, " Robot's Y: ", robot.pose.position.y)
			print("X: ", xPlus, "Y: ", yPlus)
			grid.addObstacle((xPlus - 1, yPlus))
			grid.addObstacle((xPlus + 1, yPlus))
			grid.addObstacle((xPlus, yPlus - 1))
			grid.addObstacle((xPlus, yPlus + 1))
			grid.addObstacle((xPlus - 1, yPlus -1))
			grid.addObstacle((xPlus - 1, yPlus + 1))
			grid.addObstacle((xPlus + 1, yPlus - 1))
			grid.addObstacle((xPlus + 1, yPlus + 1))

			grid.addObstacle((xPlus - 2, yPlus))
			grid.addObstacle((xPlus - 2, yPlus - 1))
			grid.addObstacle((xPlus - 2, yPlus - 2))
			grid.addObstacle((xPlus - 2, yPlus + 1))
			grid.addObstacle((xPlus - 2, yPlus + 2))
			grid.addObstacle((xPlus - 1, yPlus + 2))
			grid.addObstacle((xPlus, yPlus + 2))
			grid.addObstacle((xPlus + 1, yPlus + 2))
			grid.addObstacle((xPlus + 2, yPlus + 2))
			grid.addObstacle((xPlus + 2, yPlus + 1))
			grid.addObstacle((xPlus + 2, yPlus))
			grid.addObstacle((xPlus + 2, yPlus - 1))
			grid.addObstacle((xPlus + 2, yPlus - 2))
			grid.addObstacle((xPlus + 1, yPlus - 2))
			grid.addObstacle((xPlus, yPlus - 2))
			grid.addObstacle((xPlus - 1, yPlus - 2))

			grid.addObstacle((xPlus + 3, yPlus + 2))
			grid.addObstacle((xPlus + 3, yPlus + 1))
			grid.addObstacle((xPlus + 3, yPlus))
			grid.addObstacle((xPlus + 3, yPlus - 1))
			grid.addObstacle((xPlus + 3, yPlus - 2))
			grid.addObstacle((xPlus + 3, yPlus - 3))
			grid.addObstacle((xPlus + 2, yPlus - 3))
			grid.addObstacle((xPlus + 1, yPlus - 3))
			grid.addObstacle((xPlus, yPlus - 3))
			grid.addObstacle((xPlus - 1, yPlus - 3))
			grid.addObstacle((xPlus - 2, yPlus - 3))

			grid.addObstacle((xPlus - 3, yPlus - 3))
			grid.addObstacle((xPlus - 3, yPlus - 2))
			grid.addObstacle((xPlus - 3, yPlus - 1))
			grid.addObstacle((xPlus - 3, yPlus))
			grid.addObstacle((xPlus - 3, yPlus + 1))
			grid.addObstacle((xPlus - 3, yPlus + 2))
			grid.addObstacle((xPlus - 3, yPlus + 3))

			grid.addObstacle((xPlus - 3, yPlus + 3))
			grid.addObstacle((xPlus - 2, yPlus - 3))
			grid.addObstacle((xPlus - 1, yPlus - 3))
			grid.addObstacle((xPlus, yPlus - 3))
			grid.addObstacle((xPlus + 1, yPlus - 3))
			grid.addObstacle((xPlus + 2, yPlus - 3))
			grid.addObstacle((xPlus + 3, yPlus - 3))


		path = astar(grid, heuristic)
		if path == []:
			print("THISHITSHSITHISTHISHTIS")
			return

		currentPoint = grid.getStart()
		nextPoint = path[1]
		if (nextPoint[0] - currentPoint[0]) == 0:
			#drive up
			angle = robot.pose_angle.degrees
			turn = 90 - angle
			robot.turn_in_place(cozmo.util.degrees(turn)).wait_for_completed()
			robot.drive_straight(cozmo.util.distance_mm(27), cozmo.util.speed_mmps(25)).wait_for_completed()

		elif (nextPoint[1] - currentPoint[1]) == 0:
			#drive to the right
			angle = robot.pose_angle.degrees
			turn = 0 - angle
			robot.turn_in_place(cozmo.util.degrees(turn)).wait_for_completed()
			robot.drive_straight(cozmo.util.distance_mm(27), cozmo.util.speed_mmps(25)).wait_for_completed()

		elif (nextPoint[1] < currentPoint[1]):
			#turn diagonally up
			angle = robot.pose_angle.degrees
			turn = -45 - angle
			robot.turn_in_place(cozmo.util.degrees(turn)).wait_for_completed()
			robot.drive_straight(cozmo.util.distance_mm(37), cozmo.util.speed_mmps(25)).wait_for_completed()

		else:
			#turn diagonally down
			angle = robot.pose_angle.degrees
			turn = 45 - angle
			robot.turn_in_place(cozmo.util.degrees(turn)).wait_for_completed()
			robot.drive_straight(cozmo.util.distance_mm(37), cozmo.util.speed_mmps(25)).wait_for_completed()

		grid.setStart(nextPoint)
		grid.clearVisited()	


def cozmoBehavior(robot: cozmo.robot.Robot):
	"""Cozmo search behavior. See assignment document for details

		Has global access to grid, a CozGrid instance created yMinus the main thread, and
		stopevent, a threading.Event instance used to signal when the main thread has stopped.
		You can use stopevent.is_set() to check its status or stopevent.wait() to wait for the
		main thread to finish.

		Arguments:
		robot -- cozmo.robot.Robot instance, supplied yMinus cozmo.run_program
	"""
		
	global grid, stopevent, theta

	for x in range(26):
		grid.addObstacle((x, 0))
		grid.addObstacle((x, 17))
	for y in range(18):
		grid.addObstacle((0, y))
		grid.addObstacle((25, y))

	robot.set_head_angle(cozmo.util.degrees(-10)).wait_for_completed()
	robot.set_lift_height(1).wait_for_completed()

	cube1 = robot.world.light_cubes[cozmo.objects.LightCube1Id]
	cube2 = robot.world.light_cubes[cozmo.objects.LightCube2Id]
	cube3 = robot.world.light_cubes[cozmo.objects.LightCube3Id]
	# print("C1:", cube1.pose.position)
	# print("C2:", cube2.pose.position)
	# print("C3:", cube3.pose.position)
	if not cube1.is_visible:
		robot.turn_in_place(cozmo.util.degrees(35)).wait_for_completed()
		#have to go to center
		# if not cube2.is_visible and not cube3.is_visible:
		# 	robot.drive_wheels(l_wheel_speed=30, r_wheel_speed=30,duration=10.0)
		# else:

		grid.addGoal((13,9)) #center is goal
		helper(robot, grid)
		
	else:
		print("@!!!!@!@!@!@!@!@!@!@!@!")
		print("Robot's X: ", robot.pose.position.x, " Robot's Y: ", robot.pose.position.y)
		cubeX = math.ceil((1/25) * (cube1.pose.position.x))
		cubeY = math.ceil((1/25) * (cube1.pose.position.y))
		grid.addGoal((cubeX, cubeY))
		helper(robot, grid)

		if cube2.is_visible:
			grid.addObstacle((cube2.pose.position.x, cube2.pose.position.y))
		if cube3.is_visible:
			grid.addObstacle((cube3.pose.position.x, cube3.pose.position.y))


	cube1 = robot.world.light_cubes[cozmo.objects.LightCube1Id]
	cube2 = robot.world.light_cubes[cozmo.objects.LightCube1Id]
	cube3 = robot.world.light_cubes[cozmo.objects.LightCube1Id]

	if grid.getGoals()[0] == (13, 9):
		while not cube1.is_visible:
			robot.drive_wheels(l_wheel_speed=-20,r_wheel_speed=20)
			cube1 = robot.world.light_cubes[cozmo.objects.LightCube1Id]
		robot.drive_wheels(l_wheel_speed=0,r_wheel_speed=0)
		cubeX = math.ceil((1/25) * (robot.pose.position.x + cube1.pose.position.x))
		cubeY = math.ceil((1/25) * (robot.pose.position.y + cube1.pose.position.y))	
		grid.clearGoals()

		grid.addGoal((cubeX, cubeY))
		helper(robot, grid)	
		cube1 = robot.world.light_cubes[cozmo.objects.LightCube1Id]

	print("THETA", theta)
	if theta is not None:
		robot.turn_in_place(cozmo.util.degrees(theta)).wait_for_completed()

	print("WAHTSISTHISs", cube1.pose.rotation.angle_z)
	return
	# if not cube:
	# 	print("hhii")
	# 	#go to middle of arena
	# 	theta = math.atan(7/10)
	# 	print(theta)
	# 	robot.turn_in_place(cozmo.util.degrees(35)).wait_for_completed()
	# robot.drive_wheels(l_wheel_speed=30, r_wheel_speed=30,duration=10.0)




	# while not stopevent.is_set():
	# 	#print("hi")
	# 	cube1 = robot.world.light_cubes[cozmo.objects.LightCube1Id]
	# 	#print("CUBE",cube)
	# 	if not cube1.is_visible:
	# 		robot.drive_wheels(l_wheel_speed = -20, r_wheel_speed = 20)
	# 	else:
	# 		grid.addGoal((cube1.pose.position.x, cube1.pose.position.y))
	# 		robot.drive_wheels(l_wheel_speed = 0, r_wheel_speed = 0)



######################## DO NOT MODIFY CODE BELOW THIS LINE ####################################


class RobotThread(threading.Thread):
	"""Thread to run cozmo code separate from main thread
	"""
		
	def __init__(self):
		threading.Thread.__init__(self, daemon=True)

	def run(self):
		cozmo.run_program(cozmoBehavior)


# If run as executable, start RobotThread and launch visualizer with empty grid file
if __name__ == "__main__":
	global grid, stopevent
	stopevent = threading.Event()
	grid = CozGrid("emptygrid.json")
	visualizer = Visualizer(grid)
	updater = UpdateThread(visualizer)
	updater.start()
	robot = RobotThread()
	robot.start()
	visualizer.start()
	stopevent.set()

