#!/usr/bin/env python3

#author 1: Daniel Albuquerque
#author 2: Olatide Omojaro

''' Get a raw frame from camera and display in OpenCV
By press space, save the image from 001.bmp to ...
'''

import cv2
import cozmo
import cozmo.util
import numpy as np
from numpy.linalg import inv
import threading
import time

from ar_markers.hamming.detect import detect_markers

from grid import CozGrid
from gui import GUIWindow
from particle import Particle, Robot
from setting import *
from particle_filter import *
from utils import *


# camera params
camK = np.matrix([[295, 0, 160], [0, 295, 120], [0, 0, 1]], dtype='float32')

#marker size in inches
marker_size = 3.5

# tmp cache
last_pose = cozmo.util.Pose(0,0,0,angle_z=cozmo.util.Angle(degrees=0))
flag_odom_init = False

# goal location for the robot to drive to, (x, y, theta)
goal = (6,10,0)

# map
Map_filename = "map_arena.json"
grid = CozGrid(Map_filename)
gui = GUIWindow(grid)

#Stop variable to help with debug
stop = False
stop_count = 0
stop_threshold = 10
debug = True

lift_up = False
facing_goal = False
at_goal = False
just_got_to_goal = True



async def image_processing(robot):

    global camK, marker_size

    event = await robot.world.wait_for(cozmo.camera.EvtNewRawCameraImage, timeout=30)

    # convert camera image to opencv format
    opencv_image = np.asarray(event.image)
    
    # detect markers
    markers = detect_markers(opencv_image, marker_size, camK)
    
    # show markers
    for marker in markers:
        marker.highlite_marker(opencv_image, draw_frame=True, camK=camK)
        # print("ID =", marker.id)
        # print(marker.contours);
    cv2.imshow("Markers", opencv_image)
    # cv2.waitKey(1)

    return markers

#calculate marker pose
def cvt_2Dmarker_measurements(ar_markers):
    
    marker2d_list = [];
    
    for m in ar_markers:
        R_1_2, J = cv2.Rodrigues(m.rvec)
        R_1_1p = np.matrix([[0,0,1], [0,-1,0], [1,0,0]])
        R_2_2p = np.matrix([[0,-1,0], [0,0,-1], [1,0,0]])
        R_2p_1p = np.matmul(np.matmul(inv(R_2_2p), inv(R_1_2)), R_1_1p)
        #print('\n', R_2p_1p)
        yaw = -math.atan2(R_2p_1p[2,0], R_2p_1p[0,0])
        
        x, y = m.tvec[2][0] + 0.5, -m.tvec[0][0]
        if debug: print('x =', x, 'y =', y,'theta =', yaw)
        
        # remove any duplate markers
        dup_thresh = 2.0
        find_dup = False
        for m2d in marker2d_list:
            if grid_distance(m2d[0], m2d[1], x, y) < dup_thresh:
                find_dup = True
                break
        if not find_dup:
            marker2d_list.append((x,y,math.degrees(yaw)))
    return marker2d_list


#compute robot odometry based on past and current pose
def compute_odometry(curr_pose, cvt_inch=True):
    global last_pose, flag_odom_init
    last_x, last_y, last_h = last_pose.position.x, last_pose.position.y, \
        last_pose.rotation.angle_z.degrees
    curr_x, curr_y, curr_h = curr_pose.position.x, curr_pose.position.y, \
        curr_pose.rotation.angle_z.degrees
    
    dx, dy = rotate_point(curr_x-last_x, curr_y-last_y, -last_h)
    if cvt_inch:
        dx, dy = dx / 25.6, dy / 25.6

    return (dx, dy, diff_heading_deg(curr_h, last_h))

#particle filter functionality
class ParticleFilter:

    def __init__(self, grid):
        self.particles = Particle.create_random(PARTICLE_COUNT, grid)
        self.grid = grid

    def update(self, odom, r_marker_list):

        # ---------- Motion model update ----------
        self.particles = motion_update(self.particles, odom)

        # ---------- Sensor (markers) model update ----------
        self.particles = measurement_update(self.particles, r_marker_list, self.grid)

        # ---------- Show current state ----------
        # Try to find current best estimate for display
        m_x, m_y, m_h, m_confident = compute_mean_pose(self.particles)
        return (m_x, m_y, m_h, m_confident)


async def run(robot: cozmo.robot.Robot):

    global flag_odom_init, last_pose
    global grid, gui, stop

    # start streaming
    robot.camera.image_stream_enabled = True

    #start particle filter
    pf = ParticleFilter(grid)

    ############YOUR CODE HERE#################

    robot.set_head_angle(cozmo.util.degrees(7)).wait_for_completed()

    states = ['searching',
              'converged',
              'kidnapped']

    state = states[0]  # start state

    while True and not stop:

        global stop_count, stop, stop_threshold, lift_up, facing_goal, at_goal
        ## obtain odometry information
        # get current pose from robot
        pose = robot.pose
        odom = compute_odometry(pose)
        last_pose = pose
        # print(odom)

        ## Obtain list of currently seen markers and their poses

        markers = await image_processing(robot)
        markers2D = cvt_2Dmarker_measurements(markers)
        cv2.waitKey(1)

        # Update the particle filter using the above information
        estm = pf.update(odom, markers2D)
        print("estm =" , estm)

        ## Update the particle filter GUI for debugging
        # to turn on pf GUI display
        gui.show_particles(pf.particles)
        gui.show_mean(estm[0], estm[1], estm[2], estm[3])
        gui.updated.set()


        ######################### SET STATES ################################
        ## Determin robots actions based on the current state of the localization system
        if robot.is_picked_up:
            state = states[2]
        # if localization has converged, drive to goal Otherwise look around
        elif estm[3] is True:
            # if the particle filter has converged, drive to goal
            state = states[1]
        else:
            state = states[0]


        ######################### CHECK STATES ################################

        if state is states[0]: # S[searching]
            # look for markers to help localize
            print(" --State [0] = Looking for markers-----------------")
            await robot.set_head_angle(cozmo.util.degrees(7), in_parallel=True).wait_for_completed()
            robot.move_lift(-30)
            # await robot.set_lift_height(0, in_parallel=True).wait_for_completed()
            lift_up = False
            facing_goal = False
            just_got_to_goal= True

            await robot.drive_wheels(20, -20)

        elif state is states[1]:  # S[converged]
            # drive to goal
            print(" --State [1] = PF converged, going to Goal-----------------")
            # goal_pose = cozmo.util.pose_z_angle(goal[0], goal[1], 0, cozmo.util.degrees(goal[2]))

            await robot.drive_wheels(0, 0)

            ## compute the angle we need to rotate to face the goal
            dx = goal[0] - estm[0]
            dy = goal[1] - estm[1]

            # find angle b/w Cozmo and the goal in relation to cozmos location (PF loc)
            CozToGoal_ang = math.atan2(dy, dx) * 180 / math.pi

            if debug: print("dy: ", dy, "\ndx: ", dx,"cozmo to goal ang:", CozToGoal_ang)

            turn_ang = CozToGoal_ang - estm[2]

            if abs(turn_ang) > 10:
                facing_goal = False

            if debug: print("cozmo ang:", estm[2], "\nturn ang:", turn_ang)

            # get distance from Cozmo to Goal and convert to mm
            travel_dist = math.sqrt(math.pow(dy,2)+ math.pow(dx,2)) * 25.4

            if travel_dist < 40:
                at_goal = True

            elif travel_dist > 80:
                at_goal = False


            if debug: print("dist to drive:", travel_dist)
            if debug: print("Facing goal: ", facing_goal)

            if at_goal is False:
                if facing_goal is False:
                    #if not facing goal then turn to face the goal
                    await robot.turn_in_place(cozmo.util.degrees(turn_ang), in_parallel=True).wait_for_completed()
                    facing_goal = True

                # travel_dist = cozmo.util.distance_mm(travel_dist)
                # await  robot.drive_straight(travel_dist, speed=cozmo.util.speed_mmps(30),
                #                             should_play_anim=True, in_parallel=True).wait_for_completed()
                await robot.drive_wheels(30, 30)

            else :
                # orient cozmo at the goal to face 0 deg (the goal pose)
                if True:
                    print("-Reached goal, Orienting--")
                    if estm[2] < 0:
                        await robot.turn_in_place(cozmo.util.degrees(int(estm[2] * -1))).wait_for_completed()
                    elif estm[2] > 0:
                        await robot.turn_in_place(cozmo.util.degrees(int(estm[2] * -1))).wait_for_completed()

                # play a success animation
                if just_got_to_goal:
                    await robot.play_anim_trigger(cozmo.anim.Triggers.MeetCozmoFirstEnrollmentCelebration).wait_for_completed()
                    just_got_to_goal = False



        elif state is states[2]:  # S[kidnapped]

            # if cozmo is kidnapped
            print(" --State [2] = Kidnapped, resetting Particle Filter-----------------")
            await robot.drive_wheels(0, 0)
            pf = ParticleFilter(grid)

            #lift arms repeatedly
            if lift_up is True:
                # await robot.set_lift_height(0, duration=0).wait_for_completed()
                robot.move_lift(-30)
                lift_up = False
            else:
                # await robot.set_lift_height(1, duration=0).wait_for_completed()
                robot.move_lift(30)
                lift_up = True

            await robot.play_anim_trigger(cozmo.anim.Triggers.PouncePounce).wait_for_completed()

    # time.sleep(0.05) # needed?
class CozmoThread(threading.Thread):
    
    def __init__(self):
        threading.Thread.__init__(self, daemon=False)

    def run(self):
        cozmo.robot.Robot.drive_off_charger_on_connect = False  # Cozmo can stay on his charger
        cozmo.run_program(run, use_viewer=False)


if __name__ == '__main__':

    # cozmo thread
    cozmo_thread = CozmoThread()
    cozmo_thread.start()

    # init
    grid = CozGrid(Map_filename)
    gui = GUIWindow(grid)
    gui.start()

