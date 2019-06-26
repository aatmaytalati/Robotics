#!/usr/bin/env python3
# Team Member - 1 : Aatmay S. Talati
# Team Member - 2 : Kristian Kabbabe
# Team Member - 3 : Min Ho Lee

import cv2
import cozmo
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
import cozmo.util
# camera params
camK = np.matrix([[295, 0, 160], [0, 295, 120], [0, 0, 1]], dtype='float32')

#marker size in inches
marker_size = 3.5

# tmp cache
last_pose = cozmo.util.Pose(0,0,0,angle_z=cozmo.util.Angle(degrees=0))

# goal location for the robot to drive to, (x, y, theta)
goal = (6,10,0)

# map
Map_filename = "map_arena.json"

reached = False
done = True

shake = False
facing = False

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
        #print("ID =", marker.id);
        #print(marker.contours);
    cv2.imshow("Markers", opencv_image)

    return markers

#calculate marker pose
def cvt_2Dmarker_measurements(ar_markers):
    
    marker2d_list = []
    
    for m in ar_markers:
        R_1_2, J = cv2.Rodrigues(m.rvec)
        R_1_1p = np.matrix([[0,0,1], [0,-1,0], [1,0,0]])
        R_2_2p = np.matrix([[0,-1,0], [0,0,-1], [1,0,0]])
        R_2p_1p = np.matmul(np.matmul(inv(R_2_2p), inv(R_1_2)), R_1_1p)
        #print('\n', R_2p_1p)
        yaw = -math.atan2(R_2p_1p[2,0], R_2p_1p[0,0])
        
        x, y = m.tvec[2][0] + 0.5, -m.tvec[0][0]
        # print('x =', x, 'y =', y,'theta =', yaw)
        
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
    global last_pose
    last_x, last_y, last_h = last_pose.position.x, last_pose.position.y, \
        last_pose.rotation.angle_z.degrees
    curr_x, curr_y, curr_h = curr_pose.position.x, curr_pose.position.y, \
        curr_pose.rotation.angle_z.degrees

    if cvt_inch:
        last_x, last_y = last_x / 25.6, last_y / 25.6
        curr_x, curr_y = curr_x / 25.6, curr_y / 25.6

    return [[last_x, last_y, last_h],[curr_x, curr_y, curr_h]]

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
    global last_pose
    global grid, gui
    global shake, facing, reached
    # start streaming
    robot.camera.image_stream_enabled = True

    #start particle filter
    pf = ParticleFilter(grid)

    ############################################################################
    ######################### YOUR CODE HERE####################################
    await robot.set_head_angle(cozmo.util.degrees(7)).wait_for_completed()

    while True:
        pose = robot.pose
        odom = compute_odometry(pose)
        last_pose = pose

        markers = await image_processing(robot)
        markers2D = cvt_2Dmarker_measurements(markers)
        cv2.waitKey(1)

        updated_pf = pf.update(odom, markers2D)

        if robot.is_picked_up: 
            await robot.drive_wheels(0, 0)
            pf = ParticleFilter(grid)

            if shake:
                robot.move_lift(-30)
                shake = False
            else:
                robot.move_lift(30)
                shake = True
            await robot.play_anim_trigger(cozmo.anim.Triggers.PouncePounce).wait_for_completed()

        elif updated_pf[3]:
            await robot.drive_wheels(0, 0)

            ang = math.atan2(goal[1] - updated_pf[1], goal[0] - updated_pf[0]) * 180 / math.pi - updated_pf[2]
            dist = grid_distance(goal[0], updated_pf[0], goal[1], updated_pf[1]) * 25.6

            if dist > 80:
                reached = False
            elif dist < 40:
                reached = True
                print("REACHED?")

            if abs(ang) > 10:
                facing = False

            if reached:
                if updated_pf[2] < 0:
                    await robot.turn_in_place(cozmo.util.degrees(int(updated_pf[2] * -1))).wait_for_completed()
                elif updated_pf[2] > 0:
                    await robot.turn_in_place(cozmo.util.degrees(int(updated_pf[2] * -1))).wait_for_completed()

                if done:
                    await robot.play_anim_trigger(cozmo.anim.Triggers.FistBumpSuccess).wait_for_completed()
                    done = False
                    print("DONE")
            else :
                if facing is False:
                    await robot.turn_in_place(cozmo.util.degrees(ang), in_parallel=True).wait_for_completed()
                    facing = True

                await robot.drive_wheels(30, 30)
                
        else:
            await robot.set_head_angle(cozmo.util.degrees(7), in_parallel=True).wait_for_completed()
            if shake:
                robot.move_lift(-30)
                shake = False
            facing = False
            done= True
            await robot.drive_wheels(20, -20)

        gui.show_particles(pf.particles)
        gui.show_mean(updated_pf[0], updated_pf[1], updated_pf[2], updated_pf[3])
        gui.updated.set()
    await robot.play_anim_trigger(cozmo.anim.Triggers.FistBumpSuccess).wait_for_completed()
    ############################################################################


class CozmoThread(threading.Thread):
    def __init__(self):
        threading.Thread.__init__(self, daemon=False)

    def run(self):
        cozmo.run_program(run, use_viewer=False)


if __name__ == '__main__':

    # cozmo thread
    cozmo_thread = CozmoThread()
    cozmo_thread.start()

    # init
    grid = CozGrid(Map_filename)
    gui = GUIWindow(grid)
    gui.start()