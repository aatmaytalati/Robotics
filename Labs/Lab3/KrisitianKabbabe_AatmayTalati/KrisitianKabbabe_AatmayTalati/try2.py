# Kristian Kabbabe - Cozmo# 69
# Aatmay Talati - Cozmo# 72
import asyncio

import cozmo
from cozmo.util import degrees, distance_mm


def drive_to_cube(robot: cozmo.robot.Robot):
    found = False
    while not found:
        robot.move_lift(-3)
        robot.set_head_angle(degrees(0)).wait_for_completed()
        #look_around = robot.start_behavior(cozmo.behavior.BehaviorTypes.LookAroundInPlace)
        cube = None
        try:
            cube = robot.world.wait_for_observed_light_cube(timeout=0)
        except asyncio.TimeoutError:
            print("Didn't find a cube")
            break
        finally:
            #look_around.stop()


cozmo.run_program(drive_to_cube, use_viewer=True, force_viewer_on_top=True)
