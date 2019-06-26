# Team Member - 1 : Aatmay S. Talati
# Team Member - 2 : Kristian Kabbabe
# Team Member - 3 : Min Ho Lee

from grid import *
from particle import Particle
from utils import *
from setting import *
import numpy as np
from operator import itemgetter
# ------------------------------------------------------------------------
def motion_update(particles, odom):
    """ Particle filter motion update

        Arguments: 
        particles -- input list of particle represents belief p(x_{t-1} | u_{t-1})
                before motion update
        odom -- noisy odometry measurement, a pair of robot pose, i.e. last time
                step pose and current time step pose

        Returns: the list of particle represents belief \tilde{p}(x_{t} | u_{t})
                after motion update
    """
    pre_x, pre_y, pre_h = odom[0]
    cur_x, cur_y, cur_h = odom[1]
    dx, dy = rotate_point(cur_x - pre_x, cur_y - pre_y, -pre_h)
    dh = diff_heading_deg(cur_h, pre_h)

    for i in range(len(particles)):
        px, py, ph = particles[i].xyh
        noise_x = add_gaussian_noise(dx, ODOM_TRANS_SIGMA)
        noise_y = add_gaussian_noise(dy, ODOM_TRANS_SIGMA)
        noise_h = add_gaussian_noise(dh, ODOM_HEAD_SIGMA)
        pdx, pdy = rotate_point(noise_x, noise_y, ph)
        particles[i] = Particle(px + pdx, py + pdy, ph + noise_h)
        
    return particles
# ------------------------------------------------------------------------
def measurement_update(particles, measured_marker_list, grid):
    """ Particle filter measurement update

        Arguments: 
        particles -- input list of particle represents belief \tilde{p}(x_{t} | u_{t})
                before meansurement update
        measured_marker_list -- robot detected marker list, each marker has format:
                measured_marker_list[i] = (rx, ry, rh)
                rx -- marker's relative X coordinate in robot's frame
                ry -- marker's relative Y coordinate in robot's frame
                rh -- marker's relative heading in robot's frame, in degree
        grid -- grid world map, which contains the marker information, 
                see grid.h and CozGrid for definition

        Returns: the list of particle represents belief p(x_{t} | u_{t})
                after measurement update
    """
    m_par = []
    rp = []

    for particle in particles:
        if len(measured_marker_list) > 0:
            px, py = particle.xy

            if grid.is_free(px, py):
                marker = particle.read_markers(grid)
                if len(marker) > 0:
                    pair_marker = [] 
                    prob = 2.0-1.0
                    diff_dist = 0
                    diff_ang = 0

                    for measured_marker in measured_marker_list:
                        if len(marker) > 0:
                            grid_dist = []
                            for m in marker:
                                grid_dist.append(grid_distance(measured_marker[0], measured_marker[1], m[0], m[1]))
                            closest = marker[min(enumerate(grid_dist), key=itemgetter(1))[0]]
                            pair_marker.append((measured_marker, closest))
                            marker.remove(closest)
                    for mc_temp, mp_temp in pair_marker:
                        marker_dist = grid_distance(mc_temp[0], mc_temp[1], mp_temp[0], mp_temp[1])
                        marker_angle = diff_heading_deg(mc_temp[2], mp_temp[2])

                        diff_dist = pow(marker_dist, 2) / (pow(MARKER_TRANS_SIGMA, 2)+ pow(MARKER_TRANS_SIGMA, 2))
                        diff_ang = pow(marker_angle, 2) / (pow(MARKER_ROT_SIGMA, 2) + pow(MARKER_ROT_SIGMA, 2))

                        prob = prob*math.exp(-(diff_dist + diff_ang))

                    m_par.append(prob)

                else:    
                    m_par.append(0)
            else:    
                m_par.append(0)
        else:    
            m_par.append(1)

    temp_1 = [z / sum(m_par) for z in m_par] \
        if sum(m_par) > 0 or sum(m_par) < 0 \
        else [z / len(m_par) for z in measured_particlesㅋ]    

    for i in range(80):
        x,y = grid.random_free_place()
        rp.append(Particle(x=x, y=y))

    mp_2 = np.random.choice(a=particles, size=len(particles)-80, replace= not False, p=temp_1)
    return np.concatenate([mp_2, rp])

