#author 1: Daniel Albuquerque
#author 2: Olatide Omojaro

from grid import *
from particle import Particle
from utils import *
from setting import *


def motion_update(particles, odom):
    """ Particle filter motion update

        Arguments:
        particles -- input list of particle represents belief p(x_{t-1} | u_{t-1})
                before motion update
        odom -- odometry to move (dx, dy, dh) in *robot local frame*

        Returns: the list of particles represents belief \tilde{p}(x_{t} | u_{t})
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
                before measurement update (but after motion update)

        measured_marker_list -- robot detected marker list, each marker has format:
                measured_marker_list[i] = (rx, ry, rh)
                rx -- marker's relative X coordinate in robot's frame
                ry -- marker's relative Y coordinate in robot's frame
                rh -- marker's relative heading in robot's frame, in degree

                * Note that the robot can only see markers which is in its camera field of view,
                which is defined by ROBOT_CAMERA_FOV_DEG in setting.py
				* Note that the robot can see multiple markers at once, and may not see any one

        grid -- grid world map, which contains the marker information,
                see grid.py and CozGrid for definition
                Can be used to evaluate particles

        Returns: the list of particles represents belief p(x_{t} | u_{t})
                after measurement update
    """
    particles_wgts = []
    for particle in particles:
        # check if markers are in view of cozmo
        if len(measured_marker_list) > 0:

            # check if particle is in a valid grid on map
            px, py = particle.xy
            if grid.is_free(x=px, y=py):
                # markers visible to cozmo
                markers_visible_to_particle = particle.read_markers(grid)

                # check if particle has markers in view
                if len(markers_visible_to_particle) > 0:
                    # list of markers pairing (marker seen by robot, marker seen by particle) that's most relative
                    marker_pairings = []    # stored as [(m_c, m_p)]
                    for m_cozmo in measured_marker_list:
                        if len(markers_visible_to_particle) > 0:    # are there markers still in view of particle
                            # find m, the closest marker out of markers_visible_to_particle
                            m_particle_closest = get_closest(m_cozmo, markers_visible_to_particle)
                            # store the pairing[cm, m] for later calculations
                            marker_pairings.append((m_cozmo, m_particle_closest))  # add pairing
                            # remove m from list of possible future pairings
                            # (i.e. if Cozmo sees two markers, they need to be matched to different markers_
                            # visible_to_particle)
                            markers_visible_to_particle.remove(m_particle_closest)

                    # deduce wgts of particle, by comparing how well it sees a marker, compared to cozmo
                    prob = 1.0
                    for m_c, m_p in marker_pairings:
                        dist_btwn_markers = grid_distance(x1=m_c[0],y1=m_c[1], x2=m_p[0],y2=m_p[1])
                        ang_btwn_markers = diff_heading_deg(heading1=m_c[2], heading2=m_p[2])

                        diff_dist = ((dist_btwn_markers)**2) / (2*(MARKER_TRANS_SIGMA**2))
                        diff_ang = ((ang_btwn_markers)**2) / (2*(MARKER_ROT_SIGMA**2))

                        prob *= math.exp(-(diff_dist+diff_ang))

                    particles_wgts.append(prob)

                else:    particles_wgts.append(0)    # particle has no marker in view, wgt -> 0
            else:    particles_wgts.append(0)        # particle is not in a valid grid on map, wgt -> 0
        else:    particles_wgts.append(1)            # cozmo has no marker in view, wgt -> 1

    # normalizing particle wgts
    alpha = sum(particles_wgts)
    # if no markers seen, assign equal wgts, else normalize using sum of wgts
    wgts = [w / alpha for w in particles_wgts] \
        if alpha != 0 \
        else [w / len(particles_wgts) for w in particles_wgts]

    # probabilistic sampling (sampling w/ replacment)
    num_rand_samples = 80
    rand_particles = []

    for i in range(num_rand_samples):
        x,y = grid.random_free_place()
        rand_particles.append(Particle(x=x, y=y))
    # numpy.random.choice(a, size=None, replace=True, p=None)
    import numpy as np
    meas_particles = np.random.choice(a=particles,
                                          size=len(particles)-num_rand_samples,
                                          replace=True,
                                          p=wgts)

    return np.concatenate([meas_particles, rand_particles])


def get_closest(m_cozmo=None, m_particle_list=[]):
    """
    gets the marker visible to a particle that is closest to the marker in field of view of cozmo
    :param m_cozmo: a marker currently in fov of cozmo
    :param m_particle_list: a list of markers that is visible to a particle
    :return: the closest marker
    """

    # get a list with distance between m_cozmo and each m in m_particle_list
    m_p = [grid_distance(m_cozmo[0], m_cozmo[1], p[0], p[1]) for p in m_particle_list]

    # get the marker visible to particle with closest to marker in fov of cozmo
    from operator import itemgetter
    m_closest = m_particle_list[min(enumerate(m_p), key=itemgetter(1))[0]]

    return m_closest