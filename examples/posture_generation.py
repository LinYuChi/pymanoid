#!/usr/bin/env python
# -*- coding: utf-8 -*-
#
# Copyright (C) 2015-2017 Stephane Caron <stephane.caron@lirmm.fr>
#
# This file is part of pymanoid <https://github.com/stephane-caron/pymanoid>.
#
# pymanoid is free software: you can redistribute it and/or modify it under the
# terms of the GNU Lesser General Public License as published by the Free
# Software Foundation, either version 3 of the License, or (at your option) any
# later version.
#
# pymanoid is distributed in the hope that it will be useful, but WITHOUT ANY
# WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR
# A PARTICULAR PURPOSE. See the GNU Lesser General Public License for more
# details.
#
# You should have received a copy of the GNU Lesser General Public License along
# with pymanoid. If not, see <http://www.gnu.org/licenses/>.

"""
This example generates a robot posture from a set of tasks: foothold locations,
center-of-mass position, and a set of reference joint angles for regularization.
See <https://scaron.info/teaching/inverse-kinematics.html> for details.
"""

import IPython
import time
import pymanoid

import numpy as np

from pymanoid import Stance
from pymanoid.robots import JVRC1
from pymanoid.robots import ESCHER


if __name__ == '__main__':
    sim = pymanoid.Simulation(dt=0.03)
    
    robot = JVRC1('JVRC-1.dae', download_if_needed=True)
    # robot = ESCHER()
    
    sim.set_viewer()
    sim.viewer.SetCamera([
        [-0.28985317,  0.40434422, -0.86746233,  2.73872042],
        [0.95680251,  0.10095043, -0.2726499,  0.86080128],
        [-0.02267371, -0.90901857, -0.41613837,  2.06654644],
        [0.,  0.,  0.,  1.]])

    robot.set_z(0.8)  # hack to start with the robot above contacts
    lf_target = robot.left_foot.get_contact(pos=[0, 0.1, 0])
    rf_target = robot.right_foot.get_contact(pos=[0, -0.1, 0])
    lh_target = robot.left_hand.get_contact(pos=[0, 0.5, 1.0])
    # lh_target.set_transform(np.array([[0,1,0,0],[0,0,1,0.6],[1,0,0,1.5],[0,0,0,1]]))
    lh_target.set_transform(np.array([[0,0,1,0],[1,0,0,0.6],[0,1,0,1.5],[0,0,0,1]]))
    rh_target = robot.right_hand.get_contact(pos=[0, -0.5, 1.0])
    # rh_target.set_transform(np.array([[0,-1,0,0],[0,0,-1,-0.6],[1,0,0,1.5],[0,0,0,1]]))
    rh_target.set_transform(np.array([[0,0,1,0],[-1,0,0,-0.6],[0,-1,0,1.5],[0,0,0,1]]))
    com_target = robot.get_com_point_mass()

    com_target.set_pos(np.array([0,0,1.2]))

    start = time.time()
    stance = Stance(com=com_target, left_foot=lf_target, right_foot=rf_target, left_hand=lh_target, right_hand=rh_target)
    stance.dof_tasks[robot.R_SHOULDER_R] = -0.5
    stance.dof_tasks[robot.L_SHOULDER_R] = +0.5
    stance.bind(robot)

    robot.ik.solve(max_it=100, impr_stop=1e-4)

    end = time.time()

    print(end-start)

    sim.schedule(robot.ik)
    sim.start()

    if IPython.get_ipython() is None:  # give the user a prompt
        IPython.embed()
