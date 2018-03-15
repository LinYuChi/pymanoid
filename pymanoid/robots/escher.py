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

from numpy import array, pi

from ..body import Manipulator
from ..robot import Humanoid
from ..tasks import DOFTask, MinCAMTask


class ESCHER(Humanoid):

    """
    ESCHER humanoid robot.

    This file includes information that is publicly released in
    <http://doai.io/10.1109/IROS.2011.6094465> or over the Web for the total
    mass and kinematic chain. Other information is read from the COLLADA model.
    """

    leg_length = 0.8           # [m]   (roughly, for a strechted leg)
    mass = 39.                 # [kg]  (includes batteries)
    sole_shape = (0.125, 0.0675)  # (half-length [m], half-width [m]) of foot sole
    palm_shape = (0.1, 0.07)  # (half-length [m], half-width [m]) of palm patch

    # DOF indexes with respect to COLLADA model
    head_lidar_roll = 0
    head_pitch = 1
    head_yaw = 2
    l_ankle_pitch = 3
    l_ankle_roll = 4
    l_elbow_pitch = 5
    l_elbow_roll = 6
    l_hip_pitch = 7
    l_hip_roll = 8
    l_hip_yaw = 9
    l_index_yaw = 10
    l_knee_pitch = 11
    l_ring_yaw = 12
    l_shoulder_pitch = 13
    l_shoulder_roll = 14
    l_shoulder_yaw = 15
    l_thumb_pitch = 16
    l_thumb_roll = 17
    l_wrist_pitch = 18
    l_wrist_yaw = 19
    pitch_revolute_joint = 20
    r_ankle_pitch = 21
    r_ankle_roll = 22
    r_elbow_pitch = 23
    r_elbow_roll = 24
    r_hip_pitch = 25
    r_hip_roll = 26
    r_hip_yaw = 27
    r_index_yaw = 28
    r_knee_pitch = 29
    r_ring_yaw = 30
    r_shoulder_pitch = 31
    r_shoulder_roll = 32
    r_shoulder_yaw = 33
    r_thumb_pitch = 34
    r_thumb_roll = 35
    r_wrist_pitch = 36
    r_wrist_yaw = 37
    roll_revolute_joint = 38
    waist_yaw = 39
    x_prismatic_joint = 40
    y_prismatic_joint = 41
    yaw_revolute_joint = 42
    z_prismatic_joint = 43

    L_SHOULDER_R = l_shoulder_roll
    L_SHOULDER_P = l_shoulder_pitch
    L_SHOULDER_Y = l_shoulder_yaw
    R_SHOULDER_R = r_shoulder_roll
    R_SHOULDER_P = r_shoulder_pitch
    R_SHOULDER_Y = r_shoulder_yaw
    
    TRANS_X = x_prismatic_joint
    TRANS_Y = y_prismatic_joint
    TRANS_Z = z_prismatic_joint
    ROT_R = roll_revolute_joint
    ROT_P = pitch_revolute_joint
    ROT_Y = yaw_revolute_joint

    

    # Joints
    # chest = [CHEST_P, CHEST_Y]
    # free_pos = [TRANS_X, TRANS_Y, TRANS_Z]
    # free_rpy = [ROT_R, ROT_P, ROT_Y]
    # left_ankle = [L_ANKLE_P, L_ANKLE_R]
    # left_elbow = [L_ELBOW_P]
    # left_hip = [L_HIP_Y, L_HIP_R, L_HIP_P]
    # left_knee = [L_KNEE_P]
    # left_shoulder = [L_SHOULDER_P, L_SHOULDER_R, L_SHOULDER_Y]
    # left_thumb = [L_HAND_J0, L_HAND_J1]
    # left_wrist = [L_WRIST_Y, L_WRIST_P, L_WRIST_R]
    # neck = [NECK_Y, NECK_P]
    # right_ankle = [R_ANKLE_P, R_ANKLE_R]
    # right_elbow = [R_ELBOW_P]
    # right_hip = [R_HIP_Y, R_HIP_R, R_HIP_P]
    # right_knee = [R_KNEE_P]
    # right_shoulder = [R_SHOULDER_P, R_SHOULDER_R, R_SHOULDER_Y]
    # right_thumb = [R_HAND_J0, R_HAND_J1]
    # right_wrist = [R_WRIST_Y, R_WRIST_P, R_WRIST_R]

    # Floating Base
    free_pos = [x_prismatic_joint, y_prismatic_joint, z_prismatic_joint]
    free_rpy = [roll_revolute_joint, pitch_revolute_joint, yaw_revolute_joint]
    
    # Center
    chest = [waist_yaw]

    # Limbs
    left_arm = [l_shoulder_pitch, l_shoulder_roll, l_shoulder_yaw, l_elbow_pitch, l_elbow_roll, l_wrist_pitch, l_wrist_yaw]
    left_leg = [l_hip_yaw, l_hip_roll, l_hip_pitch, l_knee_pitch, l_ankle_pitch, l_ankle_roll]
    right_arm = [r_shoulder_pitch, r_shoulder_roll, r_shoulder_yaw, r_elbow_pitch, r_elbow_roll, r_wrist_pitch, r_wrist_yaw]
    right_leg = [r_hip_yaw, r_hip_roll, r_hip_pitch, r_knee_pitch, r_ankle_pitch, r_ankle_roll]

    # Body
    arms = left_arm + right_arm
    free = free_pos + free_rpy
    legs = left_leg + right_leg
    upper_body = arms + chest
    whole_body = arms + legs + chest + free

    # Half-sitting posture
    q_halfsit = array([  0.00000000e+00,   1.30580638e-02,  -1.40269059e-03,
                        -3.86412875e-01,   7.32566641e-04,   5.02539889e-01,
                        4.82786220e-05,  -3.05805816e-01,  -4.29312864e-04,
                        2.13561552e-04,   5.81795021e-08,   6.68554145e-01,
                        1.14303672e-07,   5.35135688e-01,   1.97788584e-01,
                        1.86868359e-04,  -1.91525395e-07,  -8.35585732e-06,
                        3.75628690e-05,   6.03302644e-05,   0.00000000e+00,
                        -3.85086517e-01,   8.67205998e-04,   4.98963063e-01,
                        7.35581810e-05,  -3.07858142e-01,  -5.49373110e-04,
                        -9.83564221e-04,   6.83513540e-07,   6.70171382e-01,
                        7.31314300e-07,   5.38380011e-01,  -1.98555985e-01,
                        2.62093203e-03,   1.86053196e-06,   1.16882178e-05,
                        1.12270384e-03,   1.49762135e-04,  -3.17439990e-04,
                        -4.27539115e-07,   2.45785498e-02,   6.09822426e-04,
                        9.91970073e-04,  -4.64415464e-02])


    def __init__(self, path=['new_escher_chest_modified.urdf','escher.srdf'], root_body='base_link'):
        """
        Add the ESCHER model to the environment.

        Parameters
        ----------
        path : string
            Path to the COLLADA model of the robot.
        root_body : string
            Name of the root body in the kinematic chain.

        Note
        ----
        Unfortunately it is unclear whether we can release the COLLADA file
        ``HRP4R.dae`` (md5sum: 38c30928b7ae62daa0fc67ed0488b0a1) due to
        copyright.
        """
        super(ESCHER, self).__init__(path, root_body)
        self.ik.set_active_dofs(self.whole_body)
        self.mass = sum([link.GetMass() for link in self.rave.GetLinks()])
        
        rave_left_foot = self.rave.GetManipulator("l_leg")
        rave_right_foot = self.rave.GetManipulator("r_leg")
        rave_left_hand = self.rave.GetManipulator("l_arm")
        rave_right_hand = self.rave.GetManipulator("r_arm")

        rave_left_hand.SetLocalToolDirection(array([1, 0, 0]))
        rave_left_hand.SetLocalToolTransform(array([
            [0,  1, 0, 0.086],
            [ -1, 0, 0, -0.03],
            [ 0,  0, 1, 0],
            [ 0,  0, 0, 1]])
        )

        rave_right_hand.SetLocalToolDirection(array([1, 0, 0]))
        rave_right_hand.SetLocalToolTransform(array([
            [ 0,  -1, 0, 0.086],
            [ 1,  0, 0, 0.03],
            [ 0,  0, 1, 0],
            [ 0,  0, 0, 1]])
        )

        rave_left_foot.SetLocalToolDirection(array([0, 0, -1]))
        rave_right_foot.SetLocalToolDirection(array([0, 0, -1]))
        
        self.left_foot = Manipulator(
            rave_left_foot, shape=self.sole_shape,
            friction=0.8)
        self.left_hand = Manipulator(
            rave_left_hand, shape=self.palm_shape,
            friction=0.8)
        self.right_foot = Manipulator(
            rave_right_foot, shape=self.sole_shape,
            friction=0.8)
        self.right_hand = Manipulator(
            rave_right_hand, shape=self.palm_shape,
            friction=0.8)

    def add_shoulder_abduction_task(self, weight=None):
        self.ik.add_task(DOFTask(self, self.r_shoulder_roll, -0.4, weight))
        self.ik.add_task(DOFTask(self, self.l_shoulder_roll, +0.4, weight))

    def add_shoulder_extension_task(self, weight=None):
        self.ik.add_task(DOFTask(self, self.l_shoulder_pitch, +0.5, weight))
        self.ik.add_task(DOFTask(self, self.r_shoulder_pitch, +0.5, weight))

    def add_shoulder_flexion_task(self, weight=None):
        self.ik.add_task(DOFTask(self, self.l_shoulder_pitch, -0.5, weight))
        self.ik.add_task(DOFTask(self, self.r_shoulder_pitch, -0.5, weight))

    def add_shoulder_neutral_pitch_task(self, weight=None):
        self.ik.add_task(DOFTask(self, self.l_shoulder_pitch, 0., weight))
        self.ik.add_task(DOFTask(self, self.r_shoulder_pitch, 0., weight))

    def add_upright_chest_task(self, weight=None):
        self.ik.add_task(DOFTask(self, self.pitch_revolute_joint, 0., weight))
        self.ik.add_task(DOFTask(self, self.waist_yaw, 0., weight))

    def setup_ik_for_walking(self, com_target):
        """
        Prepare inverse-kinematic tracking for locomotion.

        Parameters
        ----------
        com_target : pymanoid.PointMass
            Target for the IK task on the center of mass.
        """
        self.ik.tasks['COM'].update_target(com_target)
        self.ik.add_task(MinCAMTask(self))
        self.add_upright_chest_task()
        self.add_shoulder_neutral_pitch_task()
        self.ik.set_task_weights({
            self.left_foot.name:  1.,
            self.right_foot.name: 1.,
            'COM': 1e-2,
            'MIN_CAM': 1e-4,
            'ROT_P': 1e-4,
            'CHEST_P': 1e-4,
            'CHEST_Y': 1e-4,
            'L_SHOULDER_P': 1e-5,
            'R_SHOULDER_P': 1e-5,
        })

    def suntan(self, amount=0.3):
        """
        Prepare model for screenshots on white background ;)

        Parameters
        ----------
        amount : scalar
            Number between 0. (no suntan) and 1. (full burn).
        """
        ambient, diffuse = 0., 1. - amount
        for link in self.rave.GetLinks():
            if len(link.GetGeometries()) > 0:
                geom = link.GetGeometries()[0]
                geom.SetAmbientColor([ambient] * 3)
                geom.SetDiffuseColor([diffuse] * 3)
