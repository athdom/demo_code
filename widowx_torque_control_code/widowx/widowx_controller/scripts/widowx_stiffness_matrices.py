#!/usr/bin/env python3

"""
Compute state space dynamic and kinematic matrices for widowx robot arm (3 links)
"""

#Math stuff
from math import sin, cos, atan2, pi, sqrt
from numpy.linalg import inv, det, norm, pinv
import numpy as np
#Arm parameters
from widowx_arm import *


class WidowxStiffness():
    def __init__(self):
        pass

    def compute_jacobian(self, r_joints_array):
      
        r_J_e = np.matrix([[0.071014999999999994795274460557266*sin(r_joints_array[0])*sin(r_joints_array[1])*sin(r_joints_array[2]) - 0.14203*sin(r_joints_array[0])*sin(r_joints_array[1]) - 0.1145*cos(r_joints_array[1] + r_joints_array[2] + r_joints_array[3])*sin(r_joints_array[0]) - 0.071015000000000005204725539442734*cos(r_joints_array[1] + r_joints_array[2])*sin(r_joints_array[0]) - 0.04825*cos(r_joints_array[1])*sin(r_joints_array[0]) - 0.071014999999999994795274460557266*cos(r_joints_array[1])*cos(r_joints_array[2])*sin(r_joints_array[0]),\
                            0.14203*cos(r_joints_array[0])*cos(r_joints_array[1]) - 0.04825*cos(r_joints_array[0])*sin(r_joints_array[1]) - 0.1145*sin(r_joints_array[1] + r_joints_array[2] + r_joints_array[3])*cos(r_joints_array[0]) - 0.071015000000000005204725539442734*sin(r_joints_array[1] + r_joints_array[2])*cos(r_joints_array[0]) - 0.071014999999999994795274460557266*cos(r_joints_array[0])*cos(r_joints_array[1])*sin(r_joints_array[2]) - 0.071014999999999994795274460557266*cos(r_joints_array[0])*cos(r_joints_array[2])*sin(r_joints_array[1]),\
                            - 0.1145*sin(r_joints_array[1] + r_joints_array[2] + r_joints_array[3])*cos(r_joints_array[0]) - 0.071015000000000005204725539442734*sin(r_joints_array[1] + r_joints_array[2])*cos(r_joints_array[0]) - 0.071014999999999994795274460557266*cos(r_joints_array[0])*cos(r_joints_array[1])*sin(r_joints_array[2]) - 0.071014999999999994795274460557266*cos(r_joints_array[0])*cos(r_joints_array[2])*sin(r_joints_array[1]),\
                            -0.1145*sin(r_joints_array[1] + r_joints_array[2] + r_joints_array[3])*cos(r_joints_array[0]),\
                            0.0],\
                            #Line 2
                            [0.04825*cos(r_joints_array[0])*cos(r_joints_array[1]) + 0.14203*cos(r_joints_array[0])*sin(r_joints_array[1]) + 0.1145*cos(r_joints_array[1] + r_joints_array[2] + r_joints_array[3])*cos(r_joints_array[0]) + 0.071015000000000005204725539442734*cos(r_joints_array[1] + r_joints_array[2])*cos(r_joints_array[0]) + 0.071014999999999994795274460557266*cos(r_joints_array[0])*cos(r_joints_array[1])*cos(r_joints_array[2]) - 0.071014999999999994795274460557266*cos(r_joints_array[0])*sin(r_joints_array[1])*sin(r_joints_array[2]),\
                             0.14203*cos(r_joints_array[1])*sin(r_joints_array[0]) - 0.04825*sin(r_joints_array[0])*sin(r_joints_array[1]) - 0.1145*sin(r_joints_array[1] + r_joints_array[2] + r_joints_array[3])*sin(r_joints_array[0]) - 0.071015000000000005204725539442734*sin(r_joints_array[1] + r_joints_array[2])*sin(r_joints_array[0]) - 0.071014999999999994795274460557266*cos(r_joints_array[1])*sin(r_joints_array[0])*sin(r_joints_array[2]) - 0.071014999999999994795274460557266*cos(r_joints_array[2])*sin(r_joints_array[0])*sin(r_joints_array[1]),\
                             - 0.1145*sin(r_joints_array[1] + r_joints_array[2] + r_joints_array[3])*sin(r_joints_array[0]) - 0.071015000000000005204725539442734*sin(r_joints_array[1] + r_joints_array[2])*sin(r_joints_array[0]) - 0.071014999999999994795274460557266*cos(r_joints_array[1])*sin(r_joints_array[0])*sin(r_joints_array[2]) - 0.071014999999999994795274460557266*cos(r_joints_array[2])*sin(r_joints_array[0])*sin(r_joints_array[1]), -0.1145*sin(r_joints_array[1] + r_joints_array[2] + r_joints_array[3])*sin(r_joints_array[0]),\
                            0],\
                            #Line 3
                            [0.0,\
                            - 0.1145*cos(r_joints_array[1] + r_joints_array[2] + r_joints_array[3]) - 0.14203*cos(r_joints_array[1] + r_joints_array[2]) - 0.04825*cos(r_joints_array[1]) - 0.14203*sin(r_joints_array[1]),\
                            - 0.1145*cos(r_joints_array[1] + r_joints_array[2] + r_joints_array[3]) - 0.14203*cos(r_joints_array[1] + r_joints_array[2]),\
                            -0.1145*cos(r_joints_array[1] + r_joints_array[2] + r_joints_array[3]),\
                            0.0],\
                            #Line 4
                            [0.0, -1.0*sin(r_joints_array[0]), -1.0*sin(r_joints_array[0]), -1.0*sin(r_joints_array[0]),  cos(r_joints_array[1] + r_joints_array[2] + r_joints_array[3])*cos(r_joints_array[0])],\
                            #Line 5
                            [0.0, cos(r_joints_array[0]), cos(r_joints_array[0]),  cos(r_joints_array[0]), cos(r_joints_array[1] + r_joints_array[2] + r_joints_array[3])*sin(r_joints_array[0])],\
                            #Line 6
                            [1.0, 0.0, 0.0, 0.0, -1.0*sin(r_joints_array[1] + r_joints_array[2] + r_joints_array[3])]])

        return r_J_e


    def compute_zero_jacobian(self, r_joints_array):
      
        r_J_e = np.matrix([[0.071014999999999994795274460557266*sin(r_joints_array[0])*sin(r_joints_array[1])*sin(r_joints_array[2]) - 0.14203*sin(r_joints_array[0])*sin(r_joints_array[1]) - 0.1145*cos(r_joints_array[1] + r_joints_array[2] + r_joints_array[3])*sin(r_joints_array[0]) - 0.071015000000000005204725539442734*cos(r_joints_array[1] + r_joints_array[2])*sin(r_joints_array[0]) - 0.04825*cos(r_joints_array[1])*sin(r_joints_array[0]) - 0.071014999999999994795274460557266*cos(r_joints_array[1])*cos(r_joints_array[2])*sin(r_joints_array[0]),\
                            0.14203*cos(r_joints_array[0])*cos(r_joints_array[1]) - 0.04825*cos(r_joints_array[0])*sin(r_joints_array[1]) - 0.1145*sin(r_joints_array[1] + r_joints_array[2] + r_joints_array[3])*cos(r_joints_array[0]) - 0.071015000000000005204725539442734*sin(r_joints_array[1] + r_joints_array[2])*cos(r_joints_array[0]) - 0.071014999999999994795274460557266*cos(r_joints_array[0])*cos(r_joints_array[1])*sin(r_joints_array[2]) - 0.071014999999999994795274460557266*cos(r_joints_array[0])*cos(r_joints_array[2])*sin(r_joints_array[1]),\
                            - 0.1145*sin(r_joints_array[1] + r_joints_array[2] + r_joints_array[3])*cos(r_joints_array[0]) - 0.071015000000000005204725539442734*sin(r_joints_array[1] + r_joints_array[2])*cos(r_joints_array[0]) - 0.071014999999999994795274460557266*cos(r_joints_array[0])*cos(r_joints_array[1])*sin(r_joints_array[2]) - 0.071014999999999994795274460557266*cos(r_joints_array[0])*cos(r_joints_array[2])*sin(r_joints_array[1]),\
                            -0.1145*sin(r_joints_array[1] + r_joints_array[2] + r_joints_array[3])*cos(r_joints_array[0]),\
                            0.0],\
                            #Line 2
                            [0.04825*cos(r_joints_array[0])*cos(r_joints_array[1]) + 0.14203*cos(r_joints_array[0])*sin(r_joints_array[1]) + 0.1145*cos(r_joints_array[1] + r_joints_array[2] + r_joints_array[3])*cos(r_joints_array[0]) + 0.071015000000000005204725539442734*cos(r_joints_array[1] + r_joints_array[2])*cos(r_joints_array[0]) + 0.071014999999999994795274460557266*cos(r_joints_array[0])*cos(r_joints_array[1])*cos(r_joints_array[2]) - 0.071014999999999994795274460557266*cos(r_joints_array[0])*sin(r_joints_array[1])*sin(r_joints_array[2]),\
                             0.14203*cos(r_joints_array[1])*sin(r_joints_array[0]) - 0.04825*sin(r_joints_array[0])*sin(r_joints_array[1]) - 0.1145*sin(r_joints_array[1] + r_joints_array[2] + r_joints_array[3])*sin(r_joints_array[0]) - 0.071015000000000005204725539442734*sin(r_joints_array[1] + r_joints_array[2])*sin(r_joints_array[0]) - 0.071014999999999994795274460557266*cos(r_joints_array[1])*sin(r_joints_array[0])*sin(r_joints_array[2]) - 0.071014999999999994795274460557266*cos(r_joints_array[2])*sin(r_joints_array[0])*sin(r_joints_array[1]),\
                             - 0.1145*sin(r_joints_array[1] + r_joints_array[2] + r_joints_array[3])*sin(r_joints_array[0]) - 0.071015000000000005204725539442734*sin(r_joints_array[1] + r_joints_array[2])*sin(r_joints_array[0]) - 0.071014999999999994795274460557266*cos(r_joints_array[1])*sin(r_joints_array[0])*sin(r_joints_array[2]) - 0.071014999999999994795274460557266*cos(r_joints_array[2])*sin(r_joints_array[0])*sin(r_joints_array[1]), -0.1145*sin(r_joints_array[1] + r_joints_array[2] + r_joints_array[3])*sin(r_joints_array[0]),\
                            0],\
                            #Line 3
                            [0.0,\
                            - 0.1145*cos(r_joints_array[1] + r_joints_array[2] + r_joints_array[3]) - 0.14203*cos(r_joints_array[1] + r_joints_array[2]) - 0.04825*cos(r_joints_array[1]) - 0.14203*sin(r_joints_array[1]),\
                            - 0.1145*cos(r_joints_array[1] + r_joints_array[2] + r_joints_array[3]) - 0.14203*cos(r_joints_array[1] + r_joints_array[2]),\
                            -0.1145*cos(r_joints_array[1] + r_joints_array[2] + r_joints_array[3]),\
                            0.0],\
                            #Line 4
                            [0.0, -1.0*sin(0), -1.0*sin(0), -1.0*sin(0),  cos(r_joints_array[1] + r_joints_array[2] + r_joints_array[3])*cos(0)],\
                            #Line 5
                            [0.0, cos(0), cos(0),  cos(0), cos(r_joints_array[1] + r_joints_array[2] + r_joints_array[3])*sin(0)],\
                            #Line 6
                            [1.0, 0.0, 0.0, 0.0, -1.0*sin(r_joints_array[1] + r_joints_array[2] + r_joints_array[3])]])

        return r_J_e

    def compute_ee_pos(self, r_joints_array):
        r_x_e = np.array([[0.04825*cos(r_joints_array[0])*cos(r_joints_array[1]) + 0.14203*cos(r_joints_array[0])*sin(r_joints_array[1]) + 0.1145*cos(r_joints_array[1] + r_joints_array[2] + r_joints_array[3])*cos(r_joints_array[0]) + 0.071015000000000005204725539442734*cos(r_joints_array[1] + r_joints_array[2])*cos(r_joints_array[0]) + 0.071014999999999994795274460557266*cos(r_joints_array[0])*cos(r_joints_array[1])*cos(r_joints_array[2]) - 0.071014999999999994795274460557266*cos(r_joints_array[0])*sin(r_joints_array[1])*sin(r_joints_array[2])],\
                          [0.04825*cos(r_joints_array[1])*sin(r_joints_array[0]) + 0.14203*sin(r_joints_array[0])*sin(r_joints_array[1]) + 0.1145*cos(r_joints_array[1] + r_joints_array[2] + r_joints_array[3])*sin(r_joints_array[0]) + 0.071015000000000005204725539442734*cos(r_joints_array[1] + r_joints_array[2])*sin(r_joints_array[0]) - 0.071014999999999994795274460557266*sin(r_joints_array[0])*sin(r_joints_array[1])*sin(r_joints_array[2]) + 0.071014999999999994795274460557266*cos(r_joints_array[1])*cos(r_joints_array[2])*sin(r_joints_array[0])],\
                          [0.14203*cos(r_joints_array[1]) - 0.14203*sin(r_joints_array[1] + r_joints_array[2]) - 0.1145*sin(r_joints_array[1] + r_joints_array[2] + r_joints_array[3]) - 0.04825*sin(r_joints_array[1]) + 0.125]])
        return r_x_e

    def compute_ee_orientation(self, r_joints_array):
        r_rot_e = np.array([[- 1.0*sin(r_joints_array[0])*sin(r_joints_array[4]) - 1.0*sin(r_joints_array[1] + r_joints_array[2] + r_joints_array[3])*cos(r_joints_array[0])*cos(r_joints_array[4]), sin(r_joints_array[1] + r_joints_array[2] + r_joints_array[3])*cos(r_joints_array[0])*sin(r_joints_array[4]) - 1.0*cos(r_joints_array[4])*sin(r_joints_array[0]), cos(r_joints_array[1] + r_joints_array[2] + r_joints_array[3])*cos(r_joints_array[0])],\
                          [cos(r_joints_array[0])*sin(r_joints_array[4]) - 1.0*sin(r_joints_array[1] + r_joints_array[2] + r_joints_array[3])*cos(r_joints_array[4])*sin(r_joints_array[0]), cos(r_joints_array[0])*cos(r_joints_array[4]) + sin(r_joints_array[1] + r_joints_array[2] + r_joints_array[3])*sin(r_joints_array[0])*sin(r_joints_array[4]), cos(r_joints_array[1] + r_joints_array[2] + r_joints_array[3])*sin(r_joints_array[0])],\
                          [-1.0*cos(r_joints_array[1] + r_joints_array[2] + r_joints_array[3])*cos(r_joints_array[4]), cos(r_joints_array[1] + r_joints_array[2] + r_joints_array[3])*sin(r_joints_array[4]), -1.0*sin(r_joints_array[1] + r_joints_array[2] + r_joints_array[3])]])
        return r_rot_e

    # def compute_ee_orientation(self, r_joints_array):
    #     r_rot_e = np.array([[cos(r_joints_array[1] + r_joints_array[2] + r_joints_array[3])*cos(r_joints_array[0])*cos(r_joints_array[4]) - 1.0*sin(r_joints_array[0])*sin(r_joints_array[4]), - 1.0*cos(r_joints_array[4])*sin(r_joints_array[0]) - 1.0*cos(r_joints_array[1] + r_joints_array[2] + r_joints_array[3])*cos(r_joints_array[0])*sin(r_joints_array[4]), -1.0*sin(r_joints_array[1] + r_joints_array[2] + r_joints_array[3])*cos(r_joints_array[0]), 0],\
    #                       [cos(r_joints_array[0])*sin(r_joints_array[4]) + cos(r_joints_array[1] + r_joints_array[2] + r_joints_array[3])*cos(r_joints_array[4])*sin(r_joints_array[0]), cos(r_joints_array[0])*cos(r_joints_array[4]) - 1.0*cos(r_joints_array[1] + r_joints_array[2] + r_joints_array[3])*sin(r_joints_array[0])*sin(r_joints_array[4]), -1.0*sin(r_joints_array[1] + r_joints_array[2] + r_joints_array[3])*sin(r_joints_array[0]), 0],\
    #                       [sin(r_joints_array[1] + r_joints_array[2] + r_joints_array[3])*cos(r_joints_array[4]),  -1.0*sin(r_joints_array[1] + r_joints_array[2] + r_joints_array[3])*sin(r_joints_array[4]),  cos(r_joints_array[1] + r_joints_array[2] + r_joints_array[3]), 0]
    #                       [0, 0, 0, 1]])
    #     return r_rot_e

    # def compute_skew(self, vector):
    #     skew = np.array([[0, -vector[2], vector[1]],\
    #                       [vector[2], 0, -vector[0]],\
    #                       [-vector[1], vector[0], 0]])
    #     return skew

    def compute_g(self, r_joints_array):

        # end-effector mass 107gr
        # g = np.matrix([[0],\
        #                [- 0.21968293275*cos(r_joints_array[1] + r_joints_array[2] + r_joints_array[3]) - 0.59104392605999999688544018994207*cos(r_joints_array[1] + r_joints_array[2]) - 0.30515746275*cos(r_joints_array[1]) - 0.89826972920999999622168154189694*sin(r_joints_array[1])],\
        #                [- 0.21968293275*cos(r_joints_array[1] + r_joints_array[2] + r_joints_array[3]) - 0.59104392605999999688544018994207*cos(r_joints_array[1] + r_joints_array[2])],\
        #                [-0.21968293275*cos(r_joints_array[1] + r_joints_array[2] + r_joints_array[3])],\
        #                [0]])
        # end-effector mass 67gr
        g = np.matrix([[0],\
                       [- 0.17475313275*cos(r_joints_array[1] + r_joints_array[2] + r_joints_array[3]) - 0.53531135405999999688544018994207*cos(r_joints_array[1] + r_joints_array[2]) - 0.28622416275*cos(r_joints_array[1]) - 0.84253715720999999622168154189694*sin(r_joints_array[1])],\
                       [- 0.17475313275*cos(r_joints_array[1] + r_joints_array[2] + r_joints_array[3]) - 0.53531135405999999688544018994207*cos(r_joints_array[1] + r_joints_array[2])],\
                       [-0.17475313275*cos(r_joints_array[1] + r_joints_array[2] + r_joints_array[3])],\
                       [0]])
        # Old gravity compensation
        # g = np.matrix([[0],\
        #                [0.69474494555999997358275432901564*cos(r_joints_array[1]) + 0.21649055273999998623105089912144*sin(r_joints_array[1]) + 0.40336448984999999688544018994207*cos(r_joints_array[1])*cos(r_joints_array[2]) - 0.40336448984999999688544018994207*sin(r_joints_array[1])*sin(r_joints_array[2]) + 0.1384355808*cos(r_joints_array[1])*cos(r_joints_array[2])*cos(r_joints_array[3]) - 0.1384355808*cos(r_joints_array[1])*sin(r_joints_array[2])*sin(r_joints_array[3]) - 0.1384355808*cos(r_joints_array[2])*sin(r_joints_array[1])*sin(r_joints_array[3]) - 0.1384355808*cos(r_joints_array[3])*sin(r_joints_array[1])*sin(r_joints_array[2])],\
        #                [0.1384355808*cos(r_joints_array[1] + r_joints_array[2] + r_joints_array[3]) + 0.40336448984999999688544018994207*cos(r_joints_array[1] + r_joints_array[2])],\
        #                [0.1384355808*cos(r_joints_array[1] + r_joints_array[2] + r_joints_array[3])],\
        #                [0]])

      

        return g

    