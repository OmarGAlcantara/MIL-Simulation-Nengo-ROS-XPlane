#################################################################################
#                               04/02/2024
#author: Omar Garcia
#github: https://github.com/OmarGAlcantara/
#    Gain Sets File

# This is a sample script for a Neuromorphic PD Control for a MIL simulation with XPlane 11 by using XPlaneConnect and XPlaneROS for the communication
# The PD Controller was developed and programmed using Nengo and is based on a classic PD control strategy
# Simulation should be executed in the following order:
# 1. Opening a simulation in XPlane 11 with the intelAeroRTF model
# 2. Launching the XplaneROS wrapper by typing in the terminal: roslaunch xplane_ros default.launch
# 3. Launching the XplaneROS launch file of the controller roslaunch nengo_pid nengo.launch

#This controller effectively performs a quadcopter's tracking task of an ascensional ramp and a circle using a neuromorphic PD Control strategy
#################################################################################

import numpy as np

def get_gain_matrices(gain_set='pd'):

    kpx = 0.09
    kdx = 0.208
    kpy = -0.0936
    kdy = -0.192

    kiz = 0
    kpz = 0.096
    kdz = 0.72

    kphi = 3.9
    ktheta = 3.9
    kpsi = -3.4

    kpp = 0.12
    kip = 0
    kdp = 0.005

    kpq = 0.108
    kiq = 0
    kdq = 0.01

    kpr = 0.25
    kir = 0
    kdr = 0.25

    # This set of gains is the one we sent the first draft of NICE
    #
    # kpx = 0.03 * 2
    # kdx = 0.08 * 2.6
    # kpy = -0.04 * 0.9 * 1.6
    # kdy = -0.12 * 1 * 1.8
    #
    # kiz = 0
    #
    # kpz = 0.02 * 8 * 0.8
    # kdz = 0.06 * 4 * 1.3
    #
    # kphi = 3.9
    # ktheta = 3.9
    # kpsi = -3.4
    #
    # kpp = 0.1*1.3
    # kip = 0
    # kdp = 0.005

    # kpq = 0.09*1.3
    # kiq = 0
    # kdq = 0.03
    #
    # kpr = 0.25
    # kir = 0
    # kdr = 0.25
    #############################################333

    if gain_set == 'pd':
        gain_matrix_position = np.matrix([[0, kpy, 0, 0, kdy, 0, 0],
                                          [kpx, 0, 0, kdx, 0, 0, 0],
                                          [0, 0, kpz, 0, 0, kdz, kiz]])
        gain_matrix_virtual = np.matrix([[0, 0, kpsi],
                                         [0 , ktheta, 0],
                                         [kphi , 0, 0]])
        gain_matrix_proportionals = np.matrix([[kpp, 0, 0],
                                              [0, kpq, 0],
                                               [0, 0, kpr]])
        gain_matrix_derivatives = np.matrix([[kdp, 0, 0],
                                               [0, kdq, 0],
                                               [0, 0, kdr]])
        mixer = np.matrix([[1, 1, -1, 1],
                                   [1, 1, 1, -1],
                                   [1, -1, 1, 1],
                                   [1, -1, -1, -1]])
    return gain_matrix_position, gain_matrix_virtual, gain_matrix_proportionals, gain_matrix_derivatives, mixer
