import numpy as np

def get_gain_matrices(gain_set='omar'):

    # This set of gains is suitable for a 10 meter reference take off and falls about 5 seconds in the air


    kpx = 0.03 * 1.6
    kdx = 0.08 * 1.8       #If we want to improve circle response just circle decrease der to 1.6
    kpy = -0.04 * 0.9 * 1.6
    kdy = -0.12 * 1 * 1.8

    kiz = 0  # 0.00055 #0.00007

    kpz = 0.02 * 8 * 0.8  # 0.01*1.5
    kdz = 0.06 * 4 * 1.3  # 0.06 * 4.0

    kphi = 3.9# 2.2  # 0.42 #0.5
    ktheta = 3.9
    kpsi = -3.4  # 0.1 #0.15#0.1

    kpp = 0.1*1.3#/6
    kip = 0
    kdp = 0.005#/8
    # 0.03
    kpq = 0.09*1.3#/6
    kiq = 0
    kdq = 0.03#/5

    kpr = 0.25#0.11
    kir = 0
    kdr = 0.25#0.3

    ## This is the previous perfect gain set for before they asked me to remove the direct modes

    # kpx = 0.03 * 1.3
    # kdx = 0.08 * 1.3
    # kpy = -0.04 * 0.9 * 1.3
    # kdy = -0.12 * 1 * 1.3
    #
    # kiz = 0  # 0.00055 #0.00007
    #
    # kpz = 0.02 * 8 * 1.1  # 0.01*1.5
    # kdz = 0.06 * 4 * 1.1  # 0.06 * 4.0
    #
    # kphi = 1.4# 2.2  # 0.42 #0.5
    # ktheta = 1.4
    # kpsi = -1.5  # 0.1 #0.15#0.1
    #
    # kpp = 0.07#/6
    # kip = 0
    # kdp = 0.01#/8
    # # 0.03
    # kpq = 0.08#/6
    # kiq = 0
    # kdq = 0.03#/5
    #
    # kpr = 0.19#0.11
    # kir = 0
    # kdr = 0.1#0.3
    #############################################333



    if gain_set == 'omar':
        gain_matrix_position = np.matrix([[0, kpy, 0, 0, kdy, 0, 0],
                                          [kpx, 0, 0, kdx, 0, 0, 0],
                                          [0, 0, kpz, 0, 0, kdz, kiz]])
        gain_matrix_virtual = np.matrix([[0, 0, kpsi],
                                         [0 , ktheta, 0],
                                         [kphi , 0, 0]])
        gain_matrix_controller = np.matrix([[kpp, kip, kdp, 0, 0, 0, 0, 0, 0],
                                            [0, 0, 0, kpq, kiq, kdq, 0, 0, 0],
                                            [0, 0, 0, 0, 0, 0, kpr, kir, kdr]])
        gain_matrix_proportionals = np.matrix([[kpp, 0, 0],
                                              [0, kpq, 0],
                                               [0, 0, kpr]])
        gain_matrix_derivatives = np.matrix([[kdp, 0, 0],
                                               [0, kdq, 0],
                                               [0, 0, kdr]])


        '''task_to_rotor = np.matrix([[-1, 1, 1],
                                   [-1, -1, -1],
                                   [1, -1, 1],
                                   [1, 1, -1]])'''
        #Mixer as in the non neural PID
        task_to_rotor = np.matrix([[1, 1, -1, 1],
                                   [1, 1, 1, -1],
                                   [1, -1, 1, 1],
                                   [1, -1, -1, -1]])
    return gain_matrix_position, gain_matrix_virtual, gain_matrix_proportionals, gain_matrix_derivatives, task_to_rotor


