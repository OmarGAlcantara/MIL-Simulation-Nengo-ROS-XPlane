#!/usr/bin/python

# Omar Garcia
# Diego Chavez
# New Mexico State University

import sys
import time
import nengo
import subprocess
import os
import signal
import cPickle as pickle
import numpy as np
np.set_printoptions(threshold=sys.maxsize)
import os

import rospy
import matplotlib.pyplot as plt
from std_msgs.msg import Float32
import plotterDatas as pltr

import quadcopterROS as QUAD
from nav_msgs.msg import Odometry

#import PID_Nengo as Model

dt = 0.005


class QUADController():

    #def signal_handler(signal, frame):
    #print('Nengo Path:', nengo.__path__)

    def stop_other_code(self):
        # Killz the simulation of the conventionl PID when launched from the launch file for data comparisson
        process_name = "Control_QUAD_ROS_PID.py"
        os.system("pkill -f {}".format(process_name))

    def send_interrupt_to_other_code(self):
        # Assuming the other code's process name is "other_code.py"
        process_name = "Control_QUAD_ROS_PID.py"
        os.system("pkill -SIGINT -f {}".format(process_name))


    def simple_vertical(self):
        current_time = time.time() - self.reference_time
        #print('current time', current_time)
        if current_time < 10:
            return [0, 0, -current_time], [0, 0, 500]
        else:
            return [0, 0, -30], [0, 0, 500]

    def __init__(self):
        #pub = rospy.Publisher('chatter', Float32, queue_size=10)
        #rospy.init_node('talker', anonymous=True)
        #rate = rospy.Rate(10)  # 10hz

        self.is_time_set = False
        times = [0]

        if not self.is_time_set:                # Bandera para guardar el angulo de yaw inicial
            self.time_init = time.time()
            self.is_time_set = True

        #setting time to zero for height reference
        self.reference_time = time.time()
        self.zero_time = time.time() - self.reference_time
        #print("Elapsed time: ", self.zero_time)

        num_steps = 9000
        self.target_func = self.simple_vertical()
        model_name = 'PID_Nengo'
        exec("import %s as Model" % model_name)
        m = Model.Model(self.target_func)
        model = m.get_model()

        sim = nengo.Simulator(model, dt=dt, optimize=False)
        probesStateNode, probesStateEnsemble, probeEeOri, probeAngleVelInerDes, probe_pqrDesired, probes_posPID, probes_eAngVelBody, probes_DerErrorp, probes_DerErrorq, probes_DerErrorr, probe_adjustedDer, probe_ControlSignals, probe_Motors = m.get_probe() #TODO PROBES

        #print("running simulator...")

        xplane_state_data = []
        count = 0
        count_t = 0

        for i in range(num_steps):
            sim.step()
            self.target_func = self.simple_vertical()

            self.actual_time = time.time() - self.time_init
            times.append(self.actual_time)
            count += 1
            # TODO Revisar si el contador funciona
            if count == 1:
                count = 0



                t = sim.trange()
        dataProbesStateNode = sim.data[probesStateNode]
        dataProbesStateEnsemble = sim.data[probesStateEnsemble]#Probing from nengo Node
        #dataProbesErrorz = sim.data[probesErrorz]
        #dataProbesIntErrorz = sim.data[probeIntEz]
        dataProbeEOri = sim.data[probeEeOri]
        data_posPID = sim.data[probes_posPID]
        #data_eOri = sim.data[probes_eOri]
        #data_angleVelIner = sim.data[probe_angleVelIner]
        dataProbeAngleVelInerDes = sim.data[probeAngleVelInerDes]
        dataprobe_pqrDesired = sim.data[probe_pqrDesired]
        dataProbes_eAngVelBody = sim.data[probes_eAngVelBody]
        dataProbes_DerErrorp = sim.data[probes_DerErrorp]
        dataProbes_DerErrorq = sim.data[probes_DerErrorq]
        dataProbes_DerErrorr = sim.data[probes_DerErrorr]
        dataProbe_adjustedDer = sim.data[probe_adjustedDer]
        dataProbe_ControlSignals = sim.data[probe_ControlSignals]
        dataProbeMotors = sim.data[probe_Motors]




            #xplane_state_data.append(m.get_copter().get_state())   #Probing from Xplane

        timeLength = len(dataProbesStateNode[:,0])#29988#9996#29988

#        pltr.plot_3(times[:timeLength], dataProbesStateNode[:,0], dataProbesStateNode[:,1], dataProbesStateNode[:,2], 'Error x', 'Meters (m)', 'Error y', 'Meters(m)', 'Error z', 'Meters (m)', 'Nengo Node Position Errors')
#        pltr.plot_3(times[:timeLength], dataProbesStateNode[:,3], dataProbesStateNode[:,4], dataProbesStateNode[:,5], 'Error velocity x', 'Velocity (m/s)', 'Error velocity y', 'Velocity (m/s)', 'Error velocity z', 'Velocity (m/s)', 'Nengo Node Velocity Errors')
#        pltr.plot_3(times[:timeLength], dataProbesStateNode[:,6], dataProbesStateNode[:,7], dataProbesStateNode[:,8], 'Phi', 'Radians (rad)', 'Theta', 'Radians (rad)', 'Psi', 'Radians (rad)', 'Nengo Node Euler Angles')
#        pltr.plot_3(times[:timeLength], dataProbesStateNode[:,9], dataProbesStateNode[:,10], dataProbesStateNode[:,11], 'p','Angular Velocity (rad/s)', 'q', 'Angular Velocity (rad/s)', 'r', 'Angular Velocity (rad/s)', 'Nengo Node Angular Velocity Body Frame')
#        pltr.plot_1(times[:timeLength], dataProbesStateNode[:,12], 'Desired Psi', 'radians (rad)', 'Nengo Node Desired Psi')

#        pltr.plot_3(times[:timeLength], dataProbesStateEnsemble[:, 0], dataProbesStateEnsemble[:, 1], dataProbesStateEnsemble[:, 2], 'Error x', 'Meters (m)', 'Error y', 'Meters(m)', 'Error z', 'Meters (m)', 'Nengo Ensemble Position Errors')
#        pltr.plot_3(times[:timeLength], dataProbesStateEnsemble[:, 3], dataProbesStateEnsemble[:, 4], dataProbesStateEnsemble[:, 5], 'Error velocity x', 'Velocity (m/s)', 'Error velocity y', 'Velocity (m/s)', 'Error velocity z', 'Velocity (m/s)', 'Nengo Ensemble Velocity Errors')
        #pltr.plot_1(times[:timeLength], dataProbesStateEnsemble[:, 6], 'Integral Error z', 'Meters (m)', 'Nengo Ensemble Integral Error z')

#        pltr.plot_3_yaw(times[:timeLength], dataProbesStateEnsemble[:, 6], dataProbesStateEnsemble[:, 7], dataProbesStateEnsemble[:, 8], 'Phi', 'Radians (rad)', 'Theta', 'Radians (rad)', 'Psi', 'Radians (rad)', 'Nengo Ensemble Euler Angles')

#        pltr.plot_3(times[:timeLength], dataProbesStateEnsemble[:, 9], dataProbesStateEnsemble[:, 10], dataProbesStateEnsemble[:, 11], 'p', 'Angular Velocity (rad/s)', 'q', 'Angular Velocity (rad/s)', 'r', 'Angular Velocity (rad/s)', 'Nengo Ensemble Angular Velocity Body Frame')
#        pltr.plot_1(times[:timeLength], dataProbesStateEnsemble[:, 12], 'Desired Psi', 'radians (rad)', 'Nengo Ensemble Desired Psi')
#        pltr.plot_2_in_1(times[:timeLength], dataProbesStateEnsemble[:, 13], dataProbesStateEnsemble[:, 14], 'z', 'm', 'z_deseada', 'm', 'Altura z en Nengo')
        #pltr.plot_1(times[:timeLength], dataProbesIntErrorz, 'Integral Ez', 'm', 'Nengo Calculated Integral of ez')
#        pltr.plot_3(times[:timeLength], dataProbeEOri[:, 0], dataProbeEOri[:, 1], dataProbeEOri[:,2],'Error phi','rad','Error theta', 'rad', 'Error psi','rad', 'Nengo Orientation Errors')
#        pltr.plot_3(times[:timeLength], dataProbeAngleVelInerDes[:,0], dataProbeAngleVelInerDes[:,1], dataProbeAngleVelInerDes[:,2], 'Psi Dot Desired', 'rad/s', 'Theta Dot Desired', 'rad/s','Phi Dot Desired', 'rad/s',  'Nengo Desired Angular Velocities Inertial Frame')
#        pltr.plot_3(times[:timeLength],dataprobe_pqrDesired[:,0], dataprobe_pqrDesired[:,1], dataprobe_pqrDesired[:,2], 'p desired', 'rad/s', 'q desired', 'rad/s', 'r desired', 'rad/s', 'Nengo Desired Angular Velocities Body Frame')

        #pltr.plot_2(times[:timeLength],dataProbesStateNode[:,2], dataProbesStateEnsemble[:, 2], 'Error en z', 'm', 'Error en z normalizado', 'm', 'Nengo Error en z y el Error en z normalizado a 200')

#        pltr.plot_3(times[:timeLength], data_posPID[:,0], data_posPID[:,1], data_posPID[:,2], 'PDy', 'PDy units', 'PDx', 'PDx units', 'PIDz', 'PIDz units', 'Nengo PID Position')

        #pltr.plot_1(times[:timeLength], dataprobes_denorm_IntEz, 'Integral Error z', 'Meters (m)', 'Nengo Ensemble Integral Error z Denormalized')
        #pltr.plot_1(times[:timeLength], dataprobeInputIntegralEz, 'Integral Error z', 'Meters (m)', 'Nengo Input Integral Error z')
#        pltr.plot_3(times[:timeLength], dataProbes_eAngVelBody[:,0], dataProbes_eAngVelBody[:,1], dataProbes_eAngVelBody[:,2], 'error p', 'rad/s', 'error q', 'rad/s', 'error r', 'rad/s', 'Nengo Angular Velocity Errors Body Frame')
        #pltr.plot_3(times[:timeLength], dataProbes_DerErrorp, dataProbes_DerErrorq, dataProbes_DerErrorr, 'Derivative Error p','[]', 'Derivative Error q','[]', 'Derivative Error r','[]', 'Nengo Derivative of the Angular Velocity Error Body Frame')

        pltr.plot_2in1_3verticalDOS(times[:timeLength], dataProbes_DerErrorp, dataProbes_DerErrorq,
                                    dataProbes_DerErrorr,
                                    '$\dot{e}_{p}$', '$\dot{e}_{p}$ [rad/$s^2$]', '$\dot{e}_{q}$',
                                    '$\dot{e}_{q}$ [rad/$s^2$]', '$\dot{e}_{r}$', '$\dot{e}_{r}$ [rad/$s^2$]',
                                    'Nengo Derivative of the Angular Velocity Error Body Frame', 'upper right', [-0.6,0.6],
                                    [-0.4,0.4], None, [0, 130], 6, 4)






        #pltr.plot_3_Spec(times[:timeLength], dataProbe_adjustedDer[:,0], dataProbe_adjustedDer[:,1],  dataProbe_adjustedDer[:,2], 'Adjusted der p', '[]', 'Adjusted der q', '[]', 'Adjusted der r', '[]', 'Nengo Adjusted Der', [-0.6,0.6], [-1,1.5], [-1.5,2])
        pltr.plot_4_Spec(times[:timeLength], dataProbe_ControlSignals[:,0], dataProbe_ControlSignals[:,1], dataProbe_ControlSignals[:,2], dataProbe_ControlSignals[:,3], '$U_{z}$', '$U_{z}$ [N]', '$U_{p}$',
                     '$U_{p}$ [N]', '$U_{q}$', '$U_{q}$ [N]', '$U_{r}$', '$U_{r}$ [N]', 'Nengo Control Signals','upper right',  None, None,
                     None, None, [0,130], 7,3)
        pltr.plot_4_Spec(times[:timeLength], dataProbeMotors[:,0], dataProbeMotors[:,1], dataProbeMotors[:,2], dataProbeMotors[:,3],  '$U_{1}$', '$U_{1}$ [N]', '$U_{2}$', '$U_{2}$ [N]', '$U_{3}$', '$U_{3}$ [N]', '$U_{4}$', '$U_{4}$ [N]', 'Nengo Motors','upper right',  None, None, None, None, [0,130], 7, 4)





        pltr.plot_2in1_3vertical(times[:timeLength], dataProbesStateEnsemble[:, 9], dataprobe_pqrDesired[:,0], dataProbesStateEnsemble[:, 10],
                         dataprobe_pqrDesired[:,1], dataProbesStateEnsemble[:, 11], dataprobe_pqrDesired[:,2],  '$p$', '$p$ [rad/s]', '$p_d$', '$p$ [rad/s]', '$q$', '$q$ [rad/s]', '$q_d$', '$q$ [rad/s]', '$r$', '$r$ [rad/s]',
                     '$r_d$', '$r$ [rad/s]', 'Nengo Angular velocities Body Frame vel vs vel desired', 'upper right', [-0.2,0.16], [-0.2,0.16], None, [0,130], 6, 4)





        pltr.plot_2in1_3vertical(times[:timeLength], dataProbesStateEnsemble[:, 6], data_posPID[:,0],
                         dataProbesStateEnsemble[:, 7],
                         data_posPID[:,1], dataProbesStateEnsemble[:, 8], dataProbesStateEnsemble[:, 19], '$\phi$', '$\phi$ [rad]', '$\phi_{d}$', '$\phi$ [rad]', '$\Theta$', '$\Theta$ [rad]', '$\Theta_{d}$',
                             '$\Theta$ [rad]', '$\psi$', '$\psi$ [rad]', '$\psi_{d}$', '$\psi$ [rad]', 'Nengo Euler Angles vs Desired', 'upper right', [-0.15,0.35], [-0.24,0.16], None, [0,130], 6, 4)

#        pltr.plot_2in1_3(times[:timeLength], dataProbesStateEnsemble[:, 15], dataProbesStateEnsemble[:, 17], dataProbesStateEnsemble[:, 16],
#                         dataProbesStateEnsemble[:, 18], dataProbesStateEnsemble[:, 13], dataProbesStateEnsemble[:, 14], 'x', 'm', 'xd', 'm',
#                         'y',
#                         'm', 'y_d', 'm', 'z', 'm', 'z_d', 'm', 'Nengo Position vs Desired')

        pltr.plot_3d_trajectory(times[:timeLength], dataProbesStateEnsemble[:, 15], dataProbesStateEnsemble[:, 16],
                           -dataProbesStateEnsemble[:, 13],
                           dataProbesStateEnsemble[:, 17], dataProbesStateEnsemble[:, 18],
                           -dataProbesStateEnsemble[:, 14],
                           'x', 'm', 'y', 'm', '-z',
                            'm', 'Nengo 3D Trajectory', 'upper center', lim_x=None, lim_y=None, lim_z=None)
#        pltr.plot_xz_plane(times[:timeLength], dataProbesStateEnsemble[:, 15], -dataProbesStateEnsemble[:, 13], dataProbesStateEnsemble[:, 17], -dataProbesStateEnsemble[:, 14], 'x', 'm', 'z', 'm', 'NENGO x z')
#        pltr.plot_xz_plane(times[:timeLength], dataProbesStateEnsemble[:, 15], dataProbesStateEnsemble[:, 16],
#                           dataProbesStateEnsemble[:, 17], dataProbesStateEnsemble[:, 18], 'x', 'm', 'y', 'm',
#                           'NENGO x y')

        #Plotting
        refer_z = xplane_state_data

        refer_z = np.asarray(refer_z[0:len(t)])



        self.send_interrupt_to_other_code()
        '''with open('data_NengoIntErrorZ.txt', 'w') as file:
            # Iterate over the times and dataProbes lists simultaneously
            for timex, probe in zip(times[:timeLength], dataprobes_denorm_IntEz):
                # Write the values in two columns separated by a tab
                file.write('{}\t{}\n'.format(timex, probe[0]))'''




def main():
  QUADcon = QUADController()


if __name__ == '__main__':
  try:
    main()
  except rospy.ROSInterruptException:
    #plt.close()
    pass



