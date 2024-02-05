#!/usr/bin/python
#################################################################################
#                               04/02/2024
#author: Omar Garcia
#github: https://github.com/OmarGAlcantara/
#    Executable File

# This is a sample script for a Neuromorphic PD Control for a MIL simulation with XPlane 11 by using XPlaneConnect and XPlaneROS for the communication
# The PD Controller was developed and programmed using Nengo and is based on a classic PD control strategy
# Simulation should be executed in the following order:
# 1. Opening a simulation in XPlane 11 with the intelAeroRTF model
# 2. Launching the XplaneROS wrapper by typing in the terminal: roslaunch xplane_ros default.launch
# 3. Launching the XplaneROS launch file of the controller roslaunch nengo_pid nengo.launch

#This controller effectively performs a quadcopter's tracking task of an ascensional ramp and a circle using a neuromorphic PD Control strategy
#################################################################################

import sys
import time
import nengo
import numpy as np
np.set_printoptions(threshold=sys.maxsize)
import os
import rospy
import plotterDatas as pltr

dt = 0.005

class QUADController():
    def send_interrupt_to_other_code(self):
        # Stopping parallel code if any
        process_name = "Control_QUAD_ROS_PID.py"
        os.system("pkill -SIGINT -f {}".format(process_name))

    def simple_vertical(self):
        None

    def __init__(self):
        self.is_time_set = False
        times = [0]

        if not self.is_time_set:                # Bandera para guardar el angulo de yaw inicial
            self.time_init = time.time()
            self.is_time_set = True

        num_steps = 9000     # Arbitrary numsteps for simulation length
        self.target_func = self.simple_vertical()
        model_name = 'PID_Nengo'     # Control Law Python Script name
        exec("import %s as Model" % model_name)
        m = Model.Model(self.target_func)
        model = m.get_model()

        sim = nengo.Simulator(model, dt=dt, optimize=False)
        probesStateNode, probesStateEnsemble, probeEeOri, probeAngleVelInerDes, probe_pqrDesired, probes_posPID, probes_eAngVelBody, probes_DerErrorp, probes_DerErrorq, probes_DerErrorr, probe_adjustedDer, probe_ControlSignals, probe_Motors = m.get_probe()

        count = 0

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
        dataProbesStateEnsemble = sim.data[probesStateEnsemble]
        data_posPID = sim.data[probes_posPID]
        dataprobe_pqrDesired = sim.data[probe_pqrDesired]
        dataProbes_DerErrorp = sim.data[probes_DerErrorp]
        dataProbes_DerErrorq = sim.data[probes_DerErrorq]
        dataProbes_DerErrorr = sim.data[probes_DerErrorr]
        dataProbe_ControlSignals = sim.data[probe_ControlSignals]
        dataProbeMotors = sim.data[probe_Motors]

            #xplane_state_data.append(m.get_copter().get_state())   #Probing from Xplane

        timeLength = len(dataProbesStateNode[:,0])

        pltr.plot_2in1_3verticalDOS(times[:timeLength], dataProbes_DerErrorp, dataProbes_DerErrorq,
                                    dataProbes_DerErrorr,
                                    '$\dot{e}_{p}$', '$\dot{e}_{p}$ [rad/$s^2$]', '$\dot{e}_{q}$',
                                    '$\dot{e}_{q}$ [rad/$s^2$]', '$\dot{e}_{r}$', '$\dot{e}_{r}$ [rad/$s^2$]',
                                    'Nengo Derivative of the Angular Velocity Error Body Frame', 'upper right', [-0.8,0.8],
                                    [-0.8,0.8], None, [0, 130], 6, 4)

        pltr.plot_4_Spec(times[:timeLength], dataProbe_ControlSignals[:,0], dataProbe_ControlSignals[:,1], dataProbe_ControlSignals[:,2], dataProbe_ControlSignals[:,3], '$U_{z}$', '$U_{z}$ [N]', '$U_{p}$',
                     '$U_{p}$', '$U_{q}$', '$U_{q}$', '$U_{r}$', '$U_{r}$', 'Nengo Control Signals','upper right',  None, None,
                     None, None, [0,130], 7,3)
        pltr.plot_4_Spec(times[:timeLength], dataProbeMotors[:,0], dataProbeMotors[:,1], dataProbeMotors[:,2], dataProbeMotors[:,3],  '$U_{1}$', '$U_{1}$ [N]', '$U_{2}$', '$U_{2}$ [N]', '$U_{3}$', '$U_{3}$ [N]', '$U_{4}$', '$U_{4}$ [N]', 'Nengo Motors','upper right',  None, None, None, None, [0,130], 7, 4)

        pltr.plot_2in1_3vertical(times[:timeLength], dataProbesStateEnsemble[:, 9], dataprobe_pqrDesired[:,0], dataProbesStateEnsemble[:, 10],
                         dataprobe_pqrDesired[:,1], dataProbesStateEnsemble[:, 11], dataprobe_pqrDesired[:,2],  '$p$', '$p$ [rad/s]', '$p_d$', '$p$ [rad/s]', '$q$', '$q$ [rad/s]', '$q_d$', '$q$ [rad/s]', '$r$', '$r$ [rad/s]',
                     '$r_d$', '$r$ [rad/s]', 'Nengo Angular velocities Body Frame vel vs vel desired', 'upper right', [-0.4,0.4], [-0.4,0.4], None, [0,130], 6, 4)

        pltr.plot_2in1_3vertical(times[:timeLength], dataProbesStateEnsemble[:, 6], data_posPID[:,0],
                         dataProbesStateEnsemble[:, 7],
                         data_posPID[:,1], dataProbesStateEnsemble[:, 8], dataProbesStateEnsemble[:, 19], '$\phi$', '$\phi$ [rad]', '$\phi_{d}$', '$\phi$ [rad]', '$\Theta$', '$\Theta$ [rad]', '$\Theta_{d}$',
                             '$\Theta$ [rad]', '$\psi$', '$\psi$ [rad]', '$\psi_{d}$', '$\psi$ [rad]', 'Nengo Euler Angles vs Desired', 'upper right', [-0.15,0.35], [-0.15,0.25], None, [0,130], 6, 4)

        pltr.plot_3d_trajectory(times[:timeLength], dataProbesStateEnsemble[:, 15], dataProbesStateEnsemble[:, 16],
                           -dataProbesStateEnsemble[:, 13],
                           dataProbesStateEnsemble[:, 17], dataProbesStateEnsemble[:, 18],
                           -dataProbesStateEnsemble[:, 14],
                           'x', 'm', 'y', 'm', '-z',
                            'm', 'Nengo 3D Trajectory', 'upper center', lim_x=None, lim_y=None, lim_z=[0,30], num_xticks=5, num_yticks=5, num_zticks=5)

        self.send_interrupt_to_other_code()

def main():
  QUADcon = QUADController()

if __name__ == '__main__':
  try:
    main()
  except rospy.ROSInterruptException:
    pass
