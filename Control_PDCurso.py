#!/usr/bin/env python3
#################################################################################
#                               07/11/2023
#author: Omar García
#github: https://github.com/OmarGAlcantara/MIL-Nengo-XPlane

# This is a sample script for a PD Control for a MIL simulation with XPlane 11 by using XPlaneConnect and XPlaneROS for the communication
# Simulation should be executed in the following order:
# 1. Opening a simulation in XPlane 11 with the intelAeroRTF model
# 2. Launching the XplaneROS wrapper by typing in the terminal: roslaunch xplane_ros default.launch
# 3. typing in a terminal in the file location : python Simple_Control_QUAD_ROS_PIDCurso.py
#################################################################################

import sys
import time
import signal
import rospy
import Control_utlis as utlis  # XPC UDP sending functions
import matplotlib.pyplot as plt
import numpy as np
from nav_msgs.msg import Odometry
import socket
import xplane_ros.msg as xplane_msgs
import rosplane_msgs.msg as rosplane_msgs


QUADcon = None
UDP_PORT = 49005
UDP_IP = "127.0.0.1"
sock = socket.socket(socket.AF_INET,  # Internet
                     socket.SOCK_DGRAM)  # UDP
sock.bind((UDP_IP, UDP_PORT))

def pltr(t, data_1, data_2, data_3, name_1, units_1, name_2, units_2, name_3, units_3, figure_name):
    fig, axs = plt.subplots(1, 3, figsize=(15, 10))

    # Plot Input (first column, first row)
    axs[0].plot(t, data_1)
    axs[0].set_title(name_1)
    axs[0].set_xlabel("Time (s)")
    axs[0].set_ylabel(units_1)

    # Plot Graph 1 (second column, first row)
    axs[1].plot(t, data_2)
    axs[1].set_title(name_2)
    axs[1].set_xlabel("Time (s)")
    axs[1].set_ylabel(units_2)

    # Plot Graph 2 (third column, first row)
    axs[2].plot(t, data_3)
    axs[2].set_title(name_3)
    axs[2].set_xlabel("Time (s)")
    axs[2].set_ylabel(units_3)

    # plt.ylim(-0.01, 0.01)
    axs[0].grid(True), axs[1].grid(True), axs[2].grid(True)
    plt.tight_layout()
    # Define the file path with the provided figure_name appended
    file = '/home/route...' + figure_name

    plt.savefig(file)
    plt.clf()
    plt.close()

class QUADController():
    def __init__(self):
        rospy.init_node('listener', anonymous=True)

        self.saved_data = []

        self.yaw_init = None
        self.is_yaw_set = False

        self.time_init = None
        self.is_time_set = False

        self.q = 0
        self.p = 0
        self.r = 0
        self.x = 0
        self.y = 0
        self.altura = 0
        self.vx = 0
        self.vy = 0
        self.vz = 0

        self.pitch_des = 0
        self.roll_des = 0

        self.reference_time = time.time()
        self.actual_time = 0
        self.altura_des = 0
        self.x_d = 0
        self.y_d = 0
        self.velz_des = 0

        self.sumEr_p = 0
        self.Er0_p = 0
        self.sumEr_q = 0
        self.Er0_q = 0
        self.sumEr_r = 0
        self.Er0_r = 0
        self.sumEr_h = 0
        self.sumEr_h_norm = 0

        self.Int_z = 0
        self.ez = 0

        rospy.Subscriber("/xplane/flightmodel/odom", Odometry, self.odomcallback)
        rospy.Subscriber("/fixedwing/xplane/state", rosplane_msgs.State, self.callbackpqr)
        rospy.Subscriber("/xplane/flightmodel/global_state", xplane_msgs.GlobalState, self.callback)

    def reference(self):
        current_time = time.time() - self.reference_time
        if current_time < 5:
            return -0.5, 0
        elif current_time < 35:
            ramp_time = current_time - 5
            ramp_value = -0.5 + ((-30 - (-0.5)) / 30) * ramp_time
            return ramp_value, (-30 - (-0.5)) / 30
        else:
            return -30, 0

    def referencexy(self):
        current_timex = time.time() - self.reference_time

        if current_timex < 5:
            return 0
        elif current_timex < 35:
            ramp_timex = current_timex - 5
            ramp_valuex = 0.16 * ramp_timex
            return ramp_valuex
        else:
            return 5

    def odomcallback(self, data):
        self.x = data.pose.pose.position.x
        self.y = data.pose.pose.position.y
        self.z = data.pose.pose.position.z
        self.vx = data.twist.twist.linear.x
        self.vy = data.twist.twist.linear.y
        self.vz = data.twist.twist.linear.z
        self.secs = data.header.stamp.secs

    def callbackpqr(self, data):
        self.p = data.p
        self.q = data.q
        self.r = data.r
        self.altura = data.position[2]

    def callback(self, data):
        self.phi = data.roll
        self.theta = data.pitch
        self.psi = data.heading

        if not self.is_yaw_set:  # Initial yaw
            self.yaw_init = data.heading
            self.is_yaw_set = True

        if not self.is_time_set:
            self.time_init = time.time()
            self.is_time_set = True

        #Control Gains
        self.kpx = 0.03
        self.kdx = 0.08
        self.kpy = -0.03 * 4.5
        self.kdy = -0.11 * 1.2
        self.k_pz = 0.02 * 8
        self.k_dz = 0.06 * 4.0

        self.k_phi = 2.2
        self.k_theta = 1.8
        self.k_psi = -1

        self.kp_p = 0.045 * 4
        self.kp_d = 0.035 * 2

        self.kq_p = 0.06 * 4
        self.kq_d = 0.035 * 2

        self.kr_p = 0.19
        self.kr_d = 0.1

        self.dt = 0.05

############    Calculating the control law    ##################
        self.x_d = self.referencexy()
        self.y_d = self.referencexy()

        self.error_x = self.x - self.x_d
        self.error_y = self.y - self.y_d
        self.error_velx = self.vx - 0
        self.error_vely = self.vy - 0

        self.phi_d = self.kpy * self.error_y + self.kdy * self.error_vely
        self.theta_d = self.kpx * self.error_x + self.kdx * self.error_velx

        if self.actual_time > 35 and self.actual_time < 150:
            self.phi_d = 0.1745
            None

        self.phi_rad = self.phi * (3.14 / 180)
        self.theta_rad = self.theta * (3.14 / 180)
        self.psi_rad = self.psi * (3.14 / 180)
        self.psi_rad_d = 2*np.pi

        self.error_roll = -self.phi_rad + self.phi_d
        self.error_pitch = -self.theta_rad + self.theta_d

        # Wrap Psi Angle
        if self.psi_rad_d < np.pi / 2 and self.psi_rad > 3 * np.pi / 2:
            self.error_yaw = -(self.psi_rad_d + (2 * np.pi - self.psi_rad))
            if abs(self.error_yaw) > np.pi:
                self.error_yaw = -self.error_yaw
        elif self.psi_rad < np.pi / 2 and self.psi_rad_d > 3 * np.pi / 2:
            self.error_yaw = -(-(2 * np.pi - self.psi_rad_d) - self.psi_rad)
            if abs(self.error_yaw) > np.pi:
                self.error_yaw = -self.error_yaw
        else:
            self.error_yaw = self.psi_rad - self.psi_rad_d


        self.phi_dot_d = self.k_phi * self.error_roll
        self.theta_dot_d = self.k_theta * self.error_pitch
        self.psi_dot_d = self.k_psi * self.error_yaw

        # Tranforming into body axes
        self.eta_dot = np.array([self.psi_dot_d, self.theta_dot_d, self.phi_dot_d])

        ROT = np.array([[-np.sin(self.theta_rad), 0, 1],
                        [np.cos(self.theta_rad) * np.sin(self.phi_rad), np.cos(self.phi_rad), 0],
                        [np.cos(self.theta_rad) * np.cos(self.phi_rad), -np.sin(self.phi_rad), 0]])

        Omega_d = np.dot(ROT, self.eta_dot)

        self.error_p = -self.p + Omega_d[0]
        self.error_q = -self.q + Omega_d[1]
        self.error_r = -(self.r - Omega_d[2])

        # Derivating and Integrating the error
        P_p = self.kp_p * self.error_p
        Der_p = ((self.error_p - self.Er0_p) / self.dt)
        D_p = self.kp_d * Der_p
        self.Er0_p = self.error_p
        self.sumEr_p = self.error_p + self.sumEr_p
        self.PID_p = P_p + D_p

        P_q = self.kq_p * self.error_q
        Der_q = ((self.error_q - self.Er0_q) / self.dt)
        D_q = self.kq_d * Der_q
        self.Er0_q = self.error_q
        self.sumEr_q = self.error_q + self.sumEr_q
        self.PID_q = P_q + D_q

        P_r = self.kr_p * self.error_r
        Der_r = ((self.error_r - self.Er0_r) / self.dt)
        D_r = self.kr_d * Der_r
        self.Er0_r = self.error_r
        self.sumEr_r = self.error_r + self.sumEr_r
        self.PID_r = (P_r + D_r)

        # Altitude Control
        _, self.velz_des = self.reference()
        self.altura_des, _ = self.reference()
        self.error_z = self.altura - self.altura_des
        self.error_velz = self.vz - self.velz_des
        P_z = self.error_z * self.k_pz
        D_z = self.error_velz * self.k_dz

        self.PDz = P_z + D_z

        self.Throttle1 = self.PDz - self.PID_p + self.PID_q + self.PID_r
        self.Throttle2 = self.PDz - self.PID_p - self.PID_q - self.PID_r
        self.Throttle3 = self.PDz + self.PID_p - self.PID_q + self.PID_r
        self.Throttle4 = self.PDz + self.PID_p + self.PID_q - self.PID_r
        ########################################################################################################

        with utlis.XPlaneConnect() as client:
            self.data = [ \
                [25, self.Throttle1, self.Throttle2, self.Throttle3, self.Throttle4, -998, -998, -998, -998], \
                [8, -998, -998, -998, -998, -998, -998, -998, -998], \
                ]
            client.sendDATA(self.data)

        # Saving data for plotting
        self.actual_time = time.time() - self.time_init

        self.saved_data.append([self.actual_time])
        self.saved_data[-1].append(self.error_x)
        self.saved_data[-1].append(self.error_y)
        self.saved_data[-1].append(self.error_z)

def handle_interrupt(signal, frame):
    # This function is used to graph
    t = QUADcon.saved_data

    actual_time = [row[0] for row in t]
    error_x = [row[1] for row in t]
    error_y = [row[2] for row in t]
    error_z = [row[3] for row in t]

    #Plotting example
    pltr(actual_time, error_x[:len(actual_time)], error_y[:len(actual_time)], error_z[:len(actual_time)],
                'Error x', 'Meters (m)', 'Error y', 'Meters(m)', 'Error z', 'Meters (m)', 'X-Plane Position Errors')

    sys.exit(0)

def main():
    global QUADcon
    QUADcon = QUADController()
    signal.signal(signal.SIGINT, handle_interrupt)
    rospy.spin()

if __name__ == '__main__':
    main()


