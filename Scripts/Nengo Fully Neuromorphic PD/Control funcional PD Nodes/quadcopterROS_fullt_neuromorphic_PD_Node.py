#################################################################################
#                               04/02/2024
#author: Omar Garcia
#github: https://github.com/OmarGAlcantara/
#    XPlane Connection File

# This is a sample script for a Neuromorphic PD Control for a MIL simulation with XPlane 11 by using XPlaneConnect and XPlaneROS for the communication
# The PD Controller was developed and programmed using Nengo and is based on a classic PD control strategy
# Simulation should be executed in the following order:
# 1. Opening a simulation in XPlane 11 with the intelAeroRTF model
# 2. Launching the XplaneROS wrapper by typing in the terminal: roslaunch xplane_ros default.launch
# 3. Launching the XplaneROS launch file of the controller roslaunch nengo_pid nengo.launch

#This controller effectively performs a quadcopter's tracking task of an ascensional ramp and a circle using a neuromorphic PD Control strategy
#################################################################################

import numpy as np
import time
import rospy
import Control_utlis as utlis
from nav_msgs.msg import Odometry
import xplane_ros.msg as xplane_msgs
import rosplane_msgs.msg as rosplane_msgs

sim_dt = 0.01
dt = 0.001

class Quadcopter(object):
    def __init__(self, target_func):
        rospy.init_node('xplane_subs', anonymous=True)

        self.actual_time = 0
        self.time_init = None
        self.is_time_set = False
        self.reference_time = time.time()
        self.last_execution_time = time.time()

        self.target_func = target_func

        self.pos_err = [0, 0, 0]
        self.lin = [0, 0, 0]
        self.ori_err = [0, 0, 0]
        self.ang = [0, 0, 0]
        self.count = 0
        self.xplane_ori = [0, 0, 4.39]

        self.xplane_pos = [0, 0, 0]
        self.desiredZ, _ = self.reference()
        _, self.velz_des = self.reference()

        self.xplane_desired_ori = [0, 0, 4.39]
        self.lin_vel_err = [0,0,0]
        self.xplane_lin_vel = [0, 0, 0]
        self.xplane_ang_vel = [0,0,0]
        self.filtered_data_p = []
        self.filtered_data_q = []

        self.is_yaw_set = False
        self.phi = 0
        self.theta = 0
        self.p2 = 0

        self.IntZ = 0
        self.x_d = 0
        self.y_d = 0
        self.xplane_desired_pos = [self.x_d, self.y_d, self.desiredZ]

        rospy.Subscriber("/xplane/flightmodel/global_state", xplane_msgs.GlobalState, self.callback)
        rospy.Subscriber("/fixedwing/xplane/state", rosplane_msgs.State, self.callbackpqr)
        rospy.Subscriber("/xplane/flightmodel/odom", Odometry, self.odomcallback)
        self.rate = rospy.Rate(100)

    def callbackpqr(self, data):
        self.p = data.p
        self.q = data.q
        self.r = data.r

    def callback(self, data):
        self.phi = data.roll  * (3.14 / 180)
        self.theta = data.pitch  * (3.14 / 180)
        self.psi = data.heading  * (3.14 / 180)
        if not self.is_yaw_set:  # Bandera para guardar el angulo de yaw inicial
            self.yaw_init = data.heading * (3.14 / 180)
            self.is_yaw_set = True
        if not self.is_time_set:  # Bandera para guardar el angulo de yaw inicial
            self.time_init = time.time()
            self.is_time_set = True

        self.xplane_ori = [0, 0, self.yaw_init]

    def odomcallback(self, data):
        # XPlane Position
        self.x = data.pose.pose.position.x
        self.y = data.pose.pose.position.y
        self.z = data.pose.pose.position.z

        #XPlane Linear Velocity
        self.vx = data.twist.twist.linear.x
        self.vy = data.twist.twist.linear.y
        self.vz = data.twist.twist.linear.z

        self.references = self.reference()
        self.x_desired = self.referencex(self.x)
        self.y_desired = self.referencey(self.y)

# Altitude reference
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

# Circular reference
    def referencex(self, x):
        current_timex = time.time() - self.reference_time
        if current_timex < 5:
            return 0
        elif current_timex < 35:
            ramp_timex = current_timex - 5
            ramp_valuex = 0.66 * ramp_timex
            return ramp_valuex
        elif current_timex < 50:
            return 19.5
        elif current_timex < 120:
            r = 19.5
            circleTime = 0.2 * (time.time() - self.reference_time)
            circle = r * np.cos(circleTime-np.pi*1.2)
            return circle

    def referencey(self, x):
        current_timex = time.time() - self.reference_time
        if current_timex < 5:
            return 0
        elif current_timex < 35:
            ramp_timex = current_timex - 5
            ramp_valuex = 0.66 * ramp_timex
            return ramp_valuex
        elif current_timex < 50:
            return 19.5
        elif current_timex < 120:
            r = 19.5
            circleTimey = 0.2 * (time.time() - self.reference_time)
            circley = r * np.sin(circleTimey-np.pi*1.2) + 19.5
            return circley

    def get_target(self):
        self.desiredZ, _ = self.references
        self.x_d = self.x_desired
        self.y_d = self.y_desired

        self.xplane_desired_pos = [self.x_d,self.y_d,self.desiredZ]
        _, self.velz_des = self.references
        self.xplane_desired_ori = [0,0,6]

    def calculate_error(self):
        self.xplane_ori = [self.phi, self.theta, self.psi]
        self.xplane_pos = [self.x, self.y, self.z]
        self.xplane_lin_vel = [self.vx, self.vy, self.vz]
        self.xplane_ang_vel = [self.p, self.q, self.r]

        # Wrap Psi Angle
        if self.xplane_desired_ori[2] < np.pi/2 and self.xplane_ori[2] > 3*np.pi/2:
            self.error_yaw = -(self.xplane_desired_ori[2] + (2*np.pi - self.xplane_ori[2]))
            if abs(self.error_yaw) > np.pi:
                self.error_yaw = -self.error_yaw
        elif self.xplane_ori[2] < np.pi/2 and self.xplane_desired_ori[2] > 3*np.pi/2:
            self.error_yaw = -(-(2*np.pi - self.xplane_desired_ori[2]) - self.xplane_ori[2])
            if abs(self.error_yaw) > np.pi:
                self.error_yaw = -self.error_yaw
        else:
            self.error_yaw = self.xplane_ori[2] - self.xplane_desired_ori[2]
            '''if abs(self.error_yaw) > np.pi:
                self.error_yaw = -self.error_yaw'''

        self.ori_err = [-self.xplane_ori[0] + self.xplane_desired_ori[0],
                        -self.xplane_ori[1] + self.xplane_desired_ori[1],
                        self.error_yaw]

        self.pos_err = [self.xplane_pos[0] - self.xplane_desired_pos[0],
                        self.xplane_pos[1] - self.xplane_desired_pos[1],
                        self.xplane_pos[2] - self.xplane_desired_pos[2]]

        self.lin_vel_err = [self.xplane_lin_vel[0] - 0,
                            self.xplane_lin_vel[1] - 0,
                            self.xplane_lin_vel[2] - self.velz_des]

    def send_motor_commands(self, values):
        self.Throttle1 = values[0]
        self.Throttle2 = values[1]
        self.Throttle3 = values[2]
        self.Throttle4 = values[3]
        # Sending the motor Commands to XPlane using XPlane_Connect
        with utlis.XPlaneConnect() as client:
            self.data = [\
                [25, self.Throttle1, self.Throttle2, self.Throttle3, self.Throttle4, -998, -998, -998, -998], \
                [8, -998, -998, -998, -998, -998, -998, -998, -998], \
                ]

            client.sendDATA(self.data)
        self.actual_time = time.time() - self.time_init

    def handle_input(self, values):
        # Send motor commands to XPlane
        self.send_motor_commands(values)  # Remember this script handles the whole communication from and to XPlane. That's why it sends the Motors also.

        # Retrieve desired location
        self.get_target()

        # Calculate state error
        self.calculate_error()

        # Calculate rate of publication
        current_time = time.time()

    def handle_output(self):
        return [self.pos_err[0], self.pos_err[1], self.pos_err[2],
                self.lin_vel_err[0], self.lin_vel_err[1], self.lin_vel_err[2],
                self.xplane_ori[0], self.xplane_ori[1], self.xplane_ori[2],
                self.xplane_ang_vel[0], self.xplane_ang_vel[1], self.xplane_ang_vel[2],
                self.error_yaw, self.xplane_pos[2], self.xplane_desired_pos[2],
                self.xplane_pos[0], self.xplane_pos[1],
                self.xplane_desired_pos[0], self.xplane_desired_pos[1], self.xplane_desired_ori[2]]

    def __call__(self, t, values):
        # TODO Check the frequency of xplane and nengo
        self.count += 1
        if self.count == 1:
            self.count = 0
            self.handle_input(values)
            self.rate.sleep()

        return self.handle_output()
