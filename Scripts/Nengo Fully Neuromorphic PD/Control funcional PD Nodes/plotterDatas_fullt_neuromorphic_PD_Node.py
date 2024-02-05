#################################################################################
#                               04/02/2024
#author: Omar Garcia
#github: https://github.com/OmarGAlcantara/
#    Plotting Scripts File

# This is a sample script for a Neuromorphic PD Control for a MIL simulation with XPlane 11 by using XPlaneConnect and XPlaneROS for the communication
# The PD Controller was developed and programmed using Nengo and is based on a classic PD control strategy
# Simulation should be executed in the following order:
# 1. Opening a simulation in XPlane 11 with the intelAeroRTF model
# 2. Launching the XplaneROS wrapper by typing in the terminal: roslaunch xplane_ros default.launch
# 3. Launching the XplaneROS launch file of the controller roslaunch nengo_pid nengo.launch

#This controller effectively performs a quadcopter's tracking task of an ascensional ramp and a circle using a neuromorphic PD Control strategy
#################################################################################

import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D  # Import the 3D plotting toolkit

plt.rcParams['font.family'] = 'serif'  # Use a generic serif font
plt.rcParams['font.serif'] = 'DejaVu Serif'
plt.rcParams['pdf.fonttype'] = 42

def plot_1(t, data_1, name_1, units_1, figure_name):
    fig, axs = plt.subplots(1, 1, figsize=(15, 10))

    # Plot Input (first column, first row)
    axs.plot(t, data_1)
    axs.set_title(name_1)
    axs.set_xlabel("Time (s)", fontsize=32)
    axs.set_ylabel(units_1, fontsize=32)

    axs.grid(True)
    plt.tight_layout()

    # Define the file path with the provided figure_name appended
    file = '/home/omarg/control_quad_ws/src/nengo_pid/scripts/Results/New_Data/' + figure_name

    plt.savefig(file)
    plt.clf()
    plt.close()

def plot_2(t, data_1, data_2, name_1, units_1, name_2, units_2, figure_name):
    fig, axs = plt.subplots(1, 2, figsize=(15, 10))

    # Plot Input (first column, first row)
    axs[0].plot(t, data_1)
    axs[0].set_title(name_1)
    axs[0].set_xlabel("Time (s)", fontsize=32)
    axs[0].set_ylabel(units_1, fontsize=32)

    # Plot Graph 1 (second column, first row)
    axs[1].plot(t, data_2)
    axs[1].set_title(name_2)
    axs[1].set_xlabel("Time (s)", fontsize=32)
    axs[1].set_ylabel(units_2, fontsize=32)

    axs[0].grid(True), axs[1].grid(True)
    plt.tight_layout()

    # Define the file path with the provided figure_name appended
    file = '/home/omarg/control_quad_ws/src/nengo_pid/scripts/Results/New_Data/' + figure_name

    plt.savefig(file, format="pdf", bbox_inches='tight')
    plt.clf()
    plt.close()

def plot_3(t, data_1, data_2, data_3, name_1, units_1, name_2, units_2, name_3, units_3, figure_name):
    fig, axs = plt.subplots(1, 3, figsize=(15, 10))

    # Plot Input (first column, first row)
    axs[0].plot(t, data_1)
    axs[0].set_title(name_1)
    axs[0].set_xlabel("Time (s)", fontsize=32)
    axs[0].set_ylabel(units_1, fontsize=32)

    # Plot Graph 1 (second column, first row)
    axs[1].plot(t, data_2)
    axs[1].set_title(name_2)
    axs[1].set_xlabel("Time (s)", fontsize=32)
    axs[1].set_ylabel(units_2, fontsize=32)

    # Plot Graph 2 (third column, first row)
    axs[2].plot(t, data_3)
    axs[2].set_title(name_3)
    axs[2].set_xlabel("Time (s)", fontsize=32)
    axs[2].set_ylabel(units_3, fontsize=32)

    # plt.ylim(-0.01, 0.01)
    axs[0].grid(True), axs[1].grid(True), axs[2].grid(True)
    plt.tight_layout()
    # Define the file path with the provided figure_name appended
    file = '/home/omarg/control_quad_ws/src/nengo_pid/scripts/Results/New_Data/' + figure_name

    plt.savefig(file, format="pdf", bbox_inches='tight')
    plt.clf()
    plt.close()

def plot_2_in_1(t, data_1, data_2, name_1, units_1, name_2, units_2, figure_name, num_xticks = 5, num_yticks = 5):
    fig, ax = plt.subplots(figsize=(15, 15))

    # Plot data_1
    ax.plot(t, data_1, label=name_1, color = 'blue', linewidth = 2)
    ax.set_xlabel("Time (s)", fontsize=32)
    ax.set_ylabel(units_1, fontsize=32)

    # Plot data_2
    ax.plot(t, data_2, label=name_2, color = 'red', linewidth = 2)
    ax.set_xlabel("Time (s)", fontsize=32)
    ax.set_ylabel(units_2, fontsize=32)

    ax.grid(True)
    ax.legend(fontsize=32)
    ax.xaxis.set_tick_params(labelsize=28)
    ax.yaxis.set_tick_params(labelsize=28)

    plt.locator_params(axis='x', nbins=num_xticks)
    plt.locator_params(axis='y', nbins=num_yticks)

    plt.tight_layout()

    # Define the file path with the provided figure_name appended
    file = '/home/omarg/control_quad_ws/src/nengo_pid/scripts/Results/New_Data/' + figure_name

    plt.savefig(file, format="pdf", bbox_inches='tight')
    plt.clf()
    plt.close()

def plot_3_yaw(t, data_1, data_2, data_3, name_1, units_1, name_2, units_2, name_3, units_3, figure_name):
    fig, axs = plt.subplots(1, 3, figsize=(15, 10))

    # Plot Input (first column, first row)
    axs[0].plot(t, data_1)
    axs[0].set_title(name_1)
    axs[0].set_xlabel("Time (s)", fontsize=32)
    axs[0].set_ylabel(units_1, fontsize=32)

    # Plot Graph 1 (second column, first row)
    axs[1].plot(t, data_2)
    axs[1].set_title(name_2)
    axs[1].set_xlabel("Time (s)", fontsize=32)
    axs[1].set_ylabel(units_2, fontsize=32)

    # Plot Graph 2 (third column, first row)
    axs[2].plot(t, data_3)
    axs[2].set_title(name_3)
    axs[2].set_xlabel("Time (s)", fontsize=32)
    axs[2].set_ylabel(units_3, fontsize=32)
    #axs[2].set_ylim(4.3, 4.44)

    # plt.ylim(-0.01, 0.01)
    axs[0].grid(True), axs[1].grid(True), axs[2].grid(True)
    plt.tight_layout()
    # Define the file path with the provided figure_name appended
    file = '/home/omarg/control_quad_ws/src/nengo_pid/scripts/Results/New_Data/' + figure_name

    plt.savefig(file, format="pdf", bbox_inches='tight')
    plt.clf()
    plt.close()

def plot_3vertical(t, data_1, data_2, data_3, name_1, units_1, name_2, units_2, name_3, units_3, figure_name, num_xticks=5, num_yticks=5):
    fig, axs = plt.subplots(3, 1, figsize=(15, 24))

    # Plot Input (first column, first row)
    axs[0].plot(t, data_1, color='orange', linewidth=2)
    #axs[0].set_title(name_1)
    axs[0].set_ylabel(units_1, fontsize=32)
    axs[0].set_xlabel("Time (s)", fontsize=32)  # Set x-axis label here

    # Plot Graph 1 (second column, first row)
    axs[1].plot(t, data_2, color='orange', linewidth=2)
    #axs[1].set_title(name_2)
    axs[1].set_ylabel(units_2, fontsize=32)
    axs[1].set_xlabel("Time (s)", fontsize=32)  # Set x-axis label here

    # Plot Graph 2 (third column, first row)
    axs[2].plot(t, data_3, color='orange', linewidth=2)
    #axs[2].set_title(name_3)
    axs[2].set_ylabel(units_3, fontsize=32)
    axs[2].set_xlabel("Time (s)", fontsize=32)  # Set x-axis label here

    for ax in axs:
        ax.grid(True)
        ax.legend(fontsize=32)
        ax.xaxis.set_tick_params(labelsize=28)
        ax.yaxis.set_tick_params(labelsize=28)

    plt.locator_params(axis='x', nbins=num_xticks)
    plt.locator_params(axis='y', nbins=num_yticks)

    # Define the file path with the provided figure_name appended
    file = '/home/omarg/control_quad_ws/src/nengo_pid/scripts/Results/New_Data/' + figure_name

    plt.savefig(file, format="pdf", bbox_inches='tight')
    plt.clf()
    plt.close()

def plot_3_Spec(t, data_1, data_2, data_3, name_1, units_1, name_2, units_2, name_3, units_3, figure_name, lim1=None, lim2=None, lim3=None):
    fig, axs = plt.subplots(1, 3, figsize=(15, 10))

    axs[0].plot(t, data_1)
    axs[0].set_title(name_1)
    axs[0].set_xlabel("Time (s)", fontsize=32)
    axs[0].set_ylabel(units_1, fontsize=32)
    if lim1 is not None:
        axs[0].set_ylim(lim1[0], lim1[1])

    axs[1].plot(t, data_2)
    axs[1].set_title(name_2)
    axs[1].set_xlabel("Time (s)", fontsize=32)
    axs[1].set_ylabel(units_2, fontsize=32)
    if lim2 is not None:
        axs[1].set_ylim(lim2[0], lim2[1])

    axs[2].plot(t, data_3)
    axs[2].set_title(name_3)
    axs[2].set_xlabel("Time (s)", fontsize=32)
    axs[2].set_ylabel(units_3, fontsize=32)
    if lim3 is not None:
        axs[2].set_ylim(lim3[0], lim3[1])

    axs[0].grid(True), axs[1].grid(True), axs[2].grid(True)
    plt.tight_layout()

    file = '/home/omarg/control_quad_ws/src/nengo_pid/scripts/Results/New_Data/' + figure_name

    plt.savefig(file, format="pdf", bbox_inches='tight')
    plt.clf()
    plt.close()

def plot_4_Spec(t, data_1, data_2, data_3, data_4, name_1, units_1, name_2, units_2, name_3, units_3, name_4, units_4, figure_name, loc, lim1=None, lim2=None, lim3=None, lim4=None, lim5=None , num_xticks = 5, num_yticks = 5):
    fig, axs = plt.subplots(4, 1, figsize=(15, 10))

    if lim5 is not None:
        axs[0].set_xlim(lim5[0], lim5[1])
        axs[1].set_xlim(lim5[0], lim5[1])
        axs[2].set_xlim(lim5[0], lim5[1])
        axs[3].set_xlim(lim5[0], lim5[1])

    axs[0].plot(t, data_1, label = name_1, color = 'red', linewidth = 2)
    #axs[0].set_title(name_1)
    #axs[0].set_xlabel("Time (s)")
    axs[0].set_ylabel(units_1, fontsize=32)
    if lim1 is not None:
        axs[0].set_ylim(lim1[0], lim1[1])

    axs[1].plot(t, data_2, label = name_2, color = 'red', linewidth = 2)
    #axs[1].set_title(name_2)
    #axs[1].set_xlabel("Time (s)", fontsize=32)
    axs[1].set_ylabel(units_2, fontsize=32)
    if lim2 is not None:
        axs[1].set_ylim(lim2[0], lim2[1])

    axs[2].plot(t, data_3, label = name_3, color = 'red', linewidth = 2)
    #axs[2].set_title(name_3)
    #axs[2].set_xlabel("Time (s)", fontsize=32)
    axs[2].set_ylabel(units_3, fontsize=32)
    if lim3 is not None:
        axs[2].set_ylim(lim3[0], lim3[1])

    axs[3].plot(t, data_4, label = name_4, color = 'red', linewidth = 2)
    #axs[3].set_title(name_4)
    axs[3].set_xlabel("Time (s)", fontsize=32)
    axs[3].set_ylabel(units_4, fontsize=32)
    if lim4 is not None:
        axs[3].set_ylim(lim4[0], lim4[1])

    for ax in axs:
        ax.grid(True)
        ax.legend(fontsize=32, loc=loc)
        ax.xaxis.set_tick_params(labelsize=28)
        ax.yaxis.set_tick_params(labelsize=28)

    plt.locator_params(axis='x', nbins=num_xticks)
    plt.locator_params(axis='y', nbins=num_yticks)

    plt.tight_layout()

    file = '/home/omarg/control_quad_ws/src/nengo_pid/scripts/Results/New_Data/' + figure_name

    plt.savefig(file, format="pdf", bbox_inches='tight')
    plt.clf()
    plt.close()

def plot_2in1_3(t, data_1, data_2, data_3, data_4, data_5, data_6, name_1, units_1, name_2, units_2, name_3, units_3, name_4, units_4, name_5, units_5, name_6, units_6, figure_name, lim1=None, lim2=None, lim3=None, lim4=None):
    fig, axs = plt.subplots(1, 3, figsize=(15, 10))

    if lim4 is not None:
        axs[0].set_xlim(lim4[0], lim4[1])
        axs[1].set_xlim(lim4[0], lim4[1])
        axs[2].set_xlim(lim4[0], lim4[1])

    axs[0].plot(t, data_1, label = name_1)
    axs[0].set_xlabel("Time (s)", fontsize=32)
    axs[0].set_ylabel(units_1, fontsize=32)
    if lim1 is not None:
        axs[0].set_ylim(lim1[0], lim1[1])
    axs[0].plot(t, data_2, label = name_2)
    axs[0].set_xlabel("Time (s)", fontsize=32)
    axs[0].set_ylabel(units_2, fontsize=32)

    axs[1].plot(t, data_3, label = name_3)
    axs[1].set_xlabel("Time (s)", fontsize=32)
    axs[1].set_ylabel(units_3, fontsize=32)
    if lim2 is not None:
        axs[1].set_ylim(lim2[0], lim2[1])
    axs[1].plot(t, data_4, label = name_4)
    axs[1].set_xlabel("Time (s)", fontsize=32)
    axs[1].set_ylabel(units_4, fontsize=32)

    axs[2].plot(t, data_5, label = name_5)
    axs[2].set_xlabel("Time (s)", fontsize=32)
    axs[2].set_ylabel(units_5, fontsize=32)
    if lim3 is not None:
        axs[2].set_ylim(lim3[0], lim3[1])
    axs[2].plot(t, data_6, label = name_6)
    axs[2].set_xlabel("Time (s)", fontsize=32)
    axs[2].set_ylabel(units_6, fontsize=32)

    axs[0].grid(True), axs[1].grid(True), axs[2].grid(True)
    axs[0].legend(fontsize=28), axs[1].legend(fontsize=28), axs[2].legend(fontsize=28)
    plt.tight_layout()

    file = '/home/omarg/control_quad_ws/src/nengo_pid/scripts/Results/New_Data/' + figure_name

    plt.savefig(file, format="pdf", bbox_inches='tight')
    plt.clf()
    plt.close()

def plot_3d_trajectory(t, data_x, data_y, data_z, data_xd, data_yd, data_zd, name_x, units_x, name_y, units_y, name_z,
                       units_z, figure_name, loc, lim_x=None, lim_y=None, lim_z=None, num_xticks = 5, num_yticks = 5, num_zticks = 5):
    fig = plt.figure(figsize=(10, 10))
    ax = fig.add_subplot(111, projection='3d')  # Create a 3D subplot

    ax.plot(data_xd, data_yd, data_zd, label="Desired Trajectory", color='red')  # Plot the desired trajectory
    ax.plot(data_x, data_y, data_z, label="Actual Trajectory", color='blue')  # Plot the actual trajectory

    ax.set_xlabel(name_x + ' (' + units_x + ')', fontsize=22, labelpad=20)
    ax.set_ylabel(name_y + ' (' + units_y + ')', fontsize=22, labelpad=20)
    ax.set_zlabel(name_z + ' (' + units_z + ')', fontsize=22, labelpad=20)

    if lim_x is not None:
        ax.set_xlim(lim_x[0], lim_x[1])
    if lim_y is not None:
        ax.set_ylim(lim_y[0], lim_y[1])
    if lim_z is not None:
        ax.set_zlim(lim_z[0], lim_z[1])

    ax.grid(True)
    #ax.legend(fontsize=32)
    ax.xaxis.set_tick_params(labelsize=22)
    ax.yaxis.set_tick_params(labelsize=22)
    ax.zaxis.set_tick_params(labelsize=22)

    plt.locator_params(axis='x', nbins=num_xticks)
    plt.locator_params(axis='y', nbins=num_yticks)
    plt.locator_params(axis='z', nbins=num_zticks)

    #plt.title(figure_name)
    plt.grid(True)
    #plt.legend(fontsize=28, bbox_to_anchor=(0.5, 1.1), loc=loc)

    file = '/home/omarg/control_quad_ws/src/nengo_pid/scripts/Results/New_Data/' + figure_name
    plt.savefig(file, format="pdf", bbox_inches='tight')
    plt.clf()
    plt.close()

def plot_xz_plane(t, data_x, data_z, data_xd, data_zd, name_x, units_x, name_z, units_z, figure_name, loc, lim_x=None,
                  lim_z=None, num_xticks=5, num_yticks=5):
    plt.figure(figsize=(10, 10))

    plt.plot(data_xd, data_zd, label="Desired Trajectory", color='red', linewidth=2)  # Plot the desired trajectory
    plt.plot(data_x, data_z, label="Actual Trajectory", color='blue', linewidth=2)  # Plot the actual trajectory

    plt.xlabel(name_x + ' (' + units_x + ')', fontsize=32)
    plt.ylabel(name_z + ' (' + units_z + ')', fontsize=32)

    plt.locator_params(axis='x', nbins=num_xticks)
    plt.locator_params(axis='y', nbins=num_yticks)

    plt.xticks(fontsize=28)
    plt.yticks(fontsize=28)

    if lim_x is not None:
        plt.xlim(lim_x[0], lim_x[1])
    if lim_z is not None:
        plt.ylim(lim_z[0], lim_z[1])

    plt.legend(fontsize=28, loc=loc)
    #plt.title(figure_name)
    plt.grid(True)

    file = '/home/omarg/control_quad_ws/src/nengo_pid/scripts/Results/New_Data/' + figure_name
    plt.savefig(file, format="pdf", bbox_inches='tight')
    plt.clf()
    plt.close()

def plot_2in1_3_2times(t, t2, data_1, data_2, data_3, data_4, data_5, data_6, name_1, units_1, name_2, units_2, name_3, units_3, name_4, units_4, name_5, units_5, name_6, units_6, figure_name, lim1=None, lim2=None, lim3=None, num_xticks=5, num_yticks=5):
    fig, axs = plt.subplots(1, 3, figsize=(15, 10))

    axs[0].plot(t, data_1, label = name_1, color='blue', linewidth=2)
    axs[0].set_xlabel("Time (s)", fontsize=32)
    axs[0].set_ylabel(units_1, fontsize=32)
    if lim1 is not None:
        axs[0].set_ylim(lim1[0], lim1[1])
    axs[0].plot(t, data_2, label = name_2, color='red', linewidth=2)
    axs[0].set_xlabel("Time (s)", fontsize=32)
    axs[0].set_ylabel(units_2, fontsize=32)

    axs[1].plot(t, data_3, label = name_3, color='blue', linewidth=2)
    axs[1].set_xlabel("Time (s)")
    axs[1].set_ylabel(units_3)
    if lim2 is not None:
        axs[1].set_ylim(lim2[0], lim2[1])
    axs[1].plot(t, data_4, label = name_4, color='red', linewidth=2)
    axs[1].set_xlabel("Time (s)", fontsize=32)
    axs[1].set_ylabel(units_4, fontsize=32)

    axs[2].plot(t2, data_5, label = name_5, color='blue', linewidth=2)
    axs[2].set_xlabel("Time (s)", fontsize=32)
    axs[2].set_ylabel(units_5, fontsize=32)
    if lim3 is not None:
        axs[2].set_ylim(lim3[0], lim3[1])
    axs[2].plot(t2, data_6, label = name_6, color='red', linewidth=2)
    axs[2].set_xlabel("Time (s)", fontsize=32)
    axs[2].set_ylabel(units_6, fontsize=32)

    for ax in axs:
        ax.grid(True)
        ax.legend(fontsize=32)
        ax.xaxis.set_tick_params(labelsize=28)
        ax.yaxis.set_tick_params(labelsize=28)

    plt.locator_params(axis='x', nbins=num_xticks)
    plt.locator_params(axis='y', nbins=num_yticks)
    plt.tight_layout()

    file = '/home/omarg/control_quad_ws/src/nengo_pid/scripts/Results/New_Data/' + figure_name

    plt.savefig(file, format="pdf", bbox_inches='tight')
    plt.clf()
    plt.close()

def plot_2in1_3vertical(t, data_1, data_2, data_3, data_4, data_5, data_6, name_1, units_1, name_2, units_2, name_3, units_3, name_4, units_4, name_5, units_5, name_6, units_6, figure_name, loc, lim1=None, lim2=None, lim3=None, lim4=None, num_xticks=5, num_yticks=5):
    fig, axs = plt.subplots(3, 1, figsize=(15, 10))

    if lim4 is not None:
        axs[0].set_xlim(lim4[0], lim4[1])
        axs[1].set_xlim(lim4[0], lim4[1])
        axs[2].set_xlim(lim4[0], lim4[1])

    axs[0].plot(t, data_1, label = name_1, color='blue', linewidth=2)
    #axs[0].set_xlabel("Time (s)", fontsize=32)
    axs[0].set_ylabel(units_1, fontsize=32)
    if lim1 is not None:
        axs[0].set_ylim(lim1[0], lim1[1])
    axs[0].plot(t, data_2, label = name_2, color='red', linewidth=2)
    #axs[0].set_xlabel("Time (s)", fontsize=32)
    axs[0].set_ylabel(units_2, fontsize=32)

    axs[1].plot(t, data_3, label = name_3, color='blue', linewidth=2)
    #axs[1].set_xlabel("Time (s)", fontsize=32)
    axs[1].set_ylabel(units_3, fontsize=32)
    if lim2 is not None:
        axs[1].set_ylim(lim2[0], lim2[1])
    axs[1].plot(t, data_4, label = name_4, color='red', linewidth=2)
    #axs[1].set_xlabel("Time (s)", fontsize=32)
    axs[1].set_ylabel(units_4, fontsize=32)

    axs[2].plot(t, data_5, label = name_5, color='blue', linewidth=2)
    axs[2].set_xlabel("Time (s)", fontsize=32)
    axs[2].set_ylabel(units_5, fontsize=32)
    if lim3 is not None:
        axs[2].set_ylim(lim3[0], lim3[1])
    axs[2].plot(t, data_6, label = name_6, color='red', linewidth=2)
    axs[2].set_xlabel("Time (s)", fontsize=32)
    axs[2].set_ylabel(units_6, fontsize=32)

    for ax in axs:
        ax.grid(True)
        ax.legend(fontsize=32, loc=loc)
        ax.xaxis.set_tick_params(labelsize=28)
        ax.yaxis.set_tick_params(labelsize=28)

        ax.xaxis.set_major_locator(plt.MaxNLocator(num_xticks))
        ax.yaxis.set_major_locator(plt.MaxNLocator(num_yticks))
    plt.tight_layout()

    file = '/home/omarg/control_quad_ws/src/nengo_pid/scripts/Results/New_Data/' + figure_name

    plt.savefig(file, format="pdf", bbox_inches='tight')
    plt.clf()
    plt.close()

def plot_2in1_3vertical2(t, t2,  data_1, data_2, data_3, data_4, data_5, data_6, name_1, units_1, name_2, units_2, name_3, units_3, name_4, units_4, name_5, units_5, name_6, units_6, figure_name, lim1=None, lim2=None, lim3=None, lim4= None, num_xticks=5, num_yticks=5):
    fig, axs = plt.subplots(3, 1, figsize=(15, 10))

    if lim4 is not None:
        axs[0].set_xlim(lim4[0], lim4[1])
        axs[1].set_xlim(lim4[0], lim4[1])
        axs[2].set_xlim(lim4[0], lim4[1])

    axs[0].plot(t, data_1, label = name_1, color='blue', linewidth=2)
    axs[0].set_xlabel("Time (s)", fontsize=32)
    axs[0].set_ylabel(units_1, fontsize=32)
    if lim1 is not None:
        axs[0].set_ylim(lim1[0], lim1[1])
    axs[0].plot(t, data_2, label = name_2, color='red', linewidth=2)
    axs[0].set_xlabel("Time (s)", fontsize=32)
    axs[0].set_ylabel(units_2, fontsize=32)

    axs[1].plot(t, data_3, label = name_3, color='blue', linewidth=2)
    axs[1].set_xlabel("Time (s)", fontsize=32)
    axs[1].set_ylabel(units_3, fontsize=32)
    if lim2 is not None:
        axs[1].set_ylim(lim2[0], lim2[1])
    axs[1].plot(t, data_4, label = name_4, color='red', linewidth=2)
    axs[1].set_xlabel("Time (s)", fontsize=32)
    axs[1].set_ylabel(units_4, fontsize=32)


    axs[2].plot(t2, data_5, label = name_5, color='blue', linewidth=2)
    axs[2].set_xlabel("Time (s)", fontsize=32)
    axs[2].set_ylabel(units_5, fontsize=32)
    if lim3 is not None:
        axs[2].set_ylim(lim3[0], lim3[1])
    axs[2].plot(t2, data_6, label = name_6, color='red', linewidth=2)
    axs[2].set_xlabel("Time (s)", fontsize=32)
    axs[2].set_ylabel(units_6, fontsize=32)

    for ax in axs:
        ax.grid(True)
        ax.legend(fontsize=32)
        ax.xaxis.set_tick_params(labelsize=28)
        ax.yaxis.set_tick_params(labelsize=28)

    plt.locator_params(axis='x', nbins=num_xticks)
    plt.locator_params(axis='y', nbins=num_yticks)
    plt.tight_layout()

    file = '/home/omarg/control_quad_ws/src/nengo_pid/scripts/Results/New_Data/' + figure_name

    plt.savefig(file, format="pdf", bbox_inches='tight')
    plt.clf()
    plt.close()

def plot_2in1_3verticalDOS(t, data_1, data_2, data_3, name_1, units_1, name_2, units_2, name_3, units_3, figure_name, loc, lim1=None, lim2=None, lim3=None, lim4=None, num_xticks=5, num_yticks=5):
    fig, axs = plt.subplots(3, 1, figsize=(15, 10))

    if lim4 is not None:
        axs[0].set_xlim(lim4[0], lim4[1])
        axs[1].set_xlim(lim4[0], lim4[1])
        axs[2].set_xlim(lim4[0], lim4[1])

    axs[0].plot(t, data_1, label = name_1, color='blue', linewidth=2)
    #axs[0].set_xlabel("Time (s)", fontsize=32)
    axs[0].set_ylabel(units_1, fontsize=32)
    if lim1 is not None:
        axs[0].set_ylim(lim1[0], lim1[1])

    axs[1].plot(t, data_2, label = name_2, color='blue', linewidth=2)
    #axs[1].set_xlabel("Time (s)", fontsize=32)
    axs[1].set_ylabel(units_2, fontsize=32)
    if lim2 is not None:
        axs[1].set_ylim(lim2[0], lim2[1])

    axs[2].plot(t, data_3, label = name_3, color='blue', linewidth=2)
    axs[2].set_xlabel("Time (s)", fontsize=32)
    axs[2].set_ylabel(units_3, fontsize=32)
    if lim3 is not None:
        axs[2].set_ylim(lim3[0], lim3[1])

    for ax in axs:
        ax.grid(True)
        ax.legend(fontsize=32, loc=loc)
        ax.xaxis.set_tick_params(labelsize=28)
        ax.yaxis.set_tick_params(labelsize=28)

        ax.xaxis.set_major_locator(plt.MaxNLocator(num_xticks))
        ax.yaxis.set_major_locator(plt.MaxNLocator(num_yticks))
    plt.tight_layout()

    file = '/home/omarg/control_quad_ws/src/nengo_pid/scripts/Results/New_Data/' + figure_name

    plt.savefig(file, format="pdf", bbox_inches='tight')
    plt.clf()
    plt.close()

def plot_3d_trajectory_ref(t, data_x, data_y, data_z, data_xd, data_yd, data_zd, name_x, units_x, name_y, units_y, name_z,
                       units_z, figure_name, loc, lim_x=None, lim_y=None, lim_z=None, num_xticks = 5, num_yticks = 5):
    fig = plt.figure(figsize=(15, 15))
    ax = fig.add_subplot(111, projection='3d')  # Create a 3D subplot

    ax.plot(data_xd, data_yd, data_zd, label="Desired Trajectory", color='red')  # Plot the desired trajectory
    #ax.plot(data_x, data_y, data_z, label="Actual Trajectory", color='blue')  # Plot the actual trajectory

    ax.set_xlabel(name_x + ' (' + units_x + ')', fontsize=22, labelpad=20)
    ax.set_ylabel(name_y + ' (' + units_y + ')', fontsize=22, labelpad=20)
    ax.set_zlabel(name_z + ' (' + units_z + ')', fontsize=22, labelpad=20)

    if lim_x is not None:
        ax.set_xlim(lim_x[0], lim_x[1])
    if lim_y is not None:
        ax.set_ylim(lim_y[0], lim_y[1])
    if lim_z is not None:
        ax.set_zlim(lim_z[0], lim_z[1])

    ax.grid(True)
    #ax.legend(fontsize=32)
    ax.xaxis.set_tick_params(labelsize=22)
    ax.yaxis.set_tick_params(labelsize=22)
    ax.zaxis.set_tick_params(labelsize=22)

    plt.locator_params(axis='x', nbins=num_xticks)
    plt.locator_params(axis='y', nbins=num_yticks)
    plt.locator_params(axis='z', nbins=num_yticks)

    #plt.title(figure_name)
    plt.grid(True)
    #plt.legend(fontsize=28, bbox_to_anchor=(0.5, 1.1), loc=loc)

    file = '/home/omarg/control_quad_ws/src/nengo_pid/scripts/Results/New_Data/' + figure_name
    plt.savefig(file, format="pdf", bbox_inches='tight')
    plt.clf()
    plt.close()

def plot_2_in_1_ref(t, data_1, data_2, name_1, units_1, name_2, units_2, figure_name, num_xticks = 5, num_yticks = 5):
    fig, ax = plt.subplots(figsize=(10, 7))

    # Plot data_1
    ax.plot(t, data_1, label=name_1, color = 'blue', linewidth = 2)
    ax.set_xlabel("Time (s)", fontsize=32)
    ax.set_ylabel(units_1, fontsize=32)

    ax.grid(True)
    ax.legend(fontsize=32)
    ax.xaxis.set_tick_params(labelsize=28)
    ax.yaxis.set_tick_params(labelsize=28)

    plt.locator_params(axis='x', nbins=num_xticks)
    plt.locator_params(axis='y', nbins=num_yticks)

    plt.tight_layout()

    # Define the file path with the provided figure_name appended
    file = '/home/omarg/control_quad_ws/src/nengo_pid/scripts/Results/New_Data/' + figure_name

    plt.savefig(file, format="pdf", bbox_inches='tight')
    plt.clf()
    plt.close()

