import numpy as np
import matplotlib.pyplot as plt
#----------------------------------------
# CHOOSE GRAPHS TO PLOS
#----------------------------------------
POSITIONS = True
TORQUES = True
VOLTAGES = True
GROUND_FORCES = True
def joint_plot(columns,filename):
    data = np.loadtxt(filename)
    
    # Extract relevant columns
    time = data[:, columns['Time']]
    R_lh = data[:, columns['R_lh']]
    R_rh = data[:, columns['R_rh']]
    R_lk = data[:, columns['R_lk']]
    R_rk = data[:, columns['R_rk']]
    
    # Create a figure with 4 subplots
    fig, axes = plt.subplots(2, 2, figsize=(10, 8))
    
    # Plot each variable
    axes[0, 0].plot(time, R_lh, label='R_lh')
    axes[0, 0].set_title("R_lh vs Time")
    axes[0, 0].set_xlabel("Time (s)")
    axes[0, 0].set_ylabel("R_lh")
    axes[0, 0].grid(True)
    
    axes[0, 1].plot(time, R_rh, label='R_rh', color='r')
    axes[0, 1].set_title("R_rh vs Time")
    axes[0, 1].set_xlabel("Time (s)")
    axes[0, 1].set_ylabel("R_rh")
    axes[0, 1].grid(True)
    
    axes[1, 0].plot(time, R_lk, label='R_lk', color='g')
    axes[1, 0].set_title("R_lk vs Time")
    axes[1, 0].set_xlabel("Time (s)")
    axes[1, 0].set_ylabel("R_lk")
    axes[1, 0].grid(True)
    
    axes[1, 1].plot(time, R_rk, label='R_rk', color='m')
    axes[1, 1].set_title("R_rk vs Time")
    axes[1, 1].set_xlabel("Time (s)")
    axes[1, 1].set_ylabel("R_rk")
    axes[1, 1].grid(True)
    
    # Adjust layout and show plot
    plt.tight_layout()
    plt.show()
    return


columns = {'Time': 0, 'Tx_boom': 1, 'Tz_boom': 2, 'R_hip': 3, 'T_lh_fixed': 4, 
           'R_lh': 5, 'T_lk_fixed': 6, 'R_lk': 7, 'R_lf': 8, 'T_rh_fixed': 9, 
           'R_rh': 10, 'T_rk_fixed': 11, 'R_rk': 12, 'R_rf': 13}
if POSITIONS:
    filename = "../resultsR/dirdyn_q.res"
    joint_plot(columns, filename)
    
if TORQUES:
    filename = "../resultsR/dirdyn_Qq.res"
    joint_plot(columns, filename)
    
if VOLTAGES:
    # Define column indices
    columns = {'Time': 0, 'R_lh': 1, 'R_rh': 2, 'R_lk': 3, 'R_rk': 4}
    filename = "../resultsR/dirdyn_Voltages.res"
    joint_plot(columns, filename)

if GROUND_FORCES:
    # Define force indices
    force_indices = {
        'F_AvD_L': 1, 'F_AvG_L': 2, 'F_Mid_L': 3, 'F_ArD_L': 4, 'F_ArG_L': 5, 
        'F_ArG_R': 6, 'F_ArD_R': 7, 'F_Mid_R': 8, 'F_AvD_R': 9, 'F_AvG_R': 10
    }
    
    # Load data
    file_path = "../resultsR/dirdyn_Force_Sensors.res"  # Change this to your actual file path
    data = np.loadtxt(file_path)
    
    # Extract time
    time = data[:, 0]
    
    # Extract force components
    Fx = data[:, 2::4]  # Every 4th column starting from index 1
    Fy = data[:, 3::4]  # Every 4th column starting from index 2
    Fz = data[:, 4::4]  # Every 4th column starting from index 3
    
    # Sum forces for left and right
    left_indices = [force_indices[key] - 1 for key in force_indices if key.endswith('_L')]
    right_indices = [force_indices[key] - 1 for key in force_indices if key.endswith('_R')]
    print(left_indices)
    print(right_indices)
    Fx_L, Fx_R = np.sum(Fx[:, left_indices], axis=1), np.sum(Fx[:, right_indices], axis=1)
    Fy_L, Fy_R = np.sum(Fy[:, left_indices], axis=1), np.sum(Fy[:, right_indices], axis=1)
    Fz_L, Fz_R = np.sum(Fz[:, left_indices], axis=1), np.sum(Fz[:, right_indices], axis=1)
    
    # Plot function
    def plot_forces(time, force_L, force_R, ylabel, title):
        fig, axes = plt.subplots(2, 1, figsize=(10, 6), sharex=True)
        
        axes[0].plot(time, force_L, label="Left", color='b')
        axes[0].set_ylabel(ylabel)
        axes[0].set_title(f"{title} - Left Side")
        axes[0].grid()
        
        axes[1].plot(time, force_R, label="Right", color='r')
        axes[1].set_ylabel(ylabel)
        axes[1].set_title(f"{title} - Right Side")
        axes[1].set_xlabel("Time (s)")
        axes[1].grid()
        
        plt.tight_layout()
        plt.show()
    
    # Plot figures
    plot_forces(time, Fx_L, Fx_R, "Force Fx (N)", "Sum of Fx Contributions")
    plot_forces(time, Fy_L, Fy_R, "Force Fy (N)", "Sum of Fy Contributions")
    plot_forces(time, Fz_L, Fz_R, "Force Fz (N)", "Sum of Fz Contributions")
