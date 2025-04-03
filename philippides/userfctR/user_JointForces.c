/* ---------------------------
 * Robotran - MBsysC
 * 
 * Template file for the user JointForces function
 * 
 * (c) Universite catholique de Louvain
 *     
 */

#include "math.h" 

#include "mbs_data.h"
#include "user_model.h"
#include "set_output.h"
#include "useful_functions.h"

typedef struct {
    double **trajectory;  // 2D array storing the trajectory data
    int index;           // Index for tracking progress in the trajectory
} Trajectory;

// Function to initialize trajectory data by reading from a CSV file
Trajectory* init_trajectory(const char *filename, int rows, int cols) {
    Trajectory *traj = (Trajectory*)malloc(sizeof(Trajectory)); // Allocate memory for trajectory structure
    traj->trajectory = (double**)malloc(rows * sizeof(double*)); // Allocate memory for rows
    for (int i = 0; i < rows; i++) {
        traj->trajectory[i] = (double*)malloc(cols * sizeof(double)); // Allocate memory for columns
    }
    traj->index = 0;
    
    FILE *file = fopen(filename, "r"); // Open the file for reading
    if (!file) {
        perror("Error opening file");
        return NULL;
    }
    
    char buffer[1024];
    fgets(buffer, sizeof(buffer), file); // Skip the header line
    
    for (int i = 0; i < rows; i++) {
        for (int j = 0; j < cols; j++) {
            fscanf(file, "%lf,", &traj->trajectory[i][j]); // Read values from file
        }
    }
    fclose(file); // Close the file
    return traj;
}

// Function to compute joint forces based on motor states and reference trajectory
double* user_JointForces(MbsData *mbs_data, double tsim) {
    // Reset all forces in the joint force array
    for (int i = 1; i < MAX_JOINTS; i++) {
        mbs_data->Qq[i] = 0.0;
    }
    
    // Retrieve motor IDs for different joints
    int motor_ids[4] = {mbs_data->joint_id["R_lh"], mbs_data->joint_id["R_rh"],
                         mbs_data->joint_id["R_lk"], mbs_data->joint_id["R_rk"]};
    
    // Retrieve motor positions and velocities
    double q_motors[4], qd_motors[4];
    for (int i = 0; i < 4; i++) {
        q_motors[i] = mbs_data->q[motor_ids[i]];
        qd_motors[i] = mbs_data->qd[motor_ids[i]];
    }
    
    // Compute trajectory index based on simulation time
    int trajectory_index = (int)(tsim * mbs_data->frequency);
    if (trajectory_index >= TOTAL_TRAJECTORY_ROWS) {
        trajectory_index = TOTAL_TRAJECTORY_ROWS - 1; // Prevent out-of-bounds access
    }
    
    // Get reference joint positions for the current time step
    double q_ref[4];
    for (int i = 0; i < 4; i++) {
        q_ref[i] = reference_trajectory->trajectory[trajectory_index][i+1];
    }
    
    // Controller parameters
    double Kp = 900.0 / 128.0;
    double PWM_goal = 885.0;
    double Nominal_voltage = 12.0;
    
    // Compute PWM signals for motor control
    double PWM[4], PWM_sat[4], u[4];
    for (int i = 0; i < 4; i++) {
        PWM[i] = (q_ref[i] - q_motors[i]) * (4095.0 / (2 * M_PI) * Kp);
        PWM_sat[i] = fmax(-PWM_goal, fmin(PWM[i], PWM_goal)); // Clipping function
        u[i] = PWM_sat[i] * (Nominal_voltage / PWM_goal);
    }
    
    // Store computed voltages for motors
    for (int i = 0; i < 4; i++) {
        mbs_data->Voltages[i] = u[i];
    }
    
    // Motor parameters
    double HGR = 353.5;   // Hip gear ratio
    double KGR = 212.6;   // Knee gear ratio
    double ktp = 0.395 / HGR;   // Torque constant w.r.t. voltage [Nm/V]
    double Kvp = 1.589 / (HGR * HGR); // Viscous friction constant [Nm*s/rad]
    double τc_u = 0.065 / HGR;  // Dry friction torque [Nm]
    
    // Compute motor torques
    double gear_ratios[4] = {HGR, HGR, KGR, KGR};
    double w[4], τ_0[4], τ_m[4];
    for (int i = 0; i < 4; i++) {
        w[i] = qd_motors[i] * gear_ratios[i];
        τ_0[i] = u[i] * gear_ratios[i] * ktp - w[i] * gear_ratios[i] * Kvp;
        τ_m[i] = τ_0[i] - ((w[i] > 0) ? τc_u * gear_ratios[i] : -τc_u * gear_ratios[i]);
    }
    
    // Assign computed torques to joint force array
    for (int i = 0; i < 4; i++) {
        mbs_data->Qq[motor_ids[i]] = τ_m[i];
    }
    
    return mbs_data->Qq;
}
