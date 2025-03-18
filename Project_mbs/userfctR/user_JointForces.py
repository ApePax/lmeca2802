# -*- coding: utf-8 -*-
"""Module for the definition of joint forces."""
# Author: Robotran Team
# (c) Universite catholique de Louvain, 2020
import numpy as np

def user_JointForces(mbs_data, tsim):
    """Compute the force and torques in the joint.

    It fills the MBsysPy.MbsData.Qq array.

    Parameters
    ----------
    mbs_data : MBsysPy.MbsData
        The multibody system associated to this computation.
    tsim : float
        The current time of the simulation.

    Notes
    -----
    The numpy.ndarray MBsysPy.MbsData.Qq is 1D array with index starting at 1.
    The first index (array[0]) must not be modified. The first index to be
    filled is array[1].

    Returns
    -------
    None
    """
    
    mbs_data.Qq[1:] = 0.
    motor_ids = [mbs_data.joint_id["R_lh"],mbs_data.joint_id["R_rh"] ,mbs_data.joint_id["R_lk"],mbs_data.joint_id["R_rk"]]
    q_motors  = [mbs_data.q[motor_ids[0]] , mbs_data.q[motor_ids[1]] ,mbs_data.q[motor_ids[2]] ,mbs_data.q[motor_ids[3]]]
    qd_motors = [mbs_data.qd[motor_ids[0]], mbs_data.qd[motor_ids[1]],mbs_data.qd[motor_ids[2]],mbs_data.qd[motor_ids[3]]]


    trajectory_index = int(tsim // mbs_data.frequency)
    q_ref = mbs_data.reference_trajectory[trajectory_index][1:]
    
    ### Dynamixel Controller ###
    Kp = 900.0/128
    PWM_goal = 885.0
    Nominal_voltage = 12.0
    
    PWM = (q_ref - q_motors) * (4095.0 / (2 * np.pi) * Kp)
    PWM_sat = np.clip(PWM, PWM_goal, PWM_goal)
    
    u = PWM_sat * (Nominal_voltage / PWM_goal)   
    
    ### Motor Equations ###

    return
