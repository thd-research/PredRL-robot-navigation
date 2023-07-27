#!/usr/bin/env python3

import os, sys
PARENT_DIR = os.path.abspath(__file__ + '/../..')
sys.path.insert(0, PARENT_DIR)

import pathlib
    
import warnings
import csv
from datetime import datetime
import matplotlib.animation as animation
import matplotlib.pyplot as plt
import numpy as np

from rcognita import systems
from rcognita import controllers
from rcognita import loggers


import argparse
import rospy
from std_srvs.srv import Empty
import threading

import os

import tf.transformations as tftr
from geometry_msgs.msg import Twist
from math import atan2, pi, radians, cos, sin
import math
from numpy import cos, pi, sin, cos
import time as time_lib

##############################################
import main_3wrobot_ros_obst as ROS_preset_init

def main():

    dttt = 60
    #radius = 5
    #j = 0

    coordinate_list = []
    


    dim_state = 3
    dim_input = 2
    dim_output = dim_state
    dim_disturb = 2
    dim_R1 = dim_output + dim_input
    dim_R2 = dim_R1

    

#for j in range(dttt):
#    print("J = ", j)
    
    rospy.init_node('ROS_preset_node')
    
    rospy.wait_for_service('/gazebo/reset_world')
    reset_world = rospy.ServiceProxy('/gazebo/reset_world', Empty)
    reset_world()
    
    
    description = "Agent-environment preset: a 3-wheel robot (kinematic model a. k. a. non-holonomic integrator)."
    parser = argparse.ArgumentParser(description=description)
    parser.add_argument('--ctrl_mode', metavar='ctrl_mode', type=str,
                        choices=['manual',
                                'nominal',
                                'MPC',
                                'RQL',
                                'SQL',
                                'JACS'],
                        default='nominal',
                        help='Control mode. Currently available: ' +
                        '----manual: manual constant control specified by action_manual; ' +
                        '----nominal: nominal controller, usually used to benchmark optimal controllers;' +                     
                        '----MPC:model-predictive control; ' +
                        '----RQL: Q-learning actor-critic with Nactor-1 roll-outs of stage objective; ' +
                        '----SQL: stacked Q-learning; ' + 
                        '----JACS: joint actor-critic (stabilizing), system-specific, needs proper setup.')
    parser.add_argument('--j', type=int, metavar='j',
                        default=0,
                        help='For autotests' )
    parser.add_argument('--set_radius', type=int, metavar='set_radius',
                        default=1,
                        help='Set raduis' )
    parser.add_argument('--dt', type=float, metavar='dt',
                        default=0.01,
                        help='Controller sampling time.' )
    parser.add_argument('--t1', type=float, metavar='t1',
                        default=150.0,
                        help='Final time of episode.' )
    parser.add_argument('--is_log_data', type=bool,
                        default=True,
                        help='Flag to log data into a data file. Data are stored in simdata folder.')
    parser.add_argument('--is_visualization', type=bool,
                        default=True,
                        help='Flag to produce graphical output.')
    parser.add_argument('--is_print_sim_step', type=bool,
                        default=False,
                        help='Flag to print simulation data into terminal.')
    parser.add_argument('--is_est_model', type=bool,
                        default=False,
                        help='Flag to estimate environment model.')
    parser.add_argument('--model_est_stage', type=float,
                        default=2.0,
                        help='Seconds to learn model until benchmarking controller kicks in.')
    parser.add_argument('--model_est_period_multiplier', type=float,
                        default=1,
                        help='Model is updated every model_est_period_multiplier times dt seconds.')
    parser.add_argument('--model_order', type=int,
                        default=5,
                        help='Order of state-space estimation model.')
    parser.add_argument('--prob_noise_pow', type=float,
                        default=8,
                        help='Power of probing (exploration) noise.')
    parser.add_argument('--action_manual', type=float,
                        default=[0.22, 0.0], nargs='+',
                        help='Manual control action to be fed constant, system-specific!')
    parser.add_argument('--Nactor', type=int,
                        default=3,
                        help='Horizon length (in steps) for predictive controllers.')
    parser.add_argument('--pred_step_size_multiplier', type=float,
                        default=6.0,
                        help='Size of each prediction step in seconds is a pred_step_size_multiplier multiple of controller sampling time dt.')
    parser.add_argument('--buffer_size', type=int,
                        default=200,
                        help='Size of the buffer (experience replay) for model estimation, agent learning etc.')
    parser.add_argument('--stage_obj_struct', type=str,
                        default='quadratic',
                        choices=['quadratic',
                                'biquadratic'],
                        help='Structure of stage objective function.')
    parser.add_argument('--R1_diag', type=float, nargs='+',
                        default=[2, 10, 1, 0, 0],
                        help='Parameter of stage objective function. Must have proper dimension. ' +
                        'Say, if chi = [observation, action], then a quadratic stage objective reads chi.T diag(R1) chi, where diag() is transformation of a vector to a diagonal matrix.')
    parser.add_argument('--R2_diag', type=float, nargs='+',
                        default=[[10, 2, 1, 0, 0], [0, 10, 2, 0, 0], [0, 0, 10, 0, 0], [0, 0, 0, 0, 0], [0, 0, 0, 0, 0]],
                        help='Parameter of stage objective function . Must have proper dimension. ' + 
                        'Say, if chi = [observation, action], then a bi-quadratic stage objective reads chi**2.T diag(R2) chi**2 + chi.T diag(R1) chi, ' +
                        'where diag() is transformation of a vector to a diagonal matrix.')
    parser.add_argument('--Ncritic', type=int,
                        default=50,
                        help='Critic stack size (number of temporal difference terms in critic cost).')
    parser.add_argument('--gamma', type=float,
                        default=1.0,
                        help='Discount factor.')
    parser.add_argument('--critic_period_multiplier', type=float,
                        default=5.0,
                        help='Critic is updated every critic_period_multiplier times dt seconds.')
    parser.add_argument('--critic_struct', type=str,
                        default='quad-nomix', choices=['quad-lin',
                                                    'quadratic',
                                                    'quad-nomix',
                                                    'quad-mix'],
                        help='Feature structure (critic). Currently available: ' +
                        '----quad-lin: quadratic-linear; ' +
                        '----quadratic: quadratic; ' +
                        '----quad-nomix: quadratic, no mixed terms; ' +
                        '----quad-mix: quadratic, mixed observation-action terms (for, say, Q or advantage function approximations).')
    parser.add_argument('--actor_struct', type=str,
                        default='quad-nomix', choices=['quad-lin',
                                                    'quadratic',
                                                    'quad-nomix'],
                        help='Feature structure (actor). Currently available: ' +
                        '----quad-lin: quadratic-linear; ' +
                        '----quadratic: quadratic; ' +
                        '----quad-nomix: quadratic, no mixed terms.')
    parser.add_argument('--state_init', type=str, nargs="+", metavar='state_init',
                        default=['0', '0', 'pi'],
                        help='Initial state (as sequence of numbers); ' + 
                        'dimension is environment-specific!')
    args = parser.parse_args()
    #----------------------------------------Set up dimensions
    

    for i in range(dttt):
        angle = i * (2*pi/dttt)
        coordinate_x = args.set_radius * cos(angle)
        coordinate_y = args.set_radius * sin(angle)

        coordinate_list_1 = [coordinate_x, coordinate_y, angle]
        coordinate_list.append(coordinate_list_1)



    j = int(sys.argv[12])

    #----------------------------------------Post-processing of arguments
    # Convert `pi` to a number pi
    args.state_init = np.array([coordinate_list[args.j][0], coordinate_list[args.j][1], 2*pi/dttt*j])
    #for k in range(len(args.state_init)):
    #    args.state_init[k] = eval( args.state_init[k].replace('pi', str(np.pi)) )
    #args.state_init = np.array(args.state_init)
    
    #args.state_init = np.array([coordinate_list[j][0], coordinate_list[j][1], coordinate_list[j][2]])

    #print("STATE INIT:", args.state_init)
    #time_lib.sleep(5)
    args.action_manual = np.array(args.action_manual)
    pred_step_size = args.dt * args.pred_step_size_multiplier
    model_est_period = args.dt * args.model_est_period_multiplier
    critic_period = args.dt * args.critic_period_multiplier
    R1 = np.diag(np.array(args.R1_diag))
    R2 = np.diag(np.array(args.R2_diag))
    assert args.t1 > args.dt > 0.0
    assert args.state_init.size == dim_state
    globals().update(vars(args))
    #----------------------------------------(So far) fixed settings
    is_disturb = 0
    # Disturbance
    sigma_q = 1e-3 * np.ones(dim_disturb)
    mu_q = np.zeros(dim_disturb)
    tau_q = np.ones(dim_disturb)
    is_dyn_ctrl = 0
    x0 = np.zeros(dim_state)
    t0 = 0
    Nruns = 1
    x0 = np.zeros(dim_state)
    action_init = np.zeros(dim_input)
    q0 = np.zeros(dim_disturb)
    # Solver
    atol = 1e-5
    rtol = 1e-3
    # xy-plane
    xMin = -10
    xMax = 10
    yMin = -10
    yMax = 10
    # Model estimator stores models in a stack and recall the best of model_est_checks
    model_est_checks = 0
    # Control constraints
    v_min = -0.22
    v_max = 0.22
    omega_min = -2
    omega_max = 2
    ctrl_bnds=np.array([[v_min, v_max], [omega_min, omega_max]])
    state_goal = state_init.copy()
    #----------------------------------------Initialization : : system
    my_sys = systems.Sys3WRobotNI(sys_type="diff_eqn",
                                        dim_state=dim_state,
                                        dim_input=dim_input,
                                        dim_output=dim_output,
                                        dim_disturb=dim_disturb,
                                        pars=[],
                                        ctrl_bnds=ctrl_bnds,
                                        is_dyn_ctrl=is_dyn_ctrl,
                                        is_disturb=is_disturb,
                                        pars_disturb=[sigma_q, mu_q, tau_q])
    observation_init = my_sys.out(state_init)
    #----------------------------------------Initialization : : model
    #----------------------------------------Initialization : : controller
    my_ctrl_nominal = controllers.CtrlNominal3WRobotNI(ctrl_gain=0.02, ctrl_bnds=ctrl_bnds, t0=t0, sampling_time=dt)
    # Predictive optimal controller
    my_ctrl_opt_pred = controllers.CtrlOptPred(dim_input,
                                            dim_output,
                                            ctrl_mode,
                                            ctrl_bnds = ctrl_bnds,
                                            action_init = [],
                                            t0 = t0,
                                            sampling_time = dt,
                                            Nactor = Nactor,
                                            pred_step_size = pred_step_size,
                                            sys_rhs = my_sys._state_dyn,
                                            sys_out = my_sys.out,
                                            state_sys = x0,
                                            prob_noise_pow = prob_noise_pow,
                                            is_est_model = is_est_model,
                                            model_est_stage = model_est_stage,
                                            model_est_period = model_est_period,
                                            buffer_size = buffer_size,
                                            model_order = model_order,
                                            model_est_checks = model_est_checks,
                                            gamma = gamma,
                                            Ncritic = Ncritic,
                                            critic_period = critic_period,
                                            critic_struct = critic_struct,
                                            stage_obj_struct = stage_obj_struct,
                                            stage_obj_pars = [R1],
                                            observation_target = [])
    # Stabilizing RL agent
    my_ctrl_RL_stab = controllers.CtrlRLStab(dim_input,
                                            dim_output,
                                            ctrl_mode,
                                            ctrl_bnds = ctrl_bnds,
                                            action_init = action_init,
                                            t0 = t0,
                                            sampling_time = dt,
                                            Nactor = Nactor,
                                            pred_step_size = pred_step_size,
                                            sys_rhs = my_sys._state_dyn,
                                            sys_out = my_sys.out,
                                            state_sys = state_init,
                                            prob_noise_pow = prob_noise_pow,
                                            is_est_model = is_est_model,
                                            model_est_stage = model_est_stage,
                                            model_est_period = model_est_period,
                                            buffer_size = buffer_size,
                                            model_order = model_order,
                                            model_est_checks = model_est_checks,
                                            gamma = gamma,
                                            Ncritic = Ncritic,
                                            critic_period = critic_period,
                                            critic_struct = critic_struct,
                                            actor_struct = actor_struct,
                                            stage_obj_struct = stage_obj_struct,
                                            stage_obj_pars = [R1],
                                            observation_target = [],
                                            safe_ctrl = my_ctrl_nominal,
                                            safe_decay_rate = 1e-4)
    if ctrl_mode == 'JACS':
        my_ctrl_benchm = my_ctrl_RL_stab
    else:
        my_ctrl_benchm = my_ctrl_opt_pred
    #----------------------------------------Initialization : : logger
    if os.path.basename( os.path.normpath( os.path.abspath(os.getcwd()) ) ) == 'presets':
        data_folder = '../simdata'
    else:
        data_folder = 'simdata'
    pathlib.Path(data_folder).mkdir(parents=True, exist_ok=True) 
    date = datetime.now().strftime("%Y-%m-%d")
    time = datetime.now().strftime("%H:%M:%S")
    datafiles = [None] * Nruns
    for k in range(0, Nruns):
        datafiles[k] = data_folder + '/' + 'ROS__' + my_sys.name + '__' + ctrl_mode + '__' + date + '__' + time + '__run{run:02d}.csv'.format(run=k+1)
        
        if is_log_data:
            print('Logging data to:    ' + datafiles[k])
                
            with open(datafiles[k], 'w', newline='') as outfile:
                writer = csv.writer(outfile)
                writer.writerow(['System', my_sys.name ] )
                writer.writerow(['Controller', ctrl_mode ] )
                writer.writerow(['dt', str(dt) ] )
                writer.writerow(['state_init', str(state_init) ] )
                writer.writerow(['is_est_model', str(is_est_model) ] )
                writer.writerow(['model_est_stage', str(model_est_stage) ] )
                writer.writerow(['model_est_period_multiplier', str(model_est_period_multiplier) ] )
                writer.writerow(['model_order', str(model_order) ] )
                writer.writerow(['prob_noise_pow', str(prob_noise_pow) ] )
                writer.writerow(['Nactor', str(Nactor) ] )
                writer.writerow(['pred_step_size_multiplier', str(pred_step_size_multiplier) ] )
                writer.writerow(['buffer_size', str(buffer_size) ] )
                writer.writerow(['stage_obj_struct', str(stage_obj_struct) ] )
                writer.writerow(['R1_diag', str(R1_diag) ] )
                writer.writerow(['R2_diag', str(R2_diag) ] )
                writer.writerow(['Ncritic', str(Ncritic) ] )
                writer.writerow(['gamma', str(gamma) ] )
                writer.writerow(['critic_period_multiplier', str(critic_period_multiplier) ] )
                writer.writerow(['critic_struct', str(critic_struct) ] )
                writer.writerow(['actor_struct', str(actor_struct) ] )   
                writer.writerow(['t [s]', 'x [m]', 'y [m]', 'alpha [rad]', 'stage_obj', 'accum_obj', 'v [m/s]', 'omega [rad/s]'] )
    # Do not display annoying warnings when print is on
    if is_print_sim_step:
        warnings.filterwarnings('ignore')
        
    my_logger = loggers.Logger3WRobotNI()
    
    
    ros_preset_task = ROS_preset_init.ROS_preset(ctrl_mode, [0, 0, 0], state_goal, my_ctrl_nominal, my_sys, my_ctrl_benchm, action_manual, my_logger, datafiles)
    ros_preset_task.spin(action_manual, is_print_sim_step=is_print_sim_step, is_log_data=is_log_data)
    #preset.shutdown()
    #rospy.on_shutdown(clean_up)
    sys.exit()
    
    #rospy.signal_shutdown("Shutdown")

main()