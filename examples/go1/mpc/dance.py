import robotoc
from robotoc_sim import MPCSimulation, CameraSettings
from go1_simulator import A1Simulator
import numpy as np

model_info = robotoc.RobotModelInfo()
model_info.urdf_path = '../go1_description/urdf/go1.urdf'
model_info.base_joint_type = robotoc.BaseJointType.FloatingBase
baumgarte_time_step = 0.05
model_info.point_contacts = [robotoc.ContactModelInfo('FL_foot', baumgarte_time_step),
                             robotoc.ContactModelInfo('RL_foot', baumgarte_time_step),
                             robotoc.ContactModelInfo('FR_foot', baumgarte_time_step),
                             robotoc.ContactModelInfo('RR_foot', baumgarte_time_step)]
robot = robotoc.Robot(model_info)
t0 = 0.0

q0 = np.array([0, 0, 0.3500, 0, 0, 0, 1,
               0.0,  0.67, -1.3, 
               0.0,  0.67, -1.3, 
               0.0,  0.67, -1.3, 
               0.0,  0.67, -1.3]) #inital positions
               
robot.forward_kinematics(q0)
com_ref0 = robot.com()
x3d0_LF = robot.frame_position('FL_foot')
x3d0_LH = robot.frame_position('RL_foot')
x3d0_RF = robot.frame_position('FR_foot')
x3d0_RH = robot.frame_position('RR_foot')


v0 = np.zeros(robot.dimv())

q_array = np.load('config.npy')
q_array = np.transpose(q_array)

LF_ee_array = np.load('fl_ee.npy')
RF_ee_array = np.load('fr_ee.npy')
LH_ee_array = np.load('rl_ee.npy')
RH_ee_array = np.load('rr_ee.npy')

LF_inMotion= np.load('fl_inMotion.npy')
RF_inMotion = np.load('fr_inMotion.npy')
LH_inMotion = np.load('rl_inMotion.npy')
RH_inMotion = np.load('rr_inMotion.npy')


step_length = np.array([0, 0, 0]) 
v_com = np.array([0,0,0])

step_height = 0.1
swing_time = 1
contact_time = 1.4
step_yaw = 0

CoM_t0 = 0.0
LF_t0 = 0.0
LH_t0 = 0.0
RF_t0 = 0.0
RH_t0 = 0.0

size = 100

discrete_event_interval = 0
swing_start_time = 0.1

T = 0.60
N = 20
mpc = robotoc.MPCDance(robot, T, N, q0, x3d0_LF,x3d0_LH,x3d0_RF,x3d0_RH)
planner = robotoc.DanceFootStepPlanner(robot)
planner.set_gait_pattern(step_length, step_yaw, (contact_time>0.))
mpc.set_gait_pattern(planner, CoM_t0, LF_t0, LH_t0, RF_t0, RH_t0, q_array, 
                     LF_ee_array,LH_ee_array,RF_ee_array,RH_ee_array,
                     LF_inMotion,LH_inMotion,RF_inMotion, RH_inMotion,size)

option_init = robotoc.SolverOptions()
option_init.max_iter = 10
option_init.nthreads = 4
mpc.init(t0, q0, v0, option_init)

option_mpc = robotoc.SolverOptions()
option_mpc.max_iter = 2
option_mpc.nthreads = 10
mpc.set_solver_options(option_mpc)

time_step = 0.004 # 400 Hz MPC
a1_simulator = A1Simulator(urdf_path=model_info.urdf_path, time_step=time_step)
camera_settings = CameraSettings(camera_distance=1.2, camera_yaw=45, camera_pitch=-10.0, 
                                 camera_target_pos=q0[0:3]+np.array([0.1, 0.5, 0.0]))
a1_simulator.set_camera_settings(camera_settings=camera_settings)

simulation_time = 4.2
log = False
record = False
simulation = MPCSimulation(simulator=a1_simulator)
simulation.run(mpc=mpc, t0=t0, q0=q0, simulation_time=simulation_time, 
               feedback_delay=False, verbose=False, 
               record=record, log=log, name='a1_dance')

if record:
    robotoc.utils.adjust_video_duration(simulation.name+'.mp4', 
                                        desired_duration_sec=simulation_time)

if log:
    q_log = np.genfromtxt(simulation.q_log)
    v_log = np.genfromtxt(simulation.v_log)
    t_log = np.genfromtxt(simulation.t_log)
    sim_steps = t_log.shape[0]

    vcom_log = []
    wcom_log = []
    vcom_cmd_log = []
    yaw_rate_cmd_log = []
    for i in range(sim_steps):
        R = robotoc.utils.rotation_matrix_from_quaternion(q_log[i][3:7])
        robot.forward_kinematics(q_log[i], v_log[i])
        vcom_log.append(R.T@robot.com_velocity()) # robot.com_velocity() is expressed in the world coordinate
        wcom_log.append(v_log[i][3:6])
        # vcom_cmd_log.append(vcom_cmd)
        # yaw_rate_cmd_log.append(yaw_rate_cmd)

    plot_mpc = robotoc.utils.PlotCoMVelocity()
    plot_mpc.plot(t_log, vcom_log, wcom_log, vcom_cmd_log, yaw_rate_cmd_log, 
                  fig_name=simulation.name+'_com_vel')
