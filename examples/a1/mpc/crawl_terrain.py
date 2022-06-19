import robotoc
import numpy as np
from a1_simulator import A1Simulator


path_to_urdf = '../a1_description/urdf/a1.urdf'
contact_frames = ['FL_foot', 'RL_foot', 'FR_foot', 'RR_foot'] 
contact_types = [robotoc.ContactType.PointContact for i in contact_frames]
baumgarte_time_step = 0.05
robot = robotoc.Robot(path_to_urdf, robotoc.BaseJointType.FloatingBase, 
                      contact_frames, contact_types, baumgarte_time_step)

step_length = np.array([0.25, 0, 0]) 
step_yaw = 0.0

step_height = 0.1
swing_time = 0.25
stance_time = 0.05
swing_start_time = 0.5

vcom_cmd = 0.25 * step_length / (swing_time+stance_time)
yaw_rate_cmd = step_yaw / (swing_time+stance_time)

T = 0.5
N = 18
max_steps = 3
nthreads = 4
mpc = robotoc.MPCCrawl(robot, T, N, max_steps, nthreads)

planner = robotoc.CrawlFootStepPlanner(robot)
planner.set_gait_pattern(step_length, step_yaw, (stance_time > 0.))
# planner.set_raibert_gait_pattern(vcom_cmd, yaw_rate_cmd, swing_time, stance_time, gain=0.7)
mpc.set_gait_pattern(planner, step_height, swing_time, stance_time, swing_start_time)

q = np.array([0, 0, 0.3181, 0, 0, 0, 1, 
              0.0,  0.67, -1.3, 
              0.0,  0.67, -1.3, 
              0.0,  0.67, -1.3, 
              0.0,  0.67, -1.3])
q[0] -= 2.5
v = np.zeros(robot.dimv())
t = 0.0
option_init = robotoc.SolverOptions()
option_init.max_iter = 10

mpc.init(t, q, v, option_init)
option_mpc = robotoc.SolverOptions()
option_mpc.max_iter = 2 # MPC iterations
mpc.set_solver_options(option_mpc)

sim_time_step = 0.0025 # 400 Hz MPC
sim_start_time = 0.0
sim_end_time = 10.0
sim = A1Simulator(path_to_urdf, sim_time_step, sim_start_time, sim_end_time)

log = False
record = False

q0 = q.copy()
q0[2] += 0.04 # to avoid penetration at the initial configuraion
sim.set_camera(3.0, 15, 8, q[0:3]+np.array([0.0, 1.5, 0.2]))
sim.run_simulation(mpc, q0, v, feedback_delay=True, verbose=False, terrain=True, 
                   record=record, log=log, sim_name='a1_crawl_terrain')

if record:
    sim.disconnect()
    robotoc.utils.adjust_video_duration(sim.sim_name+'.mp4', 
                                        desired_duration_sec=(sim_end_time-sim_start_time))

if log:
    q_log = np.genfromtxt(sim.q_log)
    v_log = np.genfromtxt(sim.v_log)
    t_log = np.genfromtxt(sim.t_log)
    sim_steps = t_log.shape[0]

    from scipy.spatial.transform import Rotation
    vcom_log = []
    wcom_log = []
    vcom_cmd_log = []
    yaw_rate_cmd_log = []
    for i in range(sim_steps):
        R = Rotation.from_quat(q_log[i][3:7]).as_matrix()
        robot.forward_kinematics(q_log[i], v_log[i])
        vcom_log.append(R.T@robot.com_velocity()) # robot.com_velocity() is expressed in the world coordinate
        wcom_log.append(v_log[i][3:6])
        vcom_cmd_log.append(vcom_cmd)
        yaw_rate_cmd_log.append(yaw_rate_cmd)

    plot_mpc = robotoc.utils.PlotCoMVelocity()
    plot_mpc.plot(t_log, vcom_log, wcom_log, vcom_cmd_log, yaw_rate_cmd_log, 
                  fig_name=sim.sim_name+'_com_vel')