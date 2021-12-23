#include <string>
#include <memory>

#include "Eigen/Core"

#include "robotoc/solver/ocp_solver.hpp"
#include "robotoc/ocp/ocp.hpp"
#include "robotoc/robot/robot.hpp"
#include "robotoc/hybrid/contact_sequence.hpp"
#include "robotoc/cost/cost_function.hpp"
#include "robotoc/cost/configuration_space_cost.hpp"
#include "robotoc/cost/time_varying_configuration_space_cost.hpp"
#include "robotoc/constraints/constraints.hpp"
#include "robotoc/constraints/joint_position_lower_limit.hpp"
#include "robotoc/constraints/joint_position_upper_limit.hpp"
#include "robotoc/constraints/joint_velocity_lower_limit.hpp"
#include "robotoc/constraints/joint_velocity_upper_limit.hpp"
#include "robotoc/constraints/joint_torques_lower_limit.hpp"
#include "robotoc/constraints/joint_torques_upper_limit.hpp"
#include "robotoc/constraints/friction_cone.hpp"
#include "robotoc/constraints/impulse_friction_cone.hpp"
#include "robotoc/solver/solver_options.hpp"

#include "robotoc/utils/ocp_benchmarker.hpp"

#ifdef ENABLE_VIEWER
#include "robotoc/utils/trajectory_viewer.hpp"
#endif 

 
class TimeVaryingConfigurationRef final : public robotoc::TimeVaryingConfigurationRefBase {
public:
  TimeVaryingConfigurationRef(const double t0, 
                              const double period_init1, 
                              const double period_init2, 
                              const double period,
                              const double period_final, 
                              const int steps,
                              const Eigen::VectorXd& q0, const double v_ref) 
    : TimeVaryingConfigurationRefBase(),
      t0_(t0),
      period_init1_(period_init1),
      period_init2_(period_init2),
      period_(period),
      period_final_(period_final), 
      steps_(steps),
      tf_(t0+period_init1+period_init2+steps*period+period_final),
      q0_(q0),
      qf_(q0),
      v_ref_(v_ref),
      v_ref_init1_(0.25*v_ref*period/period_init1),
      v_ref_init2_(0.5*v_ref*period/period_init2),
      v_ref_final_(0.75*v_ref*period/period_final) {
    qf_.coeffRef(0) += period_init1 * v_ref_init1_;
    qf_.coeffRef(0) += period_init2 * v_ref_init2_;
    qf_.coeffRef(0) += steps * period * v_ref;
    qf_.coeffRef(0) += period_final * v_ref_final_;
  }

  ~TimeVaryingConfigurationRef() {}

  void update_q_ref(const robotoc::Robot& robot, const double t, 
                    Eigen::VectorXd& q_ref) const override {
    if (t < t0_) {
      q_ref = q0_;
    }
    else if (t < t0_ + period_init1_) {
      q_ref = q0_;
      q_ref.coeffRef(0) += (t-t0_) * v_ref_init1_;
    }
    else if (t < t0_ + period_init1_ + period_init2_) {
      q_ref = q0_;
      q_ref.coeffRef(0) += period_init1_ * v_ref_init1_;
      q_ref.coeffRef(0) += (t-t0_-period_init1_) * v_ref_init2_;
    }
    else if (t < tf_-period_final_) {
      q_ref = q0_;
      q_ref.coeffRef(0) += period_init1_ * v_ref_init1_;
      q_ref.coeffRef(0) += period_init2_ * v_ref_init2_;
      q_ref.coeffRef(0) += (t-t0_-period_init1_-period_init2_) * v_ref_;
    }
    else if (t < tf_) {
      q_ref = q0_;
      q_ref.coeffRef(0) += period_init1_ * v_ref_init1_;
      q_ref.coeffRef(0) += period_init2_ * v_ref_init2_;
      q_ref.coeffRef(0) += steps_ * period_ * v_ref_;
      q_ref.coeffRef(0) += (t-t0_-period_init1_-period_init2_-steps_*period_) * v_ref_final_;
    }
    else {
      q_ref = qf_;
    }
  }

  bool isActive(const double t) const override {
    return true;
  }

private:
  Eigen::VectorXd q0_, qf_;
  double t0_, period_init1_, period_init2_, period_, period_final_, tf_, 
         v_ref_, v_ref_init1_, v_ref_init2_, v_ref_final_;
  int steps_;
};


int main(int argc, char *argv[]) {
  const int LF_foot_id = 12;
  const int LH_foot_id = 22;
  const int RF_foot_id = 32;
  const int RH_foot_id = 42;
  robotoc::ContactFrames contact_frames; 
  contact_frames.point_contact_frames = {LF_foot_id, LH_foot_id, RF_foot_id, RH_foot_id}; 
  const std::string path_to_urdf = "../anymal_b_simple_description/urdf/anymal.urdf";
  const double baumgarte_time_step = 0.04;
  robotoc::Robot robot(path_to_urdf, robotoc::BaseJointType::FloatingBase, 
                       contact_frames, baumgarte_time_step);

  const double stride = 0.45;
  const double additive_stride_hip = 0.2;
  const double t_start = 1.0;

  const double t_front_swing = 0.135;
  const double t_front_hip_swing = 0.05;
  const double t_hip_swing = 0.165;
  const double t_period = t_front_swing + t_front_hip_swing + t_hip_swing;
  const int steps = 10;

  // Create the cost function
  auto cost = std::make_shared<robotoc::CostFunction>();
  Eigen::VectorXd v_weight(Eigen::VectorXd::Zero(robot.dimv()));
  v_weight << 0.01, 0.01, 0.01, 0.1, 0.1, 0.1, 
              0.1, 0.1, 0.1,
              0.1, 0.1, 0.1,
              0.1, 0.1, 0.1,
              0.1, 0.1, 0.1;
  Eigen::VectorXd a_weight(Eigen::VectorXd::Constant(robot.dimv(), 0.001));

  auto config_cost = std::make_shared<robotoc::ConfigurationSpaceCost>(robot);
  config_cost->set_v_weight(v_weight);
  config_cost->set_vf_weight(v_weight);
  config_cost->set_vi_weight(v_weight);
  config_cost->set_a_weight(a_weight);
  config_cost->set_dvi_weight(a_weight);
  cost->push_back(config_cost);

  Eigen::VectorXd q_standing(Eigen::VectorXd::Zero(robot.dimq()));
  q_standing << -3, 0, 0.4792, 0, 0, 0, 1, 
                -0.1,  0.7, -1.0, 
                -0.1, -0.7,  1.0, 
                 0.1,  0.7, -1.0, 
                 0.1, -0.7,  1.0;
  Eigen::VectorXd q_weight(Eigen::VectorXd::Zero(robot.dimv()));
  q_weight << 100, 100, 100, 100, 100, 100, 
              1, 1, 1,
              1, 1, 1,
              1, 1, 1,
              1, 1, 1;
  const double v_ref = stride / t_period;
  auto config_ref = std::make_shared<TimeVaryingConfigurationRef>(t_start, 0.255, 0.34, t_period, 0.5, steps,
                                                                  q_standing, v_ref);
  auto time_varying_config_cost = std::make_shared<robotoc::TimeVaryingConfigurationSpaceCost>(robot, config_ref);
  time_varying_config_cost->set_q_weight(q_weight);
  time_varying_config_cost->set_qf_weight(q_weight);
  time_varying_config_cost->set_qi_weight(q_weight);
  cost->push_back(time_varying_config_cost);

  // Create the constraints
  const double barrier = 1.0e-03;
  const double fraction_to_boundary_rule = 0.995;
  auto constraints           = std::make_shared<robotoc::Constraints>(barrier, fraction_to_boundary_rule);
  auto joint_position_lower  = std::make_shared<robotoc::JointPositionLowerLimit>(robot);
  auto joint_position_upper  = std::make_shared<robotoc::JointPositionUpperLimit>(robot);
  auto joint_velocity_lower  = std::make_shared<robotoc::JointVelocityLowerLimit>(robot);
  auto joint_velocity_upper  = std::make_shared<robotoc::JointVelocityUpperLimit>(robot);
  auto joint_torques_lower   = std::make_shared<robotoc::JointTorquesLowerLimit>(robot);
  auto joint_torques_upper   = std::make_shared<robotoc::JointTorquesUpperLimit>(robot);
  const double mu = 0.7;
  auto friction_cone         = std::make_shared<robotoc::FrictionCone>(robot, mu);
  auto impulse_friction_cone = std::make_shared<robotoc::ImpulseFrictionCone>(robot, mu);
  constraints->push_back(joint_position_lower);
  constraints->push_back(joint_position_upper);
  constraints->push_back(joint_velocity_lower);
  constraints->push_back(joint_velocity_upper);
  constraints->push_back(joint_torques_lower);
  constraints->push_back(joint_torques_upper);
  constraints->push_back(friction_cone);
  constraints->push_back(impulse_friction_cone);

  // Create the contact sequence
  const int max_num_impulses = (steps+3)*2;
  auto contact_sequence = std::make_shared<robotoc::ContactSequence>(robot, max_num_impulses);

  robot.updateFrameKinematics(q_standing);
  std::vector<Eigen::Vector3d> contact_points = {robot.framePosition(LF_foot_id), 
                                                 robot.framePosition(LH_foot_id),
                                                 robot.framePosition(RF_foot_id),
                                                 robot.framePosition(RH_foot_id)};
  auto contact_status_standing = robot.createContactStatus();
  contact_status_standing.activateContacts({0, 1, 2, 3});
  auto contact_status_front_swing = robot.createContactStatus();
  contact_status_front_swing.activateContacts({1, 3});
  auto contact_status_hip_swing = robot.createContactStatus();
  contact_status_hip_swing.activateContacts({0, 2});
  auto contact_status_front_hip_swing = robot.createContactStatus();

  contact_status_standing.setContactPoints(contact_points);
  contact_sequence->initContactSequence(contact_status_standing);

  const double t_initial_front_swing = 0.125;
  const double t_initial_front_hip_swing = 0.05;
  const double t_initial_hip_swing = 0.125;
  const double t_initial = t_initial_front_swing + t_initial_front_hip_swing + t_initial_hip_swing;
  const double t_initial_front_swing2 = 0.135;
  const double t_initial_front_hip_swing2 = 0.055;
  const double t_initial_hip_swing2 = 0.15;
  const double t_initial2 = t_initial_front_swing2 + t_initial_front_hip_swing2 + t_initial_hip_swing2;

  contact_status_front_swing.setContactPoints(contact_points);
  contact_sequence->push_back(contact_status_front_swing, t_start);
  contact_status_front_hip_swing.setContactPoints(contact_points);
  contact_sequence->push_back(contact_status_front_hip_swing, 
                              t_start+t_initial_front_swing);

  contact_points[0].coeffRef(0) += 0.25 * stride;
  contact_points[1].coeffRef(0) += 0.25 * stride + 0.5 * additive_stride_hip;
  contact_points[2].coeffRef(0) += 0.25 * stride;
  contact_points[3].coeffRef(0) += 0.25 * stride + 0.5 * additive_stride_hip;

  contact_status_hip_swing.setContactPoints(contact_points);
  contact_sequence->push_back(contact_status_hip_swing, 
                              t_start+t_initial_front_swing+t_initial_front_hip_swing);

  contact_status_front_swing.setContactPoints(contact_points);
  contact_sequence->push_back(contact_status_front_swing, t_start+t_initial);
  contact_status_front_hip_swing.setContactPoints(contact_points);
  contact_sequence->push_back(contact_status_front_hip_swing, 
                              t_start+t_initial+t_initial_front_swing2);

  contact_points[0].coeffRef(0) += 0.5 * stride;
  contact_points[1].coeffRef(0) += 0.5 * stride + 0.5 * additive_stride_hip;
  contact_points[2].coeffRef(0) += 0.5 * stride;
  contact_points[3].coeffRef(0) += 0.5 * stride + 0.5 * additive_stride_hip;

  contact_status_hip_swing.setContactPoints(contact_points);
  contact_sequence->push_back(contact_status_hip_swing, 
                              t_start+t_initial+t_initial_front_swing2+t_initial_front_hip_swing2);
  const double t_end_init = t_start+t_initial+t_initial2;

  for (int i=0; i<steps; ++i) {
    contact_status_front_swing.setContactPoints(contact_points);
    contact_sequence->push_back(contact_status_front_swing, t_end_init+i*t_period);
    contact_sequence->push_back(contact_status_front_hip_swing, 
                                t_end_init+i*t_period+t_front_swing);
    contact_points[0].coeffRef(0) += stride;
    contact_points[2].coeffRef(0) += stride;
    contact_points[1].coeffRef(0) += stride;
    contact_points[3].coeffRef(0) += stride;
    contact_status_hip_swing.setContactPoints(contact_points);
    contact_sequence->push_back(contact_status_hip_swing, 
                                t_end_init+i*t_period+t_front_swing+t_front_hip_swing);
  }

  contact_status_front_swing.setContactPoints(contact_points);
  contact_sequence->push_back(contact_status_front_swing, t_end_init+steps*t_period);

  // For the last step
  const double t_end_front_swing = 0.15;
  const double t_end_front_hip_swing = 0.05;
  const double t_end_hip_swing = 0.15;
  const double t_end = t_end_front_swing + t_end_front_hip_swing + t_end_hip_swing;

  contact_sequence->push_back(contact_status_front_hip_swing, 
                              t_end_init+steps*t_period+t_end_front_swing);

  contact_points[0].coeffRef(0) += 0.75 * stride;
  contact_points[2].coeffRef(0) += 0.75 * stride;
  contact_points[1].coeffRef(0) += 0.75 * stride - additive_stride_hip;
  contact_points[3].coeffRef(0) += 0.75 * stride - additive_stride_hip;
  contact_status_hip_swing.setContactPoints(contact_points);
  contact_sequence->push_back(contact_status_hip_swing, 
                              t_end_init+steps*t_period+t_end_front_swing+t_end_front_hip_swing);
  contact_status_standing.setContactPoints(contact_points);
  contact_sequence->push_back(contact_status_standing, t_end_init+steps*t_period+t_end);

  // you can check the contact sequence via
  // std::cout << contact_sequence << std::endl;

  // Create the OCP solver.
  const double T = 7; 
  const int N = 240;
  robotoc::OCP ocp(robot, cost, constraints, T, N, max_num_impulses);
  auto solver_options = robotoc::SolverOptions::defaultOptions();
  const int nthreads = 4;
  robotoc::OCPSolver ocp_solver(ocp, contact_sequence, solver_options, nthreads);

  // Initial time and initial state
  const double t = 0;
  const Eigen::VectorXd q(q_standing);
  const Eigen::VectorXd v(Eigen::VectorXd::Zero(robot.dimv()));

  // Solves the OCP.
  ocp_solver.setSolution("q", q);
  ocp_solver.setSolution("v", v);
  Eigen::Vector3d f_init;
  f_init << 0, 0, 0.25*robot.totalWeight();
  ocp_solver.setSolution("f", f_init);
  ocp_solver.setSolution("lmd", f_init);
  ocp_solver.initConstraints(t);
  std::cout << "Initial KKT error: " << ocp_solver.KKTError(t, q, v) << std::endl;
  ocp_solver.solve(t, q, v);
  std::cout << "KKT error after convergence: " << ocp_solver.KKTError(t, q, v) << std::endl;
  std::cout << ocp_solver.getSolverStatistics() << std::endl;

  // const int num_iteration = 10000;
  // robotoc::benchmark::CPUTime(ocp_solver, t, q, v, num_iteration);

#ifdef ENABLE_VIEWER
  robotoc::TrajectoryViewer viewer(path_to_urdf, robotoc::BaseJointType::FloatingBase);
  const auto discretization = ocp_solver.getTimeDiscretization();
  const auto time_steps = discretization.timeSteps();
  Eigen::Vector3d camera_pos;
  Eigen::Vector4d camera_quat;
  camera_pos << 0.119269, -7.96283, 1.95978;
  camera_quat << 0.609016, 0.00297497, 0.010914, 0.793077;
  viewer.setCameraTransform(camera_pos, camera_quat);
  viewer.display(ocp_solver.getSolution("q"), time_steps);
  camera_pos << 5.10483, -3.98692, 1.59321;
  camera_quat << 0.547037, 0.243328, 0.314829, 0.736495;
  viewer.setCameraTransform(camera_pos, camera_quat);
  viewer.display(robot, ocp_solver.getSolution("q"), 
                 ocp_solver.getSolution("f", "WORLD"), time_steps, mu);
#endif 

  return 0;
}