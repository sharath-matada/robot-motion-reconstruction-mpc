#include <iostream>
#include <string>
#include <memory>

#include "Eigen/Core"

#include "idocp/robot/robot.hpp"
#include "idocp/unocp/unocp_solver.hpp"
#include "idocp/cost/cost_function.hpp"
#include "idocp/cost/configuration_space_cost.hpp"
#include "idocp/cost/time_varying_task_space_6d_cost.hpp"
#include "idocp/constraints/constraints.hpp"
#include "idocp/utils/joint_constraints_factory.hpp"
#include "idocp/utils/ocp_benchmarker.hpp"

#ifdef ENABLE_VIEWER
#include "idocp/utils/trajectory_viewer.hpp"
#endif 


class TimeVaryingTaskSpace6DRef final : public idocp::TimeVaryingTaskSpace6DRefBase {
public:
  TimeVaryingTaskSpace6DRef() 
    : TimeVaryingTaskSpace6DRefBase() {
    rotm_  <<  0, 0, 1, 
               0, 1, 0,
              -1, 0, 0;
    pos0_ << 0.546, 0, 0.76;
    radius_ = 0.1;
  }

  ~TimeVaryingTaskSpace6DRef() {}

  void compute_q_6d_ref(const double t, 
                        pinocchio::SE3& se3_ref) const override {
    Eigen::Vector3d pos(pos0_);
    pos.coeffRef(1) += radius_ * sin(M_PI*t);
    pos.coeffRef(2) += radius_ * cos(M_PI*t);
    se3_ref = pinocchio::SE3(rotm_, pos);
  }

private:
  double radius_;
  Eigen::Matrix3d rotm_;
  Eigen::Vector3d pos0_;
};


int main(int argc, char *argv[]) {
  // Create a robot.
  const std::string path_to_urdf = "../iiwa_description/urdf/iiwa14.urdf";
  idocp::Robot robot(path_to_urdf);

  // Change the limits from the default parameters.
  robot.setJointEffortLimit(Eigen::VectorXd::Constant(robot.dimu(), 50));
  robot.setJointVelocityLimit(Eigen::VectorXd::Constant(robot.dimv(), M_PI_2));

  // Create a cost function.
  auto cost = std::make_shared<idocp::CostFunction>();
  auto config_cost = std::make_shared<idocp::ConfigurationSpaceCost>(robot);
  config_cost->set_q_weight(Eigen::VectorXd::Constant(robot.dimv(), 0));
  config_cost->set_qf_weight(Eigen::VectorXd::Constant(robot.dimv(), 0));
  config_cost->set_v_weight(Eigen::VectorXd::Constant(robot.dimv(), 0.01));
  config_cost->set_vf_weight(Eigen::VectorXd::Constant(robot.dimv(), 0.01));
  config_cost->set_a_weight(Eigen::VectorXd::Constant(robot.dimv(), 0.01));
  cost->push_back(config_cost);
  const int ee_frame_id = 22; 
  auto ref = std::make_shared<TimeVaryingTaskSpace6DRef>();
  auto task_cost = std::make_shared<idocp::TimeVaryingTaskSpace6DCost>(robot, ee_frame_id, ref);
  task_cost->set_q_6d_weight(Eigen::Vector3d::Constant(1000), Eigen::Vector3d::Constant(1000));
  task_cost->set_qf_6d_weight(Eigen::Vector3d::Constant(1000), Eigen::Vector3d::Constant(1000));
  cost->push_back(task_cost);

  // Create joint constraints.
  idocp::JointConstraintsFactory constraints_factory(robot);
  auto constraints = constraints_factory.create();

  // Create the OCP solver for unconstrained rigid-body systems.
  const double T = 6;
  const int N = 120;
  const int nthreads = 4;
  const double t = 0;
  Eigen::VectorXd q = Eigen::VectorXd::Zero(robot.dimq());
  q << 0, M_PI_2, 0, M_PI_2, 0, M_PI_2, 0;
  const Eigen::VectorXd v = Eigen::VectorXd::Zero(robot.dimv());
  idocp::UnOCPSolver ocp_solver(robot, cost, constraints, T, N, nthreads);

  // Solves the OCP.
  ocp_solver.setStateTrajectory(t, q, v);
  const int num_iteration = 30;
  const bool line_search = false;
  idocp::ocpbenchmarker::Convergence(ocp_solver, t, q, v, num_iteration, line_search);

#ifdef ENABLE_VIEWER
  if (argc != 2) {
    std::cout << "Invalid argment!" << std::endl;
    std::cout << "Package serach path must be specified as the second argment!" << std::endl;
  }
  else {
    const std::string pkg_search_path = argv[1];
    idocp::TrajectoryViewer viewer(pkg_search_path, path_to_urdf);
    const double dt = T/N;
    viewer.display(ocp_solver.getSolution("q"), dt);
  }
#endif 

  return 0;
}