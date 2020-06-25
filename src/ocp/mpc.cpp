#include "ocp/mpc.hpp"

#include <assert.h>


namespace idocp {

MPC::MPC(const Robot& robot, const CostFunctionInterface* cost,
         const ConstraintsInterface* constraints, const double T, 
         const unsigned int N, const unsigned int num_proc) 
  : ocp_(robot, cost, constraints, T, N, num_proc) {
  assert(T > 0);
  assert(N > 0);
  assert(num_proc > 0);
}


void MPC::initializeSolution(const double t, const Eigen::VectorXd& q, 
                             const Eigen::VectorXd& v, 
                             const unsigned int max_itr) {
  assert(max_itr >= 0);
  ocp_.setStateTrajectory(q, v);
  for (int i=0; i<max_itr; ++i) {
    ocp_.solveSQP(t, q, v, true);
  }
}


void MPC::updateSolution(const double t, const Eigen::VectorXd& q, 
                         const Eigen::VectorXd& v) {
  ocp_.solveSQP(t, q, v, false);
}


void MPC::getControlInput(Eigen::VectorXd& u) {
  ocp_.getInitialControlInput(u);
}


double MPC::KKTError(const double t, const Eigen::VectorXd& q, 
                     const Eigen::VectorXd& v) {
  return ocp_.KKTError(t, q, v);
}

} // namespace idocp