#ifndef ROBOTOC_SWITCHING_CONSTRAINT_HXX_ 
#define ROBOTOC_SWITCHING_CONSTRAINT_HXX_

#include "robotoc/ocp/switching_constraint.hpp"

#include <cassert>

namespace robotoc {

inline SwitchingConstraint::SwitchingConstraint(const Robot& robot)
  : q_(Eigen::VectorXd::Zero(robot.dimq())),
    dq_(Eigen::VectorXd::Zero(robot.dimv())),
    has_floating_base_(robot.hasFloatingBase()) {
}


inline SwitchingConstraint::SwitchingConstraint()
  : q_(),
    dq_(),
    has_floating_base_(false) {
}


inline SwitchingConstraint::~SwitchingConstraint() {
}


inline void SwitchingConstraint::evalSwitchingConstraint(
    Robot& robot, const ImpulseStatus& impulse_status, const double dt1, 
    const double dt2, const SplitSolution& s, 
    SwitchingConstraintResidual& sc_residual) {
  assert(dt1 > 0);
  assert(dt2 > 0);
  sc_residual.setImpulseStatus(impulse_status);
  dq_ = (dt1+dt2) * s.v + (dt1*dt2) * s.a;
  robot.integrateConfiguration(s.q, dq_, 1.0, q_);
  robot.updateKinematics(q_);
  robot.computeContactPositionResidual(impulse_status, 
                                       impulse_status.contactPoints(), 
                                       sc_residual.P());
}


inline void SwitchingConstraint::linearizeSwitchingConstraint(
    Robot& robot, const ImpulseStatus& impulse_status, const double dt1, 
    const double dt2, const SplitSolution& s, SplitKKTMatrix& kkt_matrix, 
    SplitKKTResidual& kkt_residual, SwitchingConstraintJacobian& sc_jacobian,
    SwitchingConstraintResidual& sc_residual) {
  assert(dt1 > 0);
  assert(dt2 > 0);
  sc_residual.setImpulseStatus(impulse_status);
  sc_jacobian.setImpulseStatus(impulse_status);
  sc_residual.setZero();
  sc_jacobian.setZero();
  evalSwitchingConstraint(robot, impulse_status, dt1, dt2, s, sc_residual);
  robot.computeContactPositionDerivative(impulse_status, sc_jacobian.Pq());
  if (has_floating_base_) {
    robot.dIntegrateTransport_dq(s.q, dq_, sc_jacobian.Pq(), sc_jacobian.Phiq());
    robot.dIntegrateTransport_dv(s.q, dq_, sc_jacobian.Pq(), sc_jacobian.Phiv());
    sc_jacobian.Phia() = (dt1*dt2) * sc_jacobian.Phiv();
    sc_jacobian.Phiv().array() *= (dt1+dt2);
  }
  else {
    sc_jacobian.Phiq() = sc_jacobian.Pq();
    sc_jacobian.Phiv() = (dt1+dt2) * sc_jacobian.Pq();
    sc_jacobian.Phia() = (dt1*dt2) * sc_jacobian.Pq();
  }
  kkt_residual.lx.noalias() += sc_jacobian.Phix().transpose() * s.xi_stack();
  kkt_residual.la.noalias() += sc_jacobian.Phia().transpose() * s.xi_stack();
}

} // namespace robotoc

#endif // ROBOTOC_SWITCHING_CONSTRAINT_HXX_