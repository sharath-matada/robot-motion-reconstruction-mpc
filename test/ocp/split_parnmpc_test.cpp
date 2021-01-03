#include <string>
#include <memory>

#include <gtest/gtest.h>
#include "Eigen/Core"

#include "idocp/robot/robot.hpp"
#include "idocp/robot/contact_status.hpp"
#include "idocp/ocp/split_parnmpc.hpp"
#include "idocp/ocp/split_solution.hpp"
#include "idocp/ocp/split_direction.hpp"
#include "idocp/ocp/split_kkt_residual.hpp"
#include "idocp/ocp/split_kkt_matrix.hpp"
#include "idocp/ocp/backward_correction.hpp"
#include "idocp/cost/cost_function.hpp"
#include "idocp/cost/cost_function_data.hpp"
#include "idocp/cost/joint_space_cost.hpp"
#include "idocp/cost/task_space_3d_cost.hpp"
#include "idocp/cost/contact_force_cost.hpp"
#include "idocp/constraints/constraints.hpp"
#include "idocp/constraints/joint_position_lower_limit.hpp"
#include "idocp/constraints/joint_position_upper_limit.hpp"
#include "idocp/constraints/joint_velocity_lower_limit.hpp"
#include "idocp/constraints/joint_velocity_upper_limit.hpp"


namespace idocp {

class SplitParNMPCTest : public ::testing::Test {
protected:
  virtual void SetUp() {
    srand((unsigned int) time(0));
    fixed_base_urdf = "../urdf/iiwa14/iiwa14.urdf";
    floating_base_urdf = "../urdf/anymal/anymal.urdf";
  }

  virtual void TearDown() {
  }

  static std::shared_ptr<CostFunction> createCost(const Robot& robot);

  static std::shared_ptr<Constraints> createConstraints(const Robot& robot);

  static SplitSolution generateFeasibleSolution(
      Robot& robot, const ContactStatus& contact_staus,
      const std::shared_ptr<Constraints>& constraints);

  static void testLinearizeOCPAndBackwardCorrection(
      Robot& robot, const ContactStatus& contact_status, 
      const std::shared_ptr<CostFunction>& cost,
      const std::shared_ptr<Constraints>& constraints);

  static void testComputeKKTResidualEmptyCostAndEmptyConstraints(
      Robot& robot, const ContactStatus& contact_status);

  static void testComputeKKTResidualEmptyCost(
      Robot& robot, const ContactStatus& contact_status, 
      const std::shared_ptr<Constraints>& constraints);

  static void testComputeKKTResidualEmptyConstraints(
      Robot& robot, const ContactStatus& contact_status, 
      const std::shared_ptr<CostFunction>& cost);

  static void testComputeKKTResidual(
      Robot& robot, const ContactStatus& contact_status, 
      const std::shared_ptr<CostFunction>& cost,
      const std::shared_ptr<Constraints>& constraints);

  static void testCostAndConstraintViolation(
      Robot& robot, const ContactStatus& contact_status, 
      const std::shared_ptr<CostFunction>& cost,
      const std::shared_ptr<Constraints>& constraints);

  std::string fixed_base_urdf, floating_base_urdf;
};


std::shared_ptr<CostFunction> SplitParNMPCTest::createCost(const Robot& robot) {
  auto joint_cost = std::make_shared<JointSpaceCost>(robot);
  const Eigen::VectorXd q_weight = Eigen::VectorXd::Random(robot.dimv()).array().abs();
  Eigen::VectorXd q_ref = Eigen::VectorXd::Random(robot.dimq());
  robot.normalizeConfiguration(q_ref);
  const Eigen::VectorXd v_weight = Eigen::VectorXd::Random(robot.dimv()).array().abs();
  const Eigen::VectorXd v_ref = Eigen::VectorXd::Random(robot.dimv());
  const Eigen::VectorXd a_weight = Eigen::VectorXd::Random(robot.dimv()).array().abs();
  const Eigen::VectorXd a_ref = Eigen::VectorXd::Random(robot.dimv());
  const Eigen::VectorXd u_weight = Eigen::VectorXd::Random(robot.dimu()).array().abs();
  const Eigen::VectorXd u_ref = Eigen::VectorXd::Random(robot.dimu());
  const Eigen::VectorXd qf_weight = Eigen::VectorXd::Random(robot.dimv()).array().abs();
  const Eigen::VectorXd vf_weight = Eigen::VectorXd::Random(robot.dimv()).array().abs();
  joint_cost->set_q_weight(q_weight);
  joint_cost->set_q_ref(q_ref);
  joint_cost->set_v_weight(v_weight);
  joint_cost->set_v_ref(v_ref);
  joint_cost->set_a_weight(a_weight);
  joint_cost->set_a_ref(a_ref);
  joint_cost->set_u_weight(u_weight);
  joint_cost->set_u_ref(u_ref);
  joint_cost->set_qf_weight(qf_weight);
  joint_cost->set_vf_weight(vf_weight);
  const int task_frame = 10;
  auto task_space_3d_cost = std::make_shared<TaskSpace3DCost >(robot, task_frame);
  const Eigen::Vector3d q_3d_weight = Eigen::Vector3d::Random().array().abs();
  const Eigen::Vector3d qf_3d_weight = Eigen::Vector3d::Random().array().abs();
  const Eigen::Vector3d q_3d_ref = Eigen::Vector3d::Random();
  task_space_3d_cost->set_q_3d_weight(q_3d_weight);
  task_space_3d_cost->set_qf_3d_weight(qf_3d_weight);
  task_space_3d_cost->set_q_3d_ref(q_3d_ref);
  auto contact_force_cost = std::make_shared<idocp::ContactForceCost>(robot);
  std::vector<Eigen::Vector3d> f_weight;
  for (int i=0; i<robot.maxPointContacts(); ++i) {
    f_weight.push_back(Eigen::Vector3d::Constant(0.001));
  }
  contact_force_cost->set_f_weight(f_weight);
  auto cost = std::make_shared<CostFunction>();
  cost->push_back(joint_cost);
  cost->push_back(task_space_3d_cost);
  cost->push_back(contact_force_cost);
  return cost;
}


std::shared_ptr<Constraints> SplitParNMPCTest::createConstraints(const Robot& robot) {
  auto joint_lower_limit = std::make_shared<JointPositionLowerLimit>(robot);
  auto joint_upper_limit = std::make_shared<JointPositionUpperLimit>(robot);
  auto velocity_lower_limit = std::make_shared<JointVelocityLowerLimit>(robot);
  auto velocity_upper_limit = std::make_shared<JointVelocityUpperLimit>(robot);
  auto constraints = std::make_shared<Constraints>();
  constraints->push_back(joint_upper_limit); 
  constraints->push_back(joint_lower_limit);
  constraints->push_back(velocity_lower_limit); 
  constraints->push_back(velocity_upper_limit);
  return constraints;
}


SplitSolution SplitParNMPCTest::generateFeasibleSolution(
    Robot& robot, const ContactStatus& contact_staus,
    const std::shared_ptr<Constraints>& constraints) {
  auto data = constraints->createConstraintsData(robot, 10);
  SplitSolution s = SplitSolution::Random(robot, contact_staus);
  while (!constraints->isFeasible(robot, data, s)) {
    s = SplitSolution::Random(robot, contact_staus);
  }
  return s;
}


void SplitParNMPCTest::testLinearizeOCPAndBackwardCorrection(
    Robot& robot, const ContactStatus& contact_status, 
    const std::shared_ptr<CostFunction>& cost,
    const std::shared_ptr<Constraints>& constraints) {
  // const SplitSolution s_prev = SplitSolution::Random(robot, contact_status);
  // const SplitSolution s_new_prev = SplitSolution::Random(robot, contact_status);
  // const SplitSolution s = generateFeasibleSolution(robot, contact_status, constraints);
  // const SplitSolution s_next = SplitSolution::Random(robot, contact_status);
  // const SplitSolution s_new_next = SplitSolution::Random(robot, contact_status);
  // SplitParNMPC parnmpc(robot, cost, constraints);
  // const double t = std::abs(Eigen::VectorXd::Random(1)[0]);
  // const double dtau = std::abs(Eigen::VectorXd::Random(1)[0]);
  // parnmpc.initConstraints(robot, 10, dtau, s);
  // SplitDirection d(robot), d_ref(robot);
  // d.setContactStatus(contact_status);
  // d_ref.setContactStatus(contact_status);
  // const int dimv = robot.dimv();
  // const Eigen::MatrixXd seed_mat = Eigen::MatrixXd::Random(2*dimv, 2*dimv);
  // const Eigen::MatrixXd aux_mat_next_old = seed_mat * seed_mat.transpose() + Eigen::MatrixXd::Identity(2*dimv, 2*dimv);
  // parnmpc.linearizeOCP(robot, contact_status, t, dtau, s_prev.q, s_prev.v, s, s_next);
  // SplitSolution s_new_coarse(robot);
  // s_new_coarse.setContactStatus(contact_status);
  // parnmpc.coarseUpdate(robot, s, aux_mat_next_old, d, s_new_coarse);
  // SplitKKTMatrix kkt_matrix(robot);
  // kkt_matrix.setContactStatus(contact_status);
  // SplitKKTResidual kkt_residual(robot);
  // kkt_residual.setContactStatus(contact_status);
  // auto cost_data = cost->createCostFunctionData(robot);
  // auto constraints_data = constraints->createConstraintsData(robot, 10);
  // constraints->setSlackAndDual(robot, constraints_data, dtau, s);
  // robot.updateKinematics(s.q, s.v, s.a);
  // cost->computeStageCostDerivatives(robot, cost_data, t, dtau, s, kkt_residual);
  // cost->computeStageCostHessian(robot, cost_data, t, dtau, s, kkt_matrix);
  // constraints->augmentDualResidual(robot, constraints_data, dtau, s, kkt_residual);
  // constraints->condenseSlackAndDual(robot, constraints_data, dtau, s, kkt_matrix, kkt_residual);
  // stateequation::LinearizeBackwardEuler(robot, dtau, s_prev.q, s_prev.v, s, s_next, 
  //                                       kkt_matrix, kkt_residual);
  // ContactDynamics cd(robot);
  // robot.updateKinematics(s.q, s.v, s.a);
  // cd.linearizeContactDynamics(robot, contact_status, dtau, s, kkt_matrix, kkt_residual);
  // cd.condenseContactDynamics(robot, contact_status, dtau, kkt_matrix, kkt_residual);
  // BackwardCorrection bc(robot);
  // SplitSolution s_new_coarse_ref(robot);
  // s_new_coarse_ref.setContactStatus(contact_status);
  // kkt_matrix.Qxx() += aux_mat_next_old;
  // kkt_matrix.symmetrize();
  // bc.coarseUpdate(robot, s, d_ref, kkt_matrix, kkt_residual, s_new_coarse_ref);
  // EXPECT_TRUE(s_new_coarse.isApprox(s_new_coarse_ref));
  // EXPECT_TRUE(d.isApprox(d_ref));
  // parnmpc.backwardCorrectionSerial(s_next, s_new_next, s_new_coarse);
  // bc.backwardCorrectionSerial(s_next, s_new_next, s_new_coarse_ref);
  // EXPECT_TRUE(s_new_coarse.isApprox(s_new_coarse_ref));
  // parnmpc.backwardCorrectionParallel(robot, d_ref, s_new_coarse);
  // bc.backwardCorrectionParallel(robot, d, s_new_coarse_ref);
  // EXPECT_TRUE(s_new_coarse.isApprox(s_new_coarse_ref));
  // EXPECT_TRUE(d.isApprox(d_ref));
  // parnmpc.forwardCorrectionSerial(robot, s_prev, s_new_prev, s_new_coarse);
  // bc.forwardCorrectionSerial(robot, s_prev, s_new_prev, s_new_coarse_ref);
  // EXPECT_TRUE(s_new_coarse.isApprox(s_new_coarse_ref));
  // parnmpc.forwardCorrectionParallel(d, s_new_coarse);
  // bc.forwardCorrectionParallel(d_ref, s_new_coarse_ref);
  // EXPECT_TRUE(s_new_coarse.isApprox(s_new_coarse_ref));
  // EXPECT_TRUE(d.isApprox(d_ref));
  // Eigen::MatrixXd aux_mat = Eigen::MatrixXd::Zero(2*dimv, 2*dimv);
  // parnmpc.getAuxiliaryMatrix(aux_mat);
  // Eigen::MatrixXd aux_mat_ref = Eigen::MatrixXd::Zero(2*dimv, 2*dimv);
  // bc.getAuxiliaryMatrix(aux_mat_ref);
  // EXPECT_TRUE(aux_mat.isApprox(aux_mat_ref));
}


void SplitParNMPCTest::testComputeKKTResidualEmptyCostAndEmptyConstraints(
    Robot& robot, const ContactStatus& contact_status) {
  auto empty_cost = std::make_shared<CostFunction>();
  auto empty_constraints = std::make_shared<Constraints>();
  const SplitSolution s_prev = SplitSolution::Random(robot, contact_status);
  const SplitSolution s = SplitSolution::Random(robot, contact_status);
  const SplitSolution s_next = SplitSolution::Random(robot, contact_status);
  SplitParNMPC parnmpc(robot, empty_cost, empty_constraints);
  const double t = std::abs(Eigen::VectorXd::Random(1)[0]);
  const double dtau = std::abs(Eigen::VectorXd::Random(1)[0]);
  parnmpc.initConstraints(robot, 10, dtau, s);
  parnmpc.computeKKTResidual(robot, contact_status, t, dtau, s_prev.q, s_prev.v, s, s_next);
  const double kkt_error = parnmpc.squaredNormKKTResidual(dtau);
  SplitKKTMatrix kkt_matrix(robot);
  kkt_matrix.setContactStatus(contact_status);
  SplitKKTResidual kkt_residual(robot);
  kkt_residual.setContactStatus(contact_status);
  if (robot.hasFloatingBase()) {
    robot.subtractConfiguration(s_prev.q, s.q, kkt_residual.Fq());
    robot.dSubtractdConfigurationMinus(s_prev.q, s.q, kkt_matrix.Fqq());
    robot.dSubtractdConfigurationPlus(s.q, s_next.q, kkt_matrix.Fqq_prev);
    kkt_residual.Fq() += dtau * s.v;
    kkt_residual.Fv() = s_prev.v + dtau * s.a - s.v;
    kkt_residual.lq() = kkt_matrix.Fqq_prev.transpose() * s_next.lmd + kkt_matrix.Fqq().transpose() * s.lmd;
    kkt_residual.lv() = dtau * s.lmd - s.gmm + s_next.gmm;
    kkt_residual.la = dtau * s.gmm;
  }
  else {
    kkt_residual.Fq() = s_prev.q - s.q + dtau * s.v;
    kkt_residual.Fv() = s_prev.v + dtau * s.a - s.v;
    kkt_residual.lq() = s_next.lmd - s.lmd;
    kkt_residual.lv() = dtau * s.lmd - s.gmm + s_next.gmm;
    kkt_residual.la = dtau * s.gmm;
  }
  Eigen::VectorXd ID = Eigen::VectorXd::Zero(robot.dimv());
  Eigen::MatrixXd dID_dq = Eigen::MatrixXd::Zero(robot.dimv(), robot.dimv());
  Eigen::MatrixXd dID_dv = Eigen::MatrixXd::Zero(robot.dimv(), robot.dimv());
  Eigen::MatrixXd dID_da = Eigen::MatrixXd::Zero(robot.dimv(), robot.dimv());
  robot.setContactForces(contact_status, s.f);
  robot.RNEA(s.q, s.v, s.a, ID);
  if (robot.hasFloatingBase()) {
    ID.head(robot.dim_passive()) -= s.u_passive;
    ID.tail(robot.dimu()) -= s.u;
  }
  else {
    ID -= s.u;
  }
  robot.RNEADerivatives(s.q, s.v, s.a, dID_dq, dID_dv, dID_da);
  kkt_residual.lq() += dtau * dID_dq.transpose() * s.beta;
  kkt_residual.lv() += dtau * dID_dv.transpose() * s.beta;
  kkt_residual.la += dtau * dID_da.transpose() * s.beta;
  if (robot.hasFloatingBase()) {
    kkt_residual.lu() -= dtau * s.beta.tail(robot.dimu());
    kkt_residual.lu_passive -= dtau * s.beta.head(robot.dim_passive());
    kkt_residual.lu_passive += dtau * s.nu_passive;
  }
  else {
    kkt_residual.lu() -= dtau * s.beta;
  }
  Eigen::VectorXd C = Eigen::VectorXd::Zero(contact_status.dimf());
  if (contact_status.hasActiveContacts()) {
    Eigen::MatrixXd dID_df = Eigen::MatrixXd::Zero(robot.dimv(), contact_status.dimf());
    robot.updateKinematics(s.q, s.v, s.a);
    robot.dRNEAPartialdFext(contact_status, dID_df);
    kkt_residual.lf() += dtau * dID_df.transpose() * s.beta;
    Eigen::MatrixXd dC_dq = Eigen::MatrixXd::Zero(contact_status.dimf(), robot.dimv());
    Eigen::MatrixXd dC_dv = Eigen::MatrixXd::Zero(contact_status.dimf(), robot.dimv());
    Eigen::MatrixXd dC_da = Eigen::MatrixXd::Zero(contact_status.dimf(), robot.dimv());
    robot.updateKinematics(s.q, s.v, s.a);
    robot.computeBaumgarteResidual(contact_status, dtau, C);
    robot.computeBaumgarteDerivatives(contact_status, dtau, dC_dq, dC_dv, dC_da);
    kkt_residual.lq() += dtau * dC_dq.transpose() * s.mu_stack();
    kkt_residual.lv() += dtau * dC_dv.transpose() * s.mu_stack();
    kkt_residual.la += dtau * dC_da.transpose() * s.mu_stack();
  }
  double kkt_error_ref = kkt_residual.Fx().squaredNorm()
                         + kkt_residual.lx().squaredNorm()
                         + kkt_residual.la.squaredNorm()
                         + kkt_residual.lf().squaredNorm()
                         + kkt_residual.lu().squaredNorm()
                         + dtau * dtau * ID.squaredNorm();
  if (robot.hasFloatingBase()) {
    kkt_error_ref += kkt_residual.lu_passive.squaredNorm();
    kkt_error_ref += dtau * dtau * s.u_passive.squaredNorm();
  }
  if (contact_status.hasActiveContacts()) {
    kkt_error_ref += dtau * dtau * C.squaredNorm();
  }
  EXPECT_DOUBLE_EQ(kkt_error, kkt_error_ref);
  double constraint_violation_ref = kkt_residual.Fx().lpNorm<1>() + dtau * ID.lpNorm<1>();
  if (robot.hasFloatingBase()) {
    constraint_violation_ref += dtau * s.u_passive.lpNorm<1>();
  }
  if (contact_status.hasActiveContacts()) {
    constraint_violation_ref += dtau * C.lpNorm<1>();
  }
}


void SplitParNMPCTest::testComputeKKTResidualEmptyCost(
    Robot& robot, const ContactStatus& contact_status, 
    const std::shared_ptr<Constraints>& constraints) {
  auto empty_cost = std::make_shared<CostFunction>();
  const SplitSolution s_prev = SplitSolution::Random(robot, contact_status);
  const SplitSolution s = SplitSolution::Random(robot, contact_status);
  const SplitSolution s_next = SplitSolution::Random(robot, contact_status);
  SplitParNMPC parnmpc(robot, empty_cost, constraints);
  const double t = std::abs(Eigen::VectorXd::Random(1)[0]);
  const double dtau = std::abs(Eigen::VectorXd::Random(1)[0]);
  parnmpc.initConstraints(robot, 10, dtau, s);
  parnmpc.computeKKTResidual(robot, contact_status, t, dtau, s_prev.q, s_prev.v, s, s_next);
  const double kkt_error = parnmpc.squaredNormKKTResidual(dtau);
  SplitKKTMatrix kkt_matrix(robot);
  kkt_matrix.setContactStatus(contact_status);
  SplitKKTResidual kkt_residual(robot);
  kkt_residual.setContactStatus(contact_status);
  auto constraints_data = constraints->createConstraintsData(robot, 10);
  constraints->setSlackAndDual(robot, constraints_data, dtau, s);
  constraints->augmentDualResidual(robot, constraints_data, dtau, s, kkt_residual);
  stateequation::LinearizeBackwardEuler(robot, dtau, s_prev.q, s_prev.v, s, s_next, 
                                        kkt_matrix, kkt_residual);
  ContactDynamics cd(robot);
  robot.updateKinematics(s.q, s.v, s.a);
  cd.linearizeContactDynamics(robot, contact_status, dtau, s, kkt_matrix, kkt_residual);
  double kkt_error_ref = kkt_residual.Fx().squaredNorm()
                         + kkt_residual.lx().squaredNorm()
                         + kkt_residual.la.squaredNorm()
                         + kkt_residual.lf().squaredNorm()
                         + kkt_residual.lu().squaredNorm()
                         + cd.squaredNormContactDynamicsResidual(dtau)
                         + constraints->squaredNormPrimalAndDualResidual(constraints_data);
  if (robot.hasFloatingBase()) {
    kkt_error_ref += kkt_residual.lu_passive.squaredNorm();
  }
  EXPECT_DOUBLE_EQ(kkt_error, kkt_error_ref);
}


void SplitParNMPCTest::testComputeKKTResidualEmptyConstraints(
    Robot& robot, const ContactStatus& contact_status, 
    const std::shared_ptr<CostFunction>& cost) {
  auto empty_constraints = std::make_shared<Constraints>();
  const SplitSolution s_prev = SplitSolution::Random(robot, contact_status);
  const SplitSolution s = SplitSolution::Random(robot, contact_status);
  const SplitSolution s_next = SplitSolution::Random(robot, contact_status);
  SplitParNMPC parnmpc(robot, cost, empty_constraints);
  const double t = std::abs(Eigen::VectorXd::Random(1)[0]);
  const double dtau = std::abs(Eigen::VectorXd::Random(1)[0]);
  parnmpc.initConstraints(robot, 10, dtau, s);
  parnmpc.computeKKTResidual(robot, contact_status, t, dtau, s_prev.q, s_prev.v, s, s_next);
  const double kkt_error = parnmpc.squaredNormKKTResidual(dtau);
  SplitKKTMatrix kkt_matrix(robot);
  kkt_matrix.setContactStatus(contact_status);
  SplitKKTResidual kkt_residual(robot);
  kkt_residual.setContactStatus(contact_status);
  auto cost_data = cost->createCostFunctionData(robot);
  robot.updateKinematics(s.q, s.v, s.a);
  cost->computeStageCostDerivatives(robot, cost_data, t, dtau, s, kkt_residual);
  stateequation::LinearizeBackwardEuler(robot, dtau, s_prev.q, s_prev.v, s, s_next, 
                                        kkt_matrix, kkt_residual);
  ContactDynamics cd(robot);
  robot.updateKinematics(s.q, s.v, s.a);
  cd.linearizeContactDynamics(robot, contact_status, dtau, s, kkt_matrix, kkt_residual);
  double kkt_error_ref = kkt_residual.Fx().squaredNorm()
                         + kkt_residual.lx().squaredNorm()
                         + kkt_residual.la.squaredNorm()
                         + kkt_residual.lf().squaredNorm()
                         + kkt_residual.lu().squaredNorm()
                         + cd.squaredNormContactDynamicsResidual(dtau);
  if (robot.hasFloatingBase()) {
    kkt_error_ref += kkt_residual.lu_passive.squaredNorm();
  }
  EXPECT_DOUBLE_EQ(kkt_error, kkt_error_ref);
}


void SplitParNMPCTest::testComputeKKTResidual(
    Robot& robot, const ContactStatus& contact_status, 
    const std::shared_ptr<CostFunction>& cost,
    const std::shared_ptr<Constraints>& constraints) {
  const SplitSolution s_prev = SplitSolution::Random(robot, contact_status);
  const SplitSolution s = SplitSolution::Random(robot, contact_status);
  const SplitSolution s_next = SplitSolution::Random(robot, contact_status);
  SplitParNMPC parnmpc(robot, cost, constraints);
  const double t = std::abs(Eigen::VectorXd::Random(1)[0]);
  const double dtau = std::abs(Eigen::VectorXd::Random(1)[0]);
  parnmpc.initConstraints(robot, 10, dtau, s);
  parnmpc.computeKKTResidual(robot, contact_status, t, dtau, s_prev.q, s_prev.v, s, s_next);
  const double kkt_error = parnmpc.squaredNormKKTResidual(dtau);
  SplitKKTMatrix kkt_matrix(robot);
  kkt_matrix.setContactStatus(contact_status);
  SplitKKTResidual kkt_residual(robot);
  kkt_residual.setContactStatus(contact_status);
  auto cost_data = cost->createCostFunctionData(robot);
  auto constraints_data = constraints->createConstraintsData(robot, 10);
  constraints->setSlackAndDual(robot, constraints_data, dtau, s);
  robot.updateKinematics(s.q, s.v, s.a);
  cost->computeStageCostDerivatives(robot, cost_data, t, dtau, s, kkt_residual);
  constraints->augmentDualResidual(robot, constraints_data, dtau, s, kkt_residual);
  stateequation::LinearizeBackwardEuler(robot, dtau, s_prev.q, s_prev.v, s, s_next, 
                                        kkt_matrix, kkt_residual);
  ContactDynamics cd(robot);
  robot.updateKinematics(s.q, s.v, s.a);
  cd.linearizeContactDynamics(robot, contact_status, dtau, s, kkt_matrix, kkt_residual);
  double kkt_error_ref = kkt_residual.Fx().squaredNorm()
                         + kkt_residual.lx().squaredNorm()
                         + kkt_residual.la.squaredNorm()
                         + kkt_residual.lf().squaredNorm()
                         + kkt_residual.lu().squaredNorm()
                         + cd.squaredNormContactDynamicsResidual(dtau)
                         + constraints->squaredNormPrimalAndDualResidual(constraints_data);
  if (robot.hasFloatingBase()) {
    kkt_error_ref += kkt_residual.lu_passive.squaredNorm();
  }
  EXPECT_DOUBLE_EQ(kkt_error, kkt_error_ref);
}


void SplitParNMPCTest::testCostAndConstraintViolation(
    Robot& robot, const ContactStatus& contact_status, 
    const std::shared_ptr<CostFunction>& cost,
    const std::shared_ptr<Constraints>& constraints) {
  const SplitSolution s_prev = SplitSolution::Random(robot, contact_status);
  const SplitSolution s = SplitSolution::Random(robot, contact_status);
  const SplitDirection d = SplitDirection::Random(robot, contact_status);
  SplitParNMPC parnmpc(robot, cost, constraints);
  const double t = std::abs(Eigen::VectorXd::Random(1)[0]);
  const double dtau = std::abs(Eigen::VectorXd::Random(1)[0]);
  const double step_size = 0.3;
  parnmpc.initConstraints(robot, 10, dtau, s);
  const double stage_cost = parnmpc.stageCost(robot, t, dtau, s, step_size);
  const double constraint_violation = parnmpc.constraintViolation(robot, contact_status, t, dtau, s_prev.q, s_prev.v, s);
  SplitKKTMatrix kkt_matrix(robot);
  kkt_matrix.setContactStatus(contact_status);
  SplitKKTResidual kkt_residual(robot);
  kkt_residual.setContactStatus(contact_status);
  auto cost_data = cost->createCostFunctionData(robot);
  auto constraints_data = constraints->createConstraintsData(robot, 10);
  constraints->setSlackAndDual(robot, constraints_data, dtau, s);
  robot.updateKinematics(s.q, s.v, s.a);
  double stage_cost_ref = 0;
  stage_cost_ref += cost->l(robot, cost_data, t, dtau, s);
  stage_cost_ref += constraints->costSlackBarrier(constraints_data, step_size);
  EXPECT_DOUBLE_EQ(stage_cost, stage_cost_ref);
  constraints->computePrimalAndDualResidual(robot, constraints_data, dtau, s);
  stateequation::ComputeBackwardEulerResidual(robot, dtau, s_prev.q, s_prev.v,
                                              s, kkt_residual);
  ContactDynamics cd(robot);
  cd.computeContactDynamicsResidual(robot, contact_status, dtau, s);
  double constraint_violation_ref = 0;
  constraint_violation_ref += constraints->l1NormPrimalResidual(constraints_data);
  constraint_violation_ref += stateequation::L1NormStateEuqationResidual(kkt_residual);
  constraint_violation_ref += cd.l1NormContactDynamicsResidual(dtau);
  EXPECT_DOUBLE_EQ(constraint_violation, constraint_violation_ref);
}


TEST_F(SplitParNMPCTest, fixedBase) {
  std::vector<int> contact_frames = {18};
  ContactStatus contact_status(contact_frames.size());
  Robot robot(fixed_base_urdf, contact_frames);
  contact_status.setContactStatus({false});
  const auto cost = createCost(robot);
  const auto constraints = createConstraints(robot);
  testLinearizeOCPAndBackwardCorrection(robot, contact_status, cost, constraints);
  testComputeKKTResidualEmptyCostAndEmptyConstraints(robot, contact_status);
  testComputeKKTResidualEmptyCost(robot, contact_status, constraints);
  testComputeKKTResidualEmptyConstraints(robot, contact_status, cost);
  testComputeKKTResidual(robot, contact_status, cost, constraints);
  testCostAndConstraintViolation(robot, contact_status, cost, constraints);
  contact_status.setContactStatus({true});
  testLinearizeOCPAndBackwardCorrection(robot, contact_status, cost, constraints);
  testComputeKKTResidualEmptyCostAndEmptyConstraints(robot, contact_status);
  testComputeKKTResidualEmptyCost(robot, contact_status, constraints);
  testComputeKKTResidualEmptyConstraints(robot, contact_status, cost);
  testComputeKKTResidual(robot, contact_status, cost, constraints);
  testCostAndConstraintViolation(robot, contact_status, cost, constraints);
}


TEST_F(SplitParNMPCTest, floatingBase) {
  std::vector<int> contact_frames = {14, 24, 34, 44};
  ContactStatus contact_status(contact_frames.size());
  Robot robot(floating_base_urdf, contact_frames);
  contact_status.setContactStatus({false, false, false, false});
  const auto cost = createCost(robot);
  const auto constraints = createConstraints(robot);
  testLinearizeOCPAndBackwardCorrection(robot, contact_status, cost, constraints);
  testComputeKKTResidualEmptyCostAndEmptyConstraints(robot, contact_status);
  testComputeKKTResidualEmptyCost(robot, contact_status, constraints);
  testComputeKKTResidualEmptyConstraints(robot, contact_status, cost);
  testComputeKKTResidual(robot, contact_status, cost, constraints);
  testCostAndConstraintViolation(robot, contact_status, cost, constraints);
  std::random_device rnd;
  std::vector<bool> is_contact_active;
  for (const auto frame : contact_frames) {
    is_contact_active.push_back(rnd()%2==0);
  }
  if (!contact_status.hasActiveContacts()) {
    contact_status.activateContact(0);
  }
  contact_status.setContactStatus(is_contact_active);
  testLinearizeOCPAndBackwardCorrection(robot, contact_status, cost, constraints);
  testComputeKKTResidualEmptyCostAndEmptyConstraints(robot, contact_status);
  testComputeKKTResidualEmptyCost(robot, contact_status, constraints);
  testComputeKKTResidualEmptyConstraints(robot, contact_status, cost);
  testComputeKKTResidual(robot, contact_status, cost, constraints);
  testCostAndConstraintViolation(robot, contact_status, cost, constraints);
}

} // namespace idocp


int main(int argc, char** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}