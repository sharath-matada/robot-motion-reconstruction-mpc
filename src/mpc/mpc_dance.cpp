#include "robotoc/mpc/mpc_dance.hpp"

#include <stdexcept>
#include <iostream>
#include <cassert>
#include <cmath>
#include <limits>
#include <algorithm>

namespace robotoc {

MPCDance::MPCDance(const Robot& robot, const double T, const int N,
                   const Eigen::VectorXd& q0,
                   const Eigen::Vector3d& x3d0_LF, const Eigen::Vector3d& x3d0_LH,
                   const Eigen::Vector3d& x3d0_RF, const Eigen::Vector3d& x3d0_RH)
  : foot_step_planner_(),
    q0_(q0),
    x3d0_LF_(x3d0_LF),
    x3d0_LH_(x3d0_LH),
    x3d0_RF_(x3d0_RF),
    x3d0_RH_(x3d0_RH),
    contact_sequence_(std::make_shared<robotoc::ContactSequence>(robot)),
    cost_(std::make_shared<CostFunction>()),
    constraints_(std::make_shared<Constraints>(1.0e-03, 0.995)),
    ocp_solver_(OCP(robot, cost_, constraints_, contact_sequence_, T, N), 
                SolverOptions()), 
    solver_options_(SolverOptions()),
    cs_standing_(robot.createContactStatus()),
    cs_flying_(robot.createContactStatus()),
    cs_12_(robot.createContactStatus()),
    cs_01_(robot.createContactStatus()),
    cs_03_(robot.createContactStatus()),
    cs_23_(robot.createContactStatus()),
    cs_123_(robot.createContactStatus()),
    cs_012_(robot.createContactStatus()),
    cs_013_(robot.createContactStatus()),
    cs_023_(robot.createContactStatus()),
    T_(T),
    dt_(T/N),
    dtm_(T/N),
    ts_last_(0),
    eps_(std::sqrt(std::numeric_limits<double>::epsilon())),
    N_(N),
    current_step_(0),
    predict_step_(0) {
  if (robot.maxNumPointContacts() < 4) {
    throw std::out_of_range(
        "[MPCDance] invalid argument: 'robot' is not a quadrupedal robot!\n robot.maxNumPointContacts() must be larger than 4!");
  }
  config_cost_ = std::make_shared<ConfigurationSpaceCost>(robot);
  Eigen::VectorXd q_weight = Eigen::VectorXd::Constant(robot.dimv(), 5.0e01);
  q_weight.template head<6>() << 1.0e03, 1.0e03, 1.0e00, 1.0e03, 1.0e00, 1.0e03;
  Eigen::VectorXd q_weight_impact = Eigen::VectorXd::Constant(robot.dimv(), 1);
  q_weight_impact.template head<6>().setZero();
  config_cost_->set_q_weight(q_weight);
  config_cost_->set_q_weight_terminal(q_weight);
  config_cost_->set_q_weight_impact(q_weight_impact);
  config_cost_->set_v_weight(Eigen::VectorXd::Constant(robot.dimv(), 1.0));
  config_cost_->set_v_weight_terminal(Eigen::VectorXd::Constant(robot.dimv(), 1.0));
  config_cost_->set_u_weight(Eigen::VectorXd::Constant(robot.dimu(), 1.0e-02));
  config_cost_->set_v_weight_impact(Eigen::VectorXd::Constant(robot.dimv(), 1.0));
  config_cost_->set_dv_weight_impact(Eigen::VectorXd::Constant(robot.dimv(), 1.0e-03));
 
  com_cost_ = std::make_shared<ConfigurationSpaceCost>(robot);
  Eigen::VectorXd com_weight = Eigen::VectorXd::Constant(robot.dimv(),0);
  com_weight.template head<6>() << 0, 0, 0, 0, 5.0e04, 0;
  com_cost_->set_q_weight(com_weight);
  com_cost_->set_q_weight_terminal(com_weight);
  com_cost_->set_q_weight_impact(com_weight);

  LF_foot_cost_ = std::make_shared<TaskSpace3DCost>(robot, robot.contactFrames()[0],
                                                    LF_foot_ref_);
  LH_foot_cost_ = std::make_shared<TaskSpace3DCost>(robot, robot.contactFrames()[1],
                                                    LH_foot_ref_);
  RF_foot_cost_ = std::make_shared<TaskSpace3DCost>(robot, robot.contactFrames()[2],
                                                    RF_foot_ref_);
  RH_foot_cost_ = std::make_shared<TaskSpace3DCost>(robot, robot.contactFrames()[3],
                                                    RH_foot_ref_);

  LF_foot_cost_->set_weight(Eigen::Vector3d(1.0e04,1.0e04,1.0e05));
  LH_foot_cost_->set_weight(Eigen::Vector3d(1.0e04,1.0e04,1.0e05));
  RF_foot_cost_->set_weight(Eigen::Vector3d(1.0e04,1.0e04,1.0e05));
  RH_foot_cost_->set_weight(Eigen::Vector3d(1.0e04,1.0e04,1.0e05));

  cost_->push_back(LF_foot_cost_);
  cost_->push_back(LH_foot_cost_);
  cost_->push_back(RF_foot_cost_);
  cost_->push_back(RH_foot_cost_);

  cost_->push_back(com_cost_);
  cost_->push_back(config_cost_);

  // create constraints 
  auto joint_position_lower = std::make_shared<robotoc::JointPositionLowerLimit>(robot);
  auto joint_position_upper = std::make_shared<robotoc::JointPositionUpperLimit>(robot);
  auto joint_velocity_lower = std::make_shared<robotoc::JointVelocityLowerLimit>(robot);
  auto joint_velocity_upper = std::make_shared<robotoc::JointVelocityUpperLimit>(robot);
  auto joint_torques_lower  = std::make_shared<robotoc::JointTorquesLowerLimit>(robot);
  auto joint_torques_upper  = std::make_shared<robotoc::JointTorquesUpperLimit>(robot);
  friction_cone_ = std::make_shared<robotoc::FrictionCone>(robot);

  // auto impact_friction_cone = std::make_shared<robotoc::ImpactFrictionCone>(robot);
  constraints_->push_back(joint_position_lower);
  constraints_->push_back(joint_position_upper);
  constraints_->push_back(joint_velocity_lower);
  constraints_->push_back(joint_velocity_upper);
  constraints_->push_back(joint_torques_lower);
  constraints_->push_back(joint_torques_upper);
  constraints_->push_back(friction_cone_);

  // constraints_->push_back(impact_friction_cone);
  // create contact status
  cs_standing_.activateContacts(std::vector<int>({0,1,2,3}));
  cs_12_.activateContacts(std::vector<int>({1,2}));
  cs_01_.activateContacts(std::vector<int>({0,1}));
  cs_03_.activateContacts(std::vector<int>({0,3}));
  cs_23_.activateContacts(std::vector<int>({2,3}));
  cs_123_.activateContacts(std::vector<int>({1,2,3}));
  cs_12_.activateContacts(std::vector<int>({1,2}));
  cs_012_.activateContacts(std::vector<int>({0,1,2}));
  cs_013_.activateContacts(std::vector<int>({0,1,3}));
  cs_023_.activateContacts(std::vector<int>({0,2,3}));

  const double friction_coefficient = 0.5;
  cs_standing_.setFrictionCoefficients(std::vector<double>(4, friction_coefficient));
  cs_12_.setFrictionCoefficients(std::vector<double>(4, friction_coefficient));
  cs_01_.setFrictionCoefficients(std::vector<double>(4, friction_coefficient));
  cs_03_.setFrictionCoefficients(std::vector<double>(4, friction_coefficient));
  cs_23_.setFrictionCoefficients(std::vector<double>(4, friction_coefficient));
  cs_123_.setFrictionCoefficients(std::vector<double>(4, friction_coefficient));
  cs_12_.setFrictionCoefficients(std::vector<double>(4, friction_coefficient));
  cs_012_.setFrictionCoefficients(std::vector<double>(4, friction_coefficient));
  cs_013_.setFrictionCoefficients(std::vector<double>(4, friction_coefficient));
  cs_023_.setFrictionCoefficients(std::vector<double>(4, friction_coefficient));
}


MPCDance::~MPCDance() {
}

void MPCDance::setGaitPattern(const std::shared_ptr<ContactPlannerBase>& foot_step_planner,
    const double CoM_t0, 
    const double LF_t0, 
    const double LH_t0, 
    const double RF_t0, 
    const double RH_t0,
    const std::vector<Eigen::VectorXd>& q_array,
    const std::vector<Eigen::Vector3d>& x3d_LF_array, 
    const std::vector<Eigen::Vector3d>& x3d_LH_array,
    const std::vector<Eigen::Vector3d>& x3d_RF_array, 
    const std::vector<Eigen::Vector3d>& x3d_RH_array,
    const std::vector<bool>& LF_inMotion, 
    const std::vector<bool>& LH_inMotion,
    const std::vector<bool>& RF_inMotion, 
    const std::vector<bool>& RH_inMotion,
    const int size
    ){

  foot_step_planner_ = foot_step_planner;
  CoM_t0_ = CoM_t0;
  LF_t0_ = LF_t0;
  LH_t0_ = LH_t0;
  RF_t0_ = RF_t0;
  RH_t0_ = RH_t0;
  
  q_array_ = q_array;
  x3d_LF_array_ = x3d_LF_array;
  x3d_LH_array_ = x3d_LH_array;
  x3d_RF_array_ = x3d_RF_array;
  x3d_RH_array_ = x3d_RH_array;

  LF_inMotion_ = LF_inMotion;
  LH_inMotion_ = LH_inMotion;
  RF_inMotion_ = RF_inMotion;
  RH_inMotion_ = RH_inMotion;

  size_ = size;
}

void MPCDance::init(const double t, const Eigen::VectorXd& q, 
                         const Eigen::VectorXd& v, 
                         const SolverOptions& solver_options) {
  total_discrete_events_ = 17;
  current_step_ = 0;
  predict_step_ = 0;
  contact_sequence_->reserve(total_discrete_events_); //Sets total number of discrete events to avoid dynamic memory allocation
  contact_sequence_->init(cs_standing_); //initializes with cs_standing
  bool add_step = addStep(t);
  while (add_step) {
    add_step = addStep(t);
  }
  foot_step_planner_->init(q);
  config_cost_->set_q_ref(q);

  com_ref_ = std::make_shared<MPCDanceConfigurationRef>(q0_,q_array_,CoM_t0_,size_);
  com_cost_->set_ref(com_ref_);

  LF_foot_ref_ = std::make_shared<FootRef>(x3d0_LF_,x3d_LF_array_,LF_t0_,size_,LF_inMotion_);
  LH_foot_ref_ = std::make_shared<FootRef>(x3d0_LH_,x3d_LH_array_,LH_t0_,size_,LH_inMotion_);
  RF_foot_ref_ = std::make_shared<FootRef>(x3d0_RF_,x3d_RF_array_,RF_t0_,size_,RF_inMotion_);
  RH_foot_ref_ = std::make_shared<FootRef>(x3d0_RH_,x3d_RH_array_,RH_t0_,size_,RH_inMotion_);

  LF_foot_cost_->set_ref(LF_foot_ref_);
  LH_foot_cost_->set_ref(LH_foot_ref_);
  RF_foot_cost_->set_ref(RF_foot_ref_);
  RH_foot_cost_->set_ref(RH_foot_ref_);

  resetContactPlacements(t, q, v);
  ocp_solver_.setSolution("q", q);
  ocp_solver_.setSolution("v", v);
  ocp_solver_.setSolverOptions(solver_options);
  ocp_solver_.solve(t, q, v, true);
}


void MPCDance::reset() {
  ocp_solver_.setSolution(s_);
}


void MPCDance::reset(const Eigen::VectorXd& q, const Eigen::VectorXd& v) {
  ocp_solver_.setSolution(s_);
  ocp_solver_.setSolution("q", q);
  ocp_solver_.setSolution("v", v);
}


void MPCDance::setSolverOptions(const SolverOptions& solver_options) {
  ocp_solver_.setSolverOptions(solver_options);
}


void MPCDance::updateSolution(const double t, const double dt,
                                   const Eigen::VectorXd& q, 
                                   const Eigen::VectorXd& v) {
  assert(dt > 0);
  const bool add_step = addStep(t);
  const auto ts = contact_sequence_->eventTimes();
  bool remove_step = false;
  if (!ts.empty()) {
    if (ts.front()+eps_ < t+dt) {
      ts_last_ = ts.front();
      contact_sequence_->pop_front();
      remove_step = true;
      ++current_step_;
    }
  }
  resetContactPlacements(t, q, v);
  ocp_solver_.solve(t, q, v, true);
}

const Eigen::VectorXd& MPCDance::getInitialControlInput() const {
  return ocp_solver_.getSolution(0).u;
}


const Solution& MPCDance::getSolution() const {
  return ocp_solver_.getSolution();
}


const aligned_vector<LQRPolicy>& MPCDance::getLQRPolicy() const {
  return ocp_solver_.getLQRPolicy();
}


double MPCDance::KKTError(const double t, const Eigen::VectorXd& q, 
                               const Eigen::VectorXd& v) {
  return ocp_solver_.KKTError(t, q, v);
}


double MPCDance::KKTError() const {
  return ocp_solver_.KKTError();
}



std::vector<std::shared_ptr<TaskSpace3DCost>> MPCDance::getSwingFootCostHandle() {
  std::vector<std::shared_ptr<TaskSpace3DCost>> swing_foot_cost;
  swing_foot_cost = {LF_foot_cost_, LH_foot_cost_, RF_foot_cost_, RH_foot_cost_};
  return swing_foot_cost;
}


std::shared_ptr<ConfigurationSpaceCost> MPCDance::getCoMCostHandle() {
  return com_cost_;
}


std::shared_ptr<Constraints> MPCDance::getConstraintsHandle() {
  return constraints_;
}


std::shared_ptr<FrictionCone> MPCDance::getFrictionConeHandle() {
  return friction_cone_;
}


std::shared_ptr<ContactSequence> MPCDance::getContactSequenceHandle() {
  return contact_sequence_;
}


void MPCDance::setRobotProperties(const RobotProperties& properties) {
  ocp_solver_.setRobotProperties(properties);
}


bool MPCDance::addStep(const double t) {
  float l = 0.4;
  if (predict_step_ == 0){
      float tt = 0.24*l;
      if(tt < t+T_-dtm_){
      contact_sequence_->push_back(cs_123_,tt);
      ++predict_step_;
       return true;
      }
      else{
        return false;
      }
      }
  else if(predict_step_ == 1){
        float tt = 1.24*l;
      if(tt < t+T_-dtm_){
      contact_sequence_->push_back(cs_12_,tt);
      ++predict_step_;
       return true;
      }
      else{
        return false;
      }
      }
  else if(predict_step_ == 2){
        float tt = 1.64*l;
      if(tt < t+T_-dtm_){
      contact_sequence_->push_back(cs_012_,tt);
      ++predict_step_;
       return true;
      }
      else{
        return false;
      }
      }
  else if(predict_step_ == 3){
       float tt = 1.72*l;
      if(tt < t+T_-dtm_){
      contact_sequence_->push_back(cs_01_,tt);
      ++predict_step_;
       return true;
      }
      else{
        return false;
      }
      }
  else if(predict_step_ == 4){
       float tt = 2.32*l;
      if(tt < t+T_-dtm_){
      contact_sequence_->push_back(cs_013_,tt);
      ++predict_step_;
       return true;
      }
      else{
        return false;
      }
      }
  else if(predict_step_ == 5){
      float tt = 2.48*l;
      if(tt < t+T_-dtm_){
      contact_sequence_->push_back(cs_03_,tt);
      ++predict_step_;
       return true;
      }
      else{
        return false;
      }
      }
  else if(predict_step_ == 6){
     float tt = 2.88*l;
      if(tt < t+T_-dtm_){
      contact_sequence_->push_back(cs_023_,tt);
      ++predict_step_;
       return true;
      }
      else{
        return false;
      }
      }
  else if(predict_step_ == 7){
        float tt = 3.04*l;
      if(tt < t+T_-dtm_){
      contact_sequence_->push_back(cs_23_,tt);
      ++predict_step_;
       return true;
      }
      else{
        return false;
      }
      }
  else if(predict_step_ == 8){
        float tt = 3.44*l;
      if(tt < t+T_-dtm_){
      contact_sequence_->push_back(cs_123_,tt);
      ++predict_step_;
       return true;
      }
      else{
        return false;
      }
      }
  else if(predict_step_ == 9){
      float tt = 4*l;
    if(tt < t+T_-dtm_){
    contact_sequence_->push_back(cs_standing_,tt);
    ++predict_step_;
      return true;
    }
    else{
      return false;
    }
    }
    else if(predict_step_ == 10){
      float tt = 4.48*l;
    if(tt < t+T_-dtm_){
    contact_sequence_->push_back(cs_flying_,tt);
    ++predict_step_;
      return true;
    }
    else{
      return false;
    }
    }    
    else if(predict_step_ == 11){
      float tt = 5.04*l;
    if(tt < t+T_-dtm_){
    contact_sequence_->push_back(cs_standing_,tt);
    ++predict_step_;
      return true;
    }
    else{
      return false;
    }
    }    
    else if(predict_step_ == 12){
      float tt = 5.44*l;
    if(tt < t+T_-dtm_){
    contact_sequence_->push_back(cs_flying_,tt);
    ++predict_step_;
      return true;
    }
    else{
      return false;
    }
    }    
    else if(predict_step_ == 13){
      float tt = 6.04*l;
    if(tt < t+T_-dtm_){
    contact_sequence_->push_back(cs_standing_,tt);
    ++predict_step_;
      return true;
    }
    else{
      return false;
    }
    }    
    else if(predict_step_ == 14){
      float tt = 6.6*l;
    if(tt < t+T_-dtm_){
    contact_sequence_->push_back(cs_flying_,tt);
    ++predict_step_;
      return true;
    }
    else{
      return false;
    }
    }    
    else if(predict_step_ == 15){
      float tt = 7*l;
    if(tt < t+T_-dtm_){
    contact_sequence_->push_back(cs_standing_,tt);
    ++predict_step_;
      return true;
    }
    else{
      return false;
    }
    }    
  else{
    return false;
  }
}

void MPCDance::resetContactPlacements(const double t, 
                                           const Eigen::VectorXd& q,
                                           const Eigen::VectorXd& v) {                                                                                                              
  const bool success = foot_step_planner_->plan(t, q, v, contact_sequence_->contactStatus(0),
                                                contact_sequence_->numContactPhases());


  for (int phase=0; phase<contact_sequence_->numContactPhases(); ++phase) {
    contact_sequence_->setContactPlacements(phase, 
                                            foot_step_planner_->contactPositions(phase+1),
                                            foot_step_planner_->contactSurfaces(phase+1));
  }

}

} // namespace robotoc 