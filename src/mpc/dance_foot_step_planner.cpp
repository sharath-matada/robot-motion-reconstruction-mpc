#include "robotoc/mpc/dance_foot_step_planner.hpp"
#include "robotoc/utils/rotation.hpp"

#include <stdexcept>
#include <iostream>
#include <cassert>


namespace robotoc {

DanceFootStepPlanner::DanceFootStepPlanner(const Robot& quadruped_robot)
  : ContactPlannerBase(),
    robot_(quadruped_robot),
    raibert_heuristic_(),
    vcom_moving_window_filter_(),
    enable_raibert_heuristic_(false),
    LF_foot_id_(quadruped_robot.pointContactFrames()[0]),
    LH_foot_id_(quadruped_robot.pointContactFrames()[1]),
    RF_foot_id_(quadruped_robot.pointContactFrames()[2]),
    RH_foot_id_(quadruped_robot.pointContactFrames()[3]),
    current_step_(0),
    contact_position_ref_(),
    prev_contact_position_(),
    com_ref_(),
    R_(),
    com_to_contact_position_local_(),
    step_length_(Eigen::Vector3d::Zero()),
    yaw_rate_cmd_(0),
    enable_stance_phase_(false) {
  if (quadruped_robot.maxNumPointContacts() < 4) {
    throw std::out_of_range(
        "[DanceFootStepPlanner] invalid argument: 'robot' is not a quadrupedal robot!\n robot.maxNumPointContacts() must be larger than 4!");
  }
  contact_surface_ref_.push_back(
      std::vector<Eigen::Matrix3d>(4, Eigen::Matrix3d::Identity()));
}


DanceFootStepPlanner::DanceFootStepPlanner() {
}


DanceFootStepPlanner::~DanceFootStepPlanner() {
}


void DanceFootStepPlanner::setGaitPattern(const Eigen::Vector3d& step_length, 
                                         const double step_yaw, 
                                         const bool enable_stance_phase) {
  step_length_ = step_length;
  R_yaw_<< std::cos(step_yaw), -std::sin(step_yaw), 0, 
           std::sin(step_yaw), std::cos(step_yaw),  0,
           0, 0, 1; 
  enable_stance_phase_ = enable_stance_phase;
  enable_raibert_heuristic_ = false;
}


void DanceFootStepPlanner::setRaibertGaitPattern(const Eigen::Vector3d& vcom_cmd,
                                                const double yaw_rate_cmd, 
                                                const double swing_time,
                                                const double stance_time,
                                                const double gain) {
  if (swing_time <= 0.0) {
    throw std::out_of_range("[DanceFootStepPlanner] invalid argument: 'swing_time' must be positive!");
  }
  if (stance_time < 0.0) {
    throw std::out_of_range("[DanceFootStepPlanner] invalid argument: 'stance_time' must be non-negative!");
  }
  if (gain <= 0.0) {
    throw std::out_of_range("[DanceFootStepPlanner] invalid argument: 'gain' must be positive!");
  }
  const double period = 2.0 * (swing_time + stance_time);
  raibert_heuristic_.setParameters(period, gain);
  vcom_moving_window_filter_.setParameters(period, 0.1*period);
  vcom_cmd_ = vcom_cmd;
  const double step_yaw = yaw_rate_cmd * (swing_time + stance_time);
  R_yaw_<< std::cos(step_yaw), -std::sin(step_yaw), 0, 
           std::sin(step_yaw),  std::cos(step_yaw), 0,
           0, 0, 1;
  yaw_rate_cmd_ = yaw_rate_cmd;
  enable_stance_phase_ = (stance_time > 0.0);
  enable_raibert_heuristic_ = true;
}

void DanceFootStepPlanner::setContactSurfaces(
    const std::vector<Eigen::Matrix3d>& contact_surfaces) {
  contact_surface_ref_.clear();
  contact_surface_ref_.push_back(contact_surfaces);
}


void DanceFootStepPlanner::setContactSurfaces(
    const std::vector<std::vector<Eigen::Matrix3d>>& contact_surfaces) {
  contact_surface_ref_.clear();
  for (const auto& e : contact_surfaces) {
    contact_surface_ref_.push_back(e);
  }
}


void DanceFootStepPlanner::init(const Eigen::VectorXd& q) {
  Eigen::Matrix3d R = rotation::RotationMatrixFromQuaternion(q.template segment<4>(3));
  rotation::ProjectRotationMatrix(R, rotation::ProjectionAxis::Z);
  robot_.updateFrameKinematics(q);
  com_to_contact_position_local_ = { R.transpose() * (robot_.framePosition(LF_foot_id_)-robot_.CoM()), 
                                     R.transpose() * (robot_.framePosition(LH_foot_id_)-robot_.CoM()),
                                     R.transpose() * (robot_.framePosition(RF_foot_id_)-robot_.CoM()),
                                     R.transpose() * (robot_.framePosition(RH_foot_id_)-robot_.CoM()) };
  contact_position_ref_.clear();
  com_ref_.clear(),
  com_ref_.push_back(robot_.CoM());
  R_.clear();
  R_.push_back(R);
  vcom_moving_window_filter_.clear();
  current_step_ = 0;
}

bool DanceFootStepPlanner::plan(const double t, const Eigen::VectorXd& q,
                               const Eigen::VectorXd& v,
                               const ContactStatus& contact_status,
                               const int planning_steps) {
  assert(planning_steps >= 0);
  std::vector<Eigen::Vector3d> contact_position;
  robot_.updateFrameKinematics(q);
  for (const auto frame : robot_.pointContactFrames()) {
    contact_position.push_back(robot_.framePosition(frame));
  }
  if (contact_status.isContactActive(0) && contact_status.isContactActive(1) 
      && contact_status.isContactActive(2) && contact_status.isContactActive(3)) {
        if(t<0.3){
          current_step_ = 0;
        }
        else if((t>0.3)&&(t<4.5)){
          current_step_ = 10;
        }
        else if((t>4.5)&&(t<5.5)){
          current_step_ = 12;
        }
        else if((t>5.5)&&(t<6.5)){
          current_step_ = 14;
        }
        else if(t>6.5){
          current_step_ = 16;
        }
    }

  else if (contact_status.isContactActive(1) && contact_status.isContactActive(2) && contact_status.isContactActive(3)) {
    if(t<0.23){
        current_step_ = 1;
    }
    else{
        current_step_ = 9;
    }

    }
  else if (contact_status.isContactActive(1) && contact_status.isContactActive(2)) {
        current_step_ = 2;
    }
  else if (contact_status.isContactActive(0) && contact_status.isContactActive(1) && contact_status.isContactActive(2)) {
        current_step_ = 3;
    }
  else if (contact_status.isContactActive(0) && contact_status.isContactActive(1)) {
        current_step_ = 4;
    }
  else if (contact_status.isContactActive(0) && contact_status.isContactActive(1) && contact_status.isContactActive(3)) {
        current_step_ = 5;
    }
  else if (contact_status.isContactActive(0) && contact_status.isContactActive(3)) {
        current_step_ = 6;
    }
  else if (contact_status.isContactActive(0) && contact_status.isContactActive(2) && contact_status.isContactActive(3)) {
        current_step_ = 7;
    }
  else if (contact_status.isContactActive(2) && contact_status.isContactActive(3)) {
        current_step_ = 8;
    }
  else if (contact_status.isContactActive(1) && contact_status.isContactActive(2) && contact_status.isContactActive(3)) {
        current_step_ = 9;
    }
   else if(!contact_status.isContactActive(0) && !contact_status.isContactActive(1) && !contact_status.isContactActive(2) && !contact_status.isContactActive(3)) {
       if(t<5){
          current_step_ = 11;
        }
        else if((t>5)&&(t<6)){
          current_step_ = 13;
        }
        else if(t>6){
          current_step_ = 15;
        }
        }  
  contact_position_ref_.clear();
    for (int step=current_step_; step<=planning_steps+current_step_; ++step) {
      if(step == 0){
        if(step == current_step_){
          // do nothing

        }
        else{
          contact_position[0] = Eigen::Vector3d( 0.1741,0.1308,0);
          contact_position[1] = Eigen::Vector3d(-0.1868,0.1308,0);
          contact_position[2] = Eigen::Vector3d(0.1741,-0.1308,0);
          contact_position[3] = Eigen::Vector3d(-0.1868,-0.1308,0);
        }
      }
      else if(step == 0){
        if(step == current_step_){
          // do nothing

        }
        else{
          contact_position[0] = Eigen::Vector3d( 0.1741,0.1308,0);
          contact_position[1] = Eigen::Vector3d(-0.1868,0.1308,0);
          contact_position[2] = Eigen::Vector3d(0.1741,-0.1308,0);
          contact_position[3] = Eigen::Vector3d(-0.1868,-0.1308,0);
        }
      }
      else if(step == 1){
        if(step == current_step_){
          // do nothing

        }
        else{
          contact_position[0] = Eigen::Vector3d( 0.1741,0.1308,0);
          contact_position[1] = Eigen::Vector3d(-0.1868,0.1308,0);
          contact_position[2] = Eigen::Vector3d(0.1741,-0.1308,0);
          contact_position[3] = Eigen::Vector3d(-0.1868,-0.1308,0);
          
        }
      }
      else if(step == 2){
        if(step == current_step_){
          // do nothing

        }
        else{
          contact_position[0] = Eigen::Vector3d( 0.1741,0.1308,0);
          contact_position[1] = Eigen::Vector3d(-0.1868,0.1308,0);
          contact_position[2] = Eigen::Vector3d(0.1741,-0.1308,0);
          contact_position[3] = Eigen::Vector3d(-0.1868,-0.1308,0);
          
        }
      }
      else if(step == 3){
        if(step == current_step_){
          // do nothing

        }
        else{
          contact_position[0] = Eigen::Vector3d( 0.1741,0.1308,0);
          contact_position[1] = Eigen::Vector3d(-0.1868,0.1308,0);
          contact_position[2] = Eigen::Vector3d(0.1741,-0.1308,0);
          contact_position[3] = Eigen::Vector3d(-0.1868,-0.1308,0);
  
        }
      }
      else if(step == 4){
        if(step == current_step_){
          // do nothing

        }
        else{
          contact_position[0] = Eigen::Vector3d( 0.1741,0.1308,0);
          contact_position[1] = Eigen::Vector3d(-0.1868,0.1308,0);
          contact_position[2] = Eigen::Vector3d(0.1741,-0.1308,0);
          contact_position[3] = Eigen::Vector3d(-0.1868,-0.1308,0);
          
        }
      }
      else if(step == 5){
        if(step == current_step_){
          // do nothing

        }
        else{
          contact_position[0] = Eigen::Vector3d( 0.1741,0.1308,0);
          contact_position[1] = Eigen::Vector3d(-0.1868,0.1308,0);
          contact_position[2] = Eigen::Vector3d(0.1741,-0.1308,0);
          contact_position[3] = Eigen::Vector3d(-0.1868,-0.1308,0);
          
        }
      }
      else if(step == 6){
        if(step == current_step_){
          // do nothing

        }
        else{
          contact_position[0] = Eigen::Vector3d( 0.1741,0.1308,0);
          contact_position[1] = Eigen::Vector3d(-0.1868,0.1308,0);
          contact_position[2] = Eigen::Vector3d(0.1741,-0.1308,0);
          contact_position[3] = Eigen::Vector3d(-0.1868,-0.1308,0);
          
        }
      }
      else if(step == 7){
        if(step == current_step_){
          // do nothing

        }
        else{
          contact_position[0] = Eigen::Vector3d( 0.1741,0.1308,0);
          contact_position[1] = Eigen::Vector3d(-0.1868,0.1308,0);
          contact_position[2] = Eigen::Vector3d(0.1741,-0.1308,0);
          contact_position[3] = Eigen::Vector3d(-0.1868,-0.1308,0);
          
        }
      }
      else if(step == 8){
        if(step == current_step_){
          // do nothing

        }
        else{
          contact_position[0] = Eigen::Vector3d( 0.1741,0.1308,0);
          contact_position[1] = Eigen::Vector3d(-0.1868,0.1308,0);
          contact_position[2] = Eigen::Vector3d(0.1741,-0.1308,0);
          contact_position[3] = Eigen::Vector3d(-0.1868,-0.1308,0);
        }
      }
      else if(step == 9){
         if(step == current_step_){
          // do nothing

        }
        else{
          contact_position[0] = Eigen::Vector3d( 0.1741,0.1308,0);
          contact_position[1] = Eigen::Vector3d(-0.1868,0.1308,0);
          contact_position[2] = Eigen::Vector3d(0.1741,-0.1308,0);
          contact_position[3] = Eigen::Vector3d(-0.1868,-0.1308,0);
          
        }
      }
      else if(step==10){
        //do nothing

      }
      else if(step==11){
        if(step == current_step_){
          // do nothing

        }
        else{
          contact_position[0] = Eigen::Vector3d( 0.1741,0.1308,0);
          contact_position[1] = Eigen::Vector3d(-0.1868,0.1308,0);
          contact_position[2] = Eigen::Vector3d(0.1741,-0.1308,0);
          contact_position[3] = Eigen::Vector3d(-0.1868,-0.1308,0);
          
        }
      }
      else if(step == 12){
        //do nothing
      }

      else if(step == 13){
        if(step == current_step_){
          // do nothing
        }
        else{
          contact_position[0] = Eigen::Vector3d( 0.1741,0.1308,0);
          contact_position[1] = Eigen::Vector3d(-0.1868,0.1308,0);
          contact_position[2] = Eigen::Vector3d(0.1741,-0.1308,0);
          contact_position[3] = Eigen::Vector3d(-0.1868,-0.1308,0);
          
        }
      }
      else if(step == 14){
        //do nothing
      }
      else if(step == 15){
          if(step == current_step_){
          // do nothing

        }
        else{
          contact_position[0] = Eigen::Vector3d( 0.1741,0.1308,0);
          contact_position[1] = Eigen::Vector3d(-0.1868,0.1308,0);
          contact_position[2] = Eigen::Vector3d(0.1741,-0.1308,0);
          contact_position[3] = Eigen::Vector3d(-0.1868,-0.1308,0);
          
        }
          
      }
      contact_position_ref_.push_back(contact_position);
    }
     const int contact_surface_size = contact_surface_ref_.size();
  for (int i=contact_surface_size; i<contact_position_ref_.size(); ++i) {
    contact_surface_ref_.push_back(contact_surface_ref_.back());
  }
  return true;
}

const std::vector<Eigen::Vector3d>& DanceFootStepPlanner::contactPositions(const int step) const {
  return contact_position_ref_[step];
}

const aligned_vector<SE3>& DanceFootStepPlanner::contactPlacements(const int step) const {
  throw std::runtime_error("[DanceFootStepPlanner] runtime error: contactPlacements() is not implemented!");
  return contact_placement_ref_[step];
}


const std::vector<Eigen::Matrix3d>& DanceFootStepPlanner::contactSurfaces(const int step) const {
  return contact_surface_ref_[step];
}


const Eigen::Vector3d& DanceFootStepPlanner::CoM(const int step) const {
  return com_ref_[step];
}
  

const Eigen::Matrix3d& DanceFootStepPlanner::R(const int step) const {
  return R_[step];
}


std::ostream& operator<<(std::ostream& os, 
                         const DanceFootStepPlanner& planner) {
  planner.disp(os);
  return os;
}


std::ostream& operator<<(std::ostream& os, 
                         const std::shared_ptr<DanceFootStepPlanner>& planner) {
  planner->disp(os);
  return os;
}

} // namespace robotoc 