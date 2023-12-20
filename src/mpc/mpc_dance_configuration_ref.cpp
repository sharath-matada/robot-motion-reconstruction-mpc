#include "robotoc/mpc/mpc_dance_configuration_ref.hpp"

#include <stdexcept>
#include <iostream>
#include <cmath>


namespace robotoc {

MPCDanceConfigurationRef::MPCDanceConfigurationRef(const Eigen::VectorXd& q,
                                                   const std::vector<Eigen::VectorXd>& q_array, 
                                                   const double t0, 
                                                   const double N)
  : ConfigurationSpaceRefBase(),
    q_(q),
    q_array_(q_array),
    t0_(t0),
    N_(N){
    }

MPCDanceConfigurationRef::~MPCDanceConfigurationRef() {
}

void MPCDanceConfigurationRef::setConfigurationRef() {
}

void MPCDanceConfigurationRef::updateRef(const Robot& robot, 
                                               const GridInfo& grid_info,
                                               Eigen::VectorXd& q_ref) const {
   if (grid_info.t < t0_){
    q_ref = q_;
   }
   else{
    int j = (grid_info.t - t0_)/(0.04*0.4);
    if (j>N_){
      j = N_-1;
    }
    q_ref = q_array_[j];
   }
}

bool MPCDanceConfigurationRef::isActive(const GridInfo& grid_info) const {
  // return ((grid_info.t > swing_start_time_) 
  //           && has_inactive_contacts_[grid_info.phase]);
  return true;
}

} // namespace robotoc