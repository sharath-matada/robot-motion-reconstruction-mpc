#include "robotoc/mpc/mpc_periodic_com_ref.hpp"

#include <stdexcept>
#include <cmath>


namespace robotoc {

MPCPeriodicCoMRef::MPCPeriodicCoMRef(const double swing_start_time, 
                                     const double period_active, 
                                     const double period_inactive)
  : TimeVaryingCoMRefBase(),
    com_(), 
    has_inactive_contacts_(),
    swing_start_time_(swing_start_time), 
    period_active_(period_active), 
    period_inactive_(period_inactive), 
    period_(period_active+period_inactive) {
}


MPCPeriodicCoMRef::~MPCPeriodicCoMRef() {
}


void MPCPeriodicCoMRef::setPeriod(const double swing_start_time, 
                                  const double period_active, 
                                  const double period_inactive) {
  swing_start_time_ = swing_start_time;
  period_active_ = period_active;
  period_inactive_ = period_inactive;
  period_ = period_active + period_inactive;
}


void MPCPeriodicCoMRef::setCoMRef(
    const std::shared_ptr<ContactSequence>& contact_sequence, 
    const std::shared_ptr<FootStepPlannerBase>& foot_step_planner) {
  has_inactive_contacts_.clear();
  for (int phase=0; phase<contact_sequence->numContactPhases(); ++phase) {
    const auto& contact_status = contact_sequence->contactStatus(phase);
    int num_active_contacts = 0;
    for (int i=0; i<contact_status.maxNumContacts(); ++i) {
      if (contact_status.isContactActive(i)) {
        ++num_active_contacts; 
      }
    }
    has_inactive_contacts_.push_back(
        num_active_contacts < contact_status.maxNumContacts());
  }
  com_ = foot_step_planner->com();
}


void MPCPeriodicCoMRef::update_com_ref(const GridInfo& grid_info,
                                       Eigen::VectorXd& com_ref) const {
  // if (isActive(grid_info)) { 
  if ((grid_info.t > swing_start_time_) 
        && has_inactive_contacts_[grid_info.contact_phase]) {
    const int cycle = std::floor((grid_info.t-swing_start_time_)/period_);
    const double rate = (grid_info.t-swing_start_time_-cycle*period_) / period_active_;
    com_ref = (1.0-rate) * com_[grid_info.contact_phase]
                + rate * com_[grid_info.contact_phase+1];
  }
  else {
    com_ref = com_[grid_info.contact_phase];
  }
}


bool MPCPeriodicCoMRef::isActive(const GridInfo& grid_info) const {
  // return ((grid_info.t > swing_start_time_) 
  //           && has_inactive_contacts_[grid_info.contact_phase]);
  return true;
}

} // namespace robotoc