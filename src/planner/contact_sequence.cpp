#include "robotoc/planner/contact_sequence.hpp"

#include <stdexcept>
#include <iostream>
#include <cassert>
#include <algorithm>


namespace robotoc {

ContactSequence::ContactSequence(const Robot& robot, 
                                 const int reserved_num_discrete_events)
  : reserved_num_discrete_events_(reserved_num_discrete_events),
    default_contact_status_(robot.createContactStatus()),
    contact_statuses_(2*reserved_num_discrete_events+1),
    impact_events_(reserved_num_discrete_events),
    event_index_impact_(reserved_num_discrete_events), 
    event_index_lift_(reserved_num_discrete_events),
    event_time_(2*reserved_num_discrete_events),
    impact_time_(reserved_num_discrete_events),
    lift_time_(reserved_num_discrete_events),
    is_impact_event_(2*reserved_num_discrete_events),
    sto_impact_(reserved_num_discrete_events), 
    sto_lift_(reserved_num_discrete_events) {
  if (reserved_num_discrete_events < 0) {
    throw std::out_of_range("[ContactSequence] invalid argument: reserved_num_discrete_events must be non-negative!");
  }
  clear();
  contact_statuses_.push_back(default_contact_status_);
}


ContactSequence::ContactSequence()
  : reserved_num_discrete_events_(0),
    default_contact_status_(),
    contact_statuses_(),
    impact_events_(),
    event_index_impact_(), 
    event_index_lift_(),
    event_time_(),
    impact_time_(),
    lift_time_(),
    is_impact_event_(),
    sto_impact_(),
    sto_lift_() {
}


void ContactSequence::init(const ContactStatus& contact_status) {
  clear();
  contact_statuses_.push_back(contact_status);
}


void ContactSequence::push_back(const DiscreteEvent& discrete_event, 
                                const double event_time, const bool sto) {
  if (numContactPhases() == 0) {
    throw std::runtime_error(
        "[ContactSequence] call init() before calling push_back()!");
  }
  if (!discrete_event.existDiscreteEvent()) {
    throw std::runtime_error(
        "[ContactSequence] discrete_event.existDiscreteEvent() must be true!");
  }
  if (discrete_event.preContactStatus() != contact_statuses_.back()) {
    throw std::runtime_error(
        "[ContactSequence] discrete_event.preContactStatus() is not consistent with the last contact status!");
  }
  if (numImpactEvents() > 0 || numLiftEvents() > 0) {
    if (event_time <= event_time_.back()) {
      throw std::runtime_error(
          "[ContactSequence] input event_time (" + std::to_string(event_time) 
          + ") must be larger than the last event time (" 
          + std::to_string(event_time_.back()) + ") !");
    }
  }
  contact_statuses_.push_back(discrete_event.postContactStatus());
  event_time_.push_back(event_time);
  if (discrete_event.existImpact()) {
    impact_events_.push_back(discrete_event);
    event_index_impact_.push_back(contact_statuses_.size()-2);
    impact_time_.push_back(event_time);
    is_impact_event_.push_back(true);
    sto_impact_.push_back(sto);
  }
  else {
    event_index_lift_.push_back(contact_statuses_.size()-2);
    lift_time_.push_back(event_time);
    is_impact_event_.push_back(false);
    sto_lift_.push_back(sto);
  }
  if (reserved_num_discrete_events_ < numDiscreteEvents()) {
    reserved_num_discrete_events_ = numDiscreteEvents();
  }
}


void ContactSequence::push_back(const ContactStatus& contact_status, 
                                const double switching_time, const bool sto) {
  DiscreteEvent discrete_event(contact_statuses_.back(), contact_status);
  push_back(discrete_event, switching_time, sto);
}


void ContactSequence::pop_back() {
  if (numDiscreteEvents() > 0) {
    if (is_impact_event_.back()) {
      impact_events_.pop_back();
      event_index_impact_.pop_back();
      impact_time_.pop_back();
      sto_impact_.pop_back();
    }
    else {
      event_index_lift_.pop_back();
      lift_time_.pop_back();
      sto_lift_.pop_back();
    }
    event_time_.pop_back();
    is_impact_event_.pop_back();
    contact_statuses_.pop_back();
  }
  else if (numContactPhases() > 0) {
    assert(numContactPhases() == 1);
    contact_statuses_.pop_back();
    contact_statuses_.push_back(default_contact_status_);
  }
}


void ContactSequence::pop_front() {
  if (numDiscreteEvents() > 0) {
    if (is_impact_event_.front()) {
      impact_events_.pop_front();
      event_index_impact_.pop_front();
      impact_time_.pop_front();
      sto_impact_.pop_front();
    }
    else {
      event_index_lift_.pop_front();
      lift_time_.pop_front();
      sto_lift_.pop_front();
    }
    event_time_.pop_front();
    is_impact_event_.pop_front();
    contact_statuses_.pop_front();
    for (auto& e : event_index_impact_) { e -= 1; }
    for (auto& e : event_index_lift_) { e -= 1; }
  }
  else if (numContactPhases() > 0) {
    assert(numContactPhases() == 1);
    contact_statuses_.pop_front();
    contact_statuses_.push_back(default_contact_status_);
  }
}


void ContactSequence::setImpactTime(const int impact_index, 
                                     const double impact_time) {
  if (numImpactEvents() <= 0) {
    throw std::runtime_error(
        "[ContactSequence] numImpactEvents() must be positive when calling setImpactTime()!");
  }
  if (impact_index < 0) {
    throw std::runtime_error("[ContactSequence] 'impact_index' must be non-negative!");
  }
  if (impact_index >= numImpactEvents()) {
    throw std::runtime_error(
        "[ContactSequence] input 'impact_index' (" + std::to_string(impact_index) 
        + ") must be less than numImpactEvents() (" 
        + std::to_string(numImpactEvents()) + ") !");
  }
  impact_time_[impact_index] = impact_time;
  event_time_[event_index_impact_[impact_index]] = impact_time;
}


void ContactSequence::setLiftTime(const int lift_index, const double lift_time) {
  if (numLiftEvents() <= 0) {
    throw std::runtime_error(
        "[ContactSequence] numLiftEvents() must be positive when calling setLiftTime()!");
  }
  if (lift_index < 0) {
    throw std::runtime_error("[ContactSequence] 'lift_index' must be non-negative!");
  }
  if (lift_index >= numLiftEvents()) {
    throw std::runtime_error(
        "[ContactSequence] input 'lift_index' (" + std::to_string(lift_index) 
        + ") must be less than numLiftEvents() (" 
        + std::to_string(numLiftEvents()) + ") !");
  }
  lift_time_[lift_index] = lift_time;
  event_time_[event_index_lift_[lift_index]] = lift_time;
}


bool ContactSequence::isSTOEnabledImpact(const int impact_index) const {
  assert(impact_index >= 0);
  assert(impact_index < numImpactEvents());
  return sto_impact_[impact_index];
}


bool ContactSequence::isSTOEnabledLift(const int lift_index) const {
  assert(lift_index >= 0);
  assert(lift_index < numLiftEvents());
  return sto_lift_[lift_index];
}


bool ContactSequence::isEventTimeConsistent() const {
  bool is_consistent = true;
  if (numDiscreteEvents() > 0) {
    for (int event_index=1; event_index<numDiscreteEvents(); ++event_index) {
      if (event_time_[event_index] <= event_time_[event_index-1]) {
        std::cerr << "[ContactSequence] event_time[" + std::to_string(event_index) + "] (" 
                        + std::to_string(event_time_[event_index]) 
                        + ") must be larger than event_time_[" 
                        + std::to_string(event_index-1) + "] (" 
                        + std::to_string(event_time_[event_index-1]) + ") !" << std::endl;
        is_consistent = false;
      }
    }
  }
  return is_consistent;
}


void ContactSequence::setContactPlacements(
    const int contact_phase, 
    const std::vector<Eigen::Vector3d>& contact_positions) {
  if (contact_phase >= numContactPhases()) {
    throw std::runtime_error(
        "[ContactSequence] input 'contact_phase' (" + std::to_string(contact_phase) 
        + ") must be smaller than numContactPhases() (" 
        + std::to_string(numContactPhases()) + ") !");
  }
  contact_statuses_[contact_phase].setContactPlacements(contact_positions);
  if (contact_phase > 0) {
    if (is_impact_event_[contact_phase-1]) {
      for (int impact_index=0; ; ++impact_index) {
        assert(impact_index < numImpactEvents());
        if (event_index_impact_[impact_index] == contact_phase-1) {
          impact_events_[impact_index].setContactPlacements(contact_positions);
          break;
        }
      }
    }
  }
}


void ContactSequence::setContactPlacements(
    const int contact_phase, 
    const std::vector<Eigen::Vector3d>& contact_positions,
    const std::vector<Eigen::Matrix3d>& contact_rotations) {
  if (contact_phase >= numContactPhases()) {
    throw std::runtime_error(
        "[ContactSequence] input 'contact_phase' (" + std::to_string(contact_phase) 
        + ") must be smaller than numContactPhases() (" 
        + std::to_string(numContactPhases()) + ") !");
  }
  contact_statuses_[contact_phase].setContactPlacements(contact_positions, 
                                                        contact_rotations);
  if (contact_phase > 0) {
    if (is_impact_event_[contact_phase-1]) {
      for (int impact_index=0; ; ++impact_index) {
        assert(impact_index < numImpactEvents());
        if (event_index_impact_[impact_index] == contact_phase-1) {
          impact_events_[impact_index].setContactPlacements(contact_positions,
                                                              contact_rotations);
          break;
        }
      }
    }
  }
}


void ContactSequence::setContactPlacements(
    const int contact_phase, const aligned_vector<SE3>& contact_placements) {
  if (contact_phase >= numContactPhases()) {
    throw std::runtime_error(
        "[ContactSequence] input 'contact_phase' (" + std::to_string(contact_phase) 
        + ") must be smaller than numContactPhases() (" 
        + std::to_string(numContactPhases()) + ") !");
  }
  contact_statuses_[contact_phase].setContactPlacements(contact_placements);
  if (contact_phase > 0) {
    if (is_impact_event_[contact_phase-1]) {
      for (int impact_index=0; ; ++impact_index) {
        assert(impact_index < numImpactEvents());
        if (event_index_impact_[impact_index] == contact_phase-1) {
          impact_events_[impact_index].setContactPlacements(contact_placements);
          break;
        }
      }
    }
  }
}


void ContactSequence::setFrictionCoefficients(
    const int contact_phase, const std::vector<double>& friction_coefficient) {
  if (contact_phase >= numContactPhases()) {
    throw std::runtime_error(
        "[ContactSequence] input 'contact_phase' (" + std::to_string(contact_phase) 
        + ") must be smaller than numContactPhases() (" 
        + std::to_string(numContactPhases()) + ") !");
  }
  contact_statuses_[contact_phase].setFrictionCoefficients(friction_coefficient);
  if (contact_phase > 0) {
    if (is_impact_event_[contact_phase-1]) {
      for (int impact_index=0; ; ++impact_index) {
        assert(impact_index < numImpactEvents());
        if (event_index_impact_[impact_index] == contact_phase-1) {
          impact_events_[impact_index].setFrictionCoefficients(friction_coefficient);
          break;
        }
      }
    }
  }
}


template <typename T> 
void reserveDeque(std::deque<T>& deq, const int size) {
  assert(size >= 0);
  if (deq.empty()) {
    deq.resize(size);
  }
  else {
    const int current_size = deq.size();
    if (current_size < size) {
      while (deq.size() < size) {
        deq.push_back(deq.back());
      }
      while (deq.size() > current_size) {
        deq.pop_back();
      }
    }
  }
}


void ContactSequence::reserve(const int reserved_num_discrete_events) {
  if (reserved_num_discrete_events_ < reserved_num_discrete_events) {
    reserveDeque(contact_statuses_, 2*reserved_num_discrete_events+1);
    reserveDeque(impact_events_, reserved_num_discrete_events);
    reserveDeque(event_index_impact_, reserved_num_discrete_events);
    reserveDeque(event_index_lift_, reserved_num_discrete_events);
    reserveDeque(event_time_, 2*reserved_num_discrete_events);
    reserveDeque(impact_time_, reserved_num_discrete_events);
    reserveDeque(lift_time_, reserved_num_discrete_events);
    reserveDeque(is_impact_event_, 2*reserved_num_discrete_events);
    reserveDeque(sto_impact_, reserved_num_discrete_events);
    reserveDeque(sto_lift_, reserved_num_discrete_events);
    reserved_num_discrete_events_ = reserved_num_discrete_events;
  }
}


int ContactSequence::reservedNumDiscreteEvents() const {
  return reserved_num_discrete_events_;
}


void ContactSequence::clear() {
  contact_statuses_.clear();
  impact_events_.clear();
  event_index_impact_.clear(); 
  event_index_lift_.clear();
  event_time_.clear();
  impact_time_.clear();
  lift_time_.clear();
  is_impact_event_.clear();
  sto_impact_.clear();
  sto_lift_.clear();
}



void ContactSequence::disp(std::ostream& os) const {
  int impact_index = 0;
  int lift_index = 0;
  os << "contact sequence:" << "\n";
  for (int event_index=0; event_index<numDiscreteEvents(); ++event_index) {
    os << "  contact phase: " << event_index << "\n";
    os << contactStatus(event_index) << "\n";
    os << "  event index: " << event_index << ", type: ";
    if (eventType(event_index) == DiscreteEventType::Impact) {
      os << "impact, time: " << impactTime(impact_index) 
         << ", sto: " << std::boolalpha << isSTOEnabledImpact(impact_index) <<  "\n";
      os << impactStatus(impact_index) << "\n";
      ++impact_index;
    }
    else {
      os << "lift, time: " << liftTime(lift_index) 
         << ", sto: " << std::boolalpha << isSTOEnabledLift(lift_index) << "\n";
      ++lift_index;
    }
  }
  os << "  contact phase: " << numDiscreteEvents() << "\n";
  os << contactStatus(numDiscreteEvents()) << std::flush;
}


std::ostream& operator<<(std::ostream& os, 
                         const ContactSequence& contact_sequence) {
  contact_sequence.disp(os);
  return os;
}


std::ostream& operator<<(
    std::ostream& os, 
    const std::shared_ptr<ContactSequence>& contact_sequence) {
  contact_sequence->disp(os);
  return os;
}

} // namespace robotoc 