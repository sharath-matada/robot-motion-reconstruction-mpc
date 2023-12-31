#include "robotoc/ocp/time_discretization.hpp"
#include "robotoc/utils/numerics.hpp"

#include <iomanip>


namespace robotoc {

TimeDiscretization::TimeDiscretization(const double T, const int N, 
                                       const int reserved_num_discrete_events) 
  : T_(T),
    N_(N),
    num_grids_(N),
    reserved_num_discrete_events_(reserved_num_discrete_events),
    grid_(N+1+3*reserved_num_discrete_events, GridInfo()), 
    sto_event_(), 
    sto_phase_() {
  if (T <= 0) {
    throw std::out_of_range("[TimeDiscretization] invalid argument: 'T' must be positive!");
  }
  if (N <= 0) {
    throw std::out_of_range("[TimeDiscretization] invalid argument: 'N' must be positive!");
  }
  if (reserved_num_discrete_events < 0) {
    throw std::out_of_range("[TimeDiscretization] invalid argument: 'reserved_num_discrete_events' must be non-negative!");
  }
  sto_event_.reserve(2*reserved_num_discrete_events+2);
  sto_phase_.reserve(2*reserved_num_discrete_events+2);
}


TimeDiscretization::TimeDiscretization()
  : T_(0),
    N_(0),
    num_grids_(0),
    reserved_num_discrete_events_(0),
    grid_(), 
    sto_event_(), 
    sto_phase_() {
}


void TimeDiscretization::discretize(
    const std::shared_ptr<ContactSequence>& contact_sequence, const double t) {
  const int N = N_ + contact_sequence->numLiftEvents() + 2 * contact_sequence->numImpactEvents() + 1;
  if (grid_.size() <=N) {
    grid_.resize(N);
  }
  int next_impact_index = 0;
  int next_lift_index = 0;
  while (next_impact_index<contact_sequence->numImpactEvents()) {
    if (contact_sequence->impactTime(next_impact_index) > t) break;
    ++next_impact_index;
  }
  while (next_lift_index<contact_sequence->numLiftEvents()) {
    if (contact_sequence->liftTime(next_lift_index) > t) break;
    ++next_lift_index;
  }
  const double dt = T_ / N_;
  const double eps = std::sqrt(std::numeric_limits<double>::epsilon());
  const double margin = 0.5 * dt;
  int stage = 0;
  double ti = t;
  while (ti+eps < t+T_) {
    const bool has_next_impact = (next_impact_index < contact_sequence->numImpactEvents());
    const bool has_next_lift = (next_lift_index < contact_sequence->numLiftEvents());
    grid_[stage].t = ti;
    grid_[stage].dt = dt;
    grid_[stage].stage = stage;
    grid_[stage].phase = next_impact_index + next_lift_index;
    grid_[stage].impact_index = next_impact_index - 1;
    grid_[stage].lift_index = next_lift_index - 1;
    grid_[stage].type = GridType::Intermediate;
    if (has_next_impact) {
      const double next_impact_time = contact_sequence->impactTime(next_impact_index);
      if (next_impact_time <= ti+dt+eps && (next_impact_time+margin < t+T_)) {
        grid_[stage].dt = next_impact_time - ti;
        ++stage;
        ++next_impact_index;
        grid_[stage].t = next_impact_time;
        grid_[stage].dt = 0;
        grid_[stage].stage = stage;
        grid_[stage].phase = next_impact_index + next_lift_index;
        grid_[stage].impact_index = next_impact_index - 1;
        grid_[stage].lift_index = next_lift_index - 1;
        grid_[stage].type = GridType::Impact;
        ++stage;
        grid_[stage].t = next_impact_time;
        grid_[stage].dt = std::min(ti+dt, t+T_) - next_impact_time;
        grid_[stage].stage = stage;
        grid_[stage].phase = next_impact_index + next_lift_index;
        grid_[stage].impact_index = next_impact_index - 1;
        grid_[stage].lift_index = next_lift_index - 1;
        grid_[stage].type = GridType::Intermediate;
        if (numerics::isApprox(ti+dt, next_impact_time, eps)) {
          ti += dt;
          grid_[stage].dt = ti + dt - next_impact_time;
        }
      }
    }
    if (has_next_lift) {
      const double next_lift_time = contact_sequence->liftTime(next_lift_index);
      if (next_lift_time <= ti+dt+eps && (next_lift_time+margin < t+T_)) {
        grid_[stage].dt = next_lift_time - ti;
        ++stage;
        ++next_lift_index;
        grid_[stage].t = next_lift_time;
        grid_[stage].dt = std::min(ti+dt, t+T_) - next_lift_time;
        grid_[stage].stage = stage;
        grid_[stage].phase = next_impact_index + next_lift_index;
        grid_[stage].impact_index = next_impact_index - 1;
        grid_[stage].lift_index = next_lift_index - 1;
        grid_[stage].type = GridType::Lift;
        if (numerics::isApprox(ti+dt, next_lift_time, eps)) {
          ti += dt;
          grid_[stage].dt = ti + dt - next_lift_time;
        }
      }
    }
    ++stage;
    ti += dt;
  }
  grid_[stage].t  = t+T_;
  grid_[stage].dt = 0;
  grid_[stage].stage = stage;
  grid_[stage].phase = next_impact_index + next_lift_index;
  grid_[stage].impact_index = next_impact_index - 1;
  grid_[stage].lift_index = next_lift_index - 1;
  grid_[stage].type = GridType::Terminal;
  num_grids_ = stage;

  // set dt_next
  for (int i=0; i<num_grids_; ++i) {
    grid_[i].dt_next = grid_[i+1].dt;
  }
  grid_[num_grids_].dt_next = 0.0;

  // set switching_constraint flag
  for (int i=0; i<num_grids_-1; ++i) {
    grid_[i].switching_constraint = (grid_[i+2].type == GridType::Impact);
  }
  grid_[num_grids_-1].switching_constraint = false;
  grid_[num_grids_].switching_constraint = false;

  // set t0 and disable sto
  for (int i=0; i<=num_grids_; ++i) {
    grid_[i].t0 = t;
    grid_[i].sto = false;
    grid_[i].sto_next = false;
    grid_[i].stage_in_phase = 1;
    grid_[i].num_grids_in_phase = 1;
  }

  // count grids
  int stage_in_phase = 0;
  int phase_start_stage = 0;
  for (int i=0; i<num_grids_; ++i) {
    if (grid_[i].type == GridType::Impact) {
      for (int j=phase_start_stage; j<i; ++j) {
        grid_[j].num_grids_in_phase = stage_in_phase;
      }
      grid_[i].stage_in_phase = 0;
      grid_[i].num_grids_in_phase = 0;
      ++i;
      stage_in_phase = 0;
      phase_start_stage = i;
    }
    else if (grid_[i].type == GridType::Lift) {
      for (int j=phase_start_stage; j<i; ++j) {
        grid_[j].num_grids_in_phase = stage_in_phase;
      }
      stage_in_phase = 0;
      phase_start_stage = i;
    }
    grid_[i].stage_in_phase = stage_in_phase;
    ++stage_in_phase;
  }
  for (int j=phase_start_stage; j<num_grids_; ++j) {
    grid_[j].num_grids_in_phase = stage_in_phase;
  }
  grid_[num_grids_].stage_in_phase = 0;
  grid_[num_grids_].num_grids_in_phase = 0;
}


void TimeDiscretization::correctTimeSteps(
    const std::shared_ptr<ContactSequence>& contact_sequence, const double t) {
  int prev_event_stage = 0;
  double prev_event_time = t;
  for (int i=0; i<num_grids_; ++i) {
    if (grid_[i].type == GridType::Impact) {
      const double event_time = contact_sequence->impactTime(grid_[i+1].impact_index);
      const double dt = (event_time - prev_event_time) / grid_[i-1].num_grids_in_phase;
      for (int j=prev_event_stage; j<=i-1; ++j) {
        grid_[j].t = prev_event_time + (j-prev_event_stage) * dt;
        grid_[j].dt = dt;
      }
      grid_[i].t = event_time; 
      grid_[i].dt = 0.0; 
      prev_event_time = event_time;
      prev_event_stage = i+1;
      ++i;
    }
    else if (grid_[i+1].type == GridType::Lift) {
      const double event_time = contact_sequence->liftTime(grid_[i+1].lift_index);
      const double dt = (event_time - prev_event_time) / grid_[i].num_grids_in_phase;
      for (int j=prev_event_stage; j<=i; ++j) {
        grid_[j].t = prev_event_time + (j-prev_event_stage) * dt;
        grid_[j].dt = dt;
      }
      prev_event_time = event_time;
      prev_event_stage = i+1;
    }
    else if (grid_[i+1].type == GridType::Terminal) {
      const double dt = (t + T_ - prev_event_time) / grid_[i].num_grids_in_phase;
      for (int j=prev_event_stage; j<=i; ++j) {
        grid_[j].t = prev_event_time + (j-prev_event_stage) * dt;
        grid_[j].dt = dt;
      }
    }
  }
  grid_[num_grids_].t = t + T_;
  grid_[num_grids_].dt = 0;

  // set dt_next
  for (int i=0; i<num_grids_; ++i) {
    grid_[i].dt_next = grid_[i+1].dt;
  }
  grid_[num_grids_].dt_next = 0.0;

  // set sto
  sto_event_.clear();
  sto_event_.reserve(grid_[num_grids_].phase-grid_[num_grids_].phase);
  for (int i=0; i<num_grids_; ++i) {
    if (grid_[i].type == GridType::Impact) {
      sto_event_.push_back(
          contact_sequence->isSTOEnabledImpact(grid_[i+1].impact_index));
    }
    else if (grid_[i].type == GridType::Lift) {
      sto_event_.push_back(
          contact_sequence->isSTOEnabledLift(grid_[i+1].lift_index));
    }
  }
  if (sto_event_.empty()) return;

  sto_phase_.clear();
  sto_phase_.reserve(grid_[num_grids_].phase-grid_[num_grids_].phase+2);
  sto_phase_.push_back(sto_event_.front());
  for (int i=1; i<sto_event_.size(); ++i) {
    sto_phase_.push_back(sto_event_[i-1] || sto_event_[i]);
  }
  sto_phase_.push_back(sto_event_.back());
  sto_phase_.push_back(false);

  for (int i=0; i<num_grids_; ++i) {
    const int phase = grid_[i].phase - grid_[0].phase;
    grid_[i].sto = sto_phase_[phase];
    grid_[i].sto_next = sto_phase_[phase+1];
  }
  grid_[num_grids_].sto = false;
  grid_[num_grids_].sto_next = false;
}


void TimeDiscretization::disp(std::ostream& os) const {
  auto gridTypeToString = [](const GridType& type) {
    switch (type)
    {
    case GridType::Intermediate:
      return "Intermediate";
      break;
    case GridType::Impact:
      return "     Impact";
      break;
    case GridType::Lift:
      return "        Lift";
      break;
    case GridType::Terminal:
      return "    Terminal";
      break;
    default:
      return "";
      break;
    }
  };
  os << "Time discretization of optimal control problem (OCP):" << "\n";
  os << "  T: " << T_ << "\n";
  os << "  N: " << N() << "\n";
  os << "  num_grids: " << size() << "\n";
  os << " -------------------------------------------------------------------------------------------------" << "\n";
  os << "  stage |         type | grid count |      t |     dt | phase | impact | lift |  sto  | sto_next |" << "\n";
  os << " -------------------------------------------------------------------------------------------------" << "\n";
  for (int i=0; i<size(); ++i) {
    os << "    " << std::right << std::setw(3) << i << " | "
       << gridTypeToString(grid(i).type) << " | "
       << "       " << std::right << std::setw(3) << grid(i).stage_in_phase << " | " 
       << std::fixed << std::setprecision(4) << grid(i).t << " | " << grid(i).dt
       << " |   " << std::setw(3) << grid(i).phase
       << " |    " << std::setw(3) << grid(i).impact_index
       << " |  " << std::setw(3) << grid(i).lift_index
       << " | " << std::setw(5) << std::boolalpha << grid(i).sto
       << " |   " << std::setw(5) << std::boolalpha << grid(i).sto_next << "  |" << "\n";
  }
  os << " -------------------------------------------------------------------------------------------------" << std::flush;
}


std::ostream& operator<<(std::ostream& os, 
                         const TimeDiscretization& time_discretization) {
  time_discretization.disp(os);
  return os;
}

} // namespace robotoc 