#include "robotoc/sto/switching_time_optimization.hpp"

#include <cassert>


namespace robotoc {

SwitchingTimeOptimization::SwitchingTimeOptimization(const OCP& ocp) 
  : sto_cost_(ocp.sto_cost), 
    sto_constraints_(ocp.sto_constraints),
    contact_sequence_(ocp.contact_sequence),
    sto_reg_(0),
    is_sto_enabled_((ocp.sto_cost && ocp.sto_constraints)),
    performance_index_(),
    lt_(),
    h_(),
    dts_(),
    Qtt_(),
    constraint_data_() {
}


SwitchingTimeOptimization::SwitchingTimeOptimization() 
  : sto_cost_(), 
    sto_constraints_(),
    sto_reg_(0),
    is_sto_enabled_(false),
    performance_index_(),
    lt_(),
    h_(),
    dts_(),
    Qtt_(),
    constraint_data_() {
}



void SwitchingTimeOptimization::setRegularization(const double sto_reg) {
  sto_reg_ = sto_reg;
}


void SwitchingTimeOptimization::initConstraints(
    const TimeDiscretization& time_discretization) {
  if (!is_sto_enabled_) return;

  constraint_data_ = sto_constraints_->createConstraintsData(time_discretization);
  sto_constraints_->setSlackAndDual(time_discretization, constraint_data_);
}


bool SwitchingTimeOptimization::isFeasible(
    const TimeDiscretization& time_discretization) {
  if (!is_sto_enabled_) return true;

  return sto_constraints_->isFeasible(time_discretization, constraint_data_);
}


void SwitchingTimeOptimization::evalSTO(
    const TimeDiscretization& time_discretization) {
  if (!is_sto_enabled_) return;

  const int N = time_discretization.size() - 1;
  const int num_discrete_events = time_discretization.grid(N).phase 
                                    - time_discretization.grid(0).phase;
  if (num_discrete_events == 0)  {
    performance_index_.setZero();
    return;
  }

  performance_index_.cost = sto_cost_->evalCost(time_discretization);
  sto_constraints_->evalConstraint(time_discretization, constraint_data_);
  performance_index_.cost_barrier = constraint_data_.log_barrier;
  performance_index_.primal_feasibility = constraint_data_.primalFeasibility();
}


void SwitchingTimeOptimization::evalKKT(
    const TimeDiscretization& time_discretization,
    KKTMatrix& kkt_matrix, KKTResidual& kkt_residual) {
  if (!is_sto_enabled_) return;

  const int N = time_discretization.size() - 1;
  const int num_discrete_events = time_discretization.grid(N).phase 
                                    - time_discretization.grid(0).phase;
  if (num_discrete_events == 0)  {
    performance_index_.setZero();
    return;
  }

  performance_index_.setZero();
  lt_.setZero(num_discrete_events);
  Qtt_.setZero(num_discrete_events, num_discrete_events);
  Qtt_.diagonal().fill(sto_reg_);
  performance_index_.cost = sto_cost_->quadratizeCost(time_discretization, lt_, Qtt_);
  sto_constraints_->linearizeConstraints(time_discretization, constraint_data_, lt_);
  performance_index_.cost_barrier = constraint_data_.log_barrier;
  performance_index_.primal_feasibility = constraint_data_.primalFeasibility();
  performance_index_.dual_feasibility = constraint_data_.dualFeasibility();
  performance_index_.kkt_error = constraint_data_.KKTError();
  performance_index_.cost_barrier = constraint_data_.log_barrier;
  sto_constraints_->condenseSlackAndDual(constraint_data_, lt_, Qtt_);

  int event_index = 0;
  for (int i=0; i<N; ++i) {
    const auto& grid = time_discretization[i];
    if (grid.type == GridType::Impact) {
      kkt_residual[i+1].h -= lt_.coeff(event_index);
      kkt_matrix[i+1].Qtt += Qtt_.coeff(event_index, event_index);
      ++event_index;
    }
    else if (grid.type == GridType::Lift) {
      kkt_residual[i].h -= lt_.coeff(event_index);
      kkt_matrix[i].Qtt += Qtt_.coeff(event_index, event_index);
      ++event_index;
    }
  }

  h_.setZero(num_discrete_events+1);
  for (int i=0; i<N; ++i) {
    const auto& grid = time_discretization[i];
    h_.coeffRef(grid.phase) += kkt_residual[i].h;
  }

  event_index = 0;
  for (int i=0; i<N; ++i) {
    const auto& grid = time_discretization[i];
    if ((grid.type == GridType::Impact && time_discretization[i+1].sto)
        || (grid.type == GridType::Lift && grid.sto)) {
      const double hdiff = h_.coeff(event_index) - h_.coeff(event_index+1);
      performance_index_.dual_feasibility += std::abs(hdiff);
      performance_index_.kkt_error += hdiff * hdiff;
      ++event_index;
    }
  }
}


void SwitchingTimeOptimization::computeStepSizes(
    const TimeDiscretization& time_discretization, const Direction& d) {
  if (!is_sto_enabled_) return;

  const int N = time_discretization.size() - 1;
  const int num_discrete_events = time_discretization.grid(N).phase 
                                    - time_discretization.grid(0).phase;
  dts_.resize(num_discrete_events);
  int event_index = 0;
  for (int i=0; i<N; ++i) {
    const auto& grid = time_discretization[i];
    if (grid.type == GridType::Impact || grid.type == GridType::Lift) {
      dts_.coeffRef(event_index) = d[i].dts;
      ++event_index;
    }
  }

  sto_constraints_->expandSlackAndDual(constraint_data_, dts_);
}


double SwitchingTimeOptimization::maxPrimalStepSize() const {
  if (is_sto_enabled_) {
    return sto_constraints_->maxSlackStepSize(constraint_data_);
  }
  else {
    return 1.0;
  }
}


double SwitchingTimeOptimization::maxDualStepSize() const {
  if (is_sto_enabled_) {
    return sto_constraints_->maxDualStepSize(constraint_data_);
  }
  else {
    return 1.0;
  }
}


void SwitchingTimeOptimization::integrateSolution(
    const TimeDiscretization& time_discretization, 
    const double primal_step_size, const double dual_step_size, 
    const Direction& d) {
  if (!is_sto_enabled_) return;

  const int N = time_discretization.size() - 1;
  const int num_discrete_events = time_discretization.grid(N).phase 
                                    - time_discretization.grid(0).phase;
  for (int i=0; i<N; ++i) {
    const auto& grid = time_discretization[i];
    if (grid.type == GridType::Impact) {
      const int impact_index = grid.impact_index;
      const double ts = contact_sequence_->impactTime(impact_index) 
                          + primal_step_size * d[i].dts;
      contact_sequence_->setImpactTime(impact_index, ts);
    }
    else if (grid.type == GridType::Lift) {
      const int lift_index = grid.lift_index;
      const double ts = contact_sequence_->liftTime(lift_index)  
                          + primal_step_size * d[i].dts;
      contact_sequence_->setLiftTime(lift_index, ts);
    }
  }
  sto_constraints_->updateSlack(constraint_data_, primal_step_size);
  sto_constraints_->updateDual(constraint_data_, dual_step_size);
}


const PerformanceIndex& SwitchingTimeOptimization::getEval() const {
  return performance_index_;
}

} // namespace robotoc