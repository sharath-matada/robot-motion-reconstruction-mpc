#ifndef ROBOTOC_CONSTRAINTS_HPP_
#define ROBOTOC_CONSTRAINTS_HPP_

#include <vector>
#include <memory>

#include "robotoc/robot/robot.hpp"
#include "robotoc/robot/contact_status.hpp"
#include "robotoc/robot/impact_status.hpp"
#include "robotoc/core/split_solution.hpp"
#include "robotoc/core/split_direction.hpp"
#include "robotoc/core/split_kkt_residual.hpp"
#include "robotoc/core/split_kkt_matrix.hpp"
#include "robotoc/constraints/constraint_component_base.hpp"
#include "robotoc/constraints/impact_constraint_component_base.hpp"
#include "robotoc/constraints/constraint_component_data.hpp"
#include "robotoc/constraints/constraints_data.hpp"


namespace robotoc {

///
/// @class Constraints 
/// @brief Stack of the inequality constraints. Composed by constraint 
/// components that inherits ConstraintComponentBase or 
/// ImpactConstraintComponentBase.
///
class Constraints {
public:
   using ConstraintComponentBasePtr = std::shared_ptr<ConstraintComponentBase>;
   using ImpactConstraintComponentBasePtr 
      = std::shared_ptr<ImpactConstraintComponentBase>;

  ///
  /// @brief Constructor. 
  /// @param[in] barrier_param Barrier parameter. Must be positive. Should be small.
  /// Default is 1.0e-03
  /// @param[in] fraction_to_boundary_rule Must be larger than 0 and smaller 
  /// than 1. Should be between 0.9 and 0.995. Default is 0.995.
  ///
  Constraints(const double barrier_param=1.0e-03, 
              const double fraction_to_boundary_rule=0.995);

  ///
  /// @brief Default destructor. 
  ///
  ~Constraints() = default;

  ///
  /// @brief Default copy constructor. 
  ///
  Constraints(const Constraints&) = default;

  ///
  /// @brief Default copy operator. 
  ///
  Constraints& operator=(const Constraints&) = default;

  ///
  /// @brief Default move constructor. 
  ///
  Constraints(Constraints&&) noexcept = default;

  ///
  /// @brief Default move assign operator. 
  ///
  Constraints& operator=(Constraints&&) noexcept = default;

  ///
  /// @brief Appends a constraint component to the cost function.
  /// @param[in, out] constraint_component Shared pointer to the constraint 
  /// component appended to the constraints. The internal barrier parameter and 
  /// the parameter of the fraction-to-boundary rule will be overitten.
  ///
  void push_back(ConstraintComponentBasePtr constraint_component);

  ///
  /// @brief Appends a constraint component to the cost function.
  /// @param[in, out] constraint_component Shared pointer to the constraint 
  /// component appended to the constraints. The internal barrier parameter and 
  /// the parameter of the fraction-to-boundary rule will be overitten.
  ///
  void push_back(ImpactConstraintComponentBasePtr constraint_component);

  ///
  /// @brief Clears constraints by removing all components.
  ///
  void clear();

  ///
  /// @brief Creates ConstraintsData according to robot model and constraint 
  /// components. 
  /// @param[in] robot Robot model.
  /// @param[in] time_stage Time stage. If -1, the impact stage is assumed. 
  /// @return Constraints data.
  ///
  ConstraintsData createConstraintsData(const Robot& robot, 
                                        const int time_stage=-1) const;

  ///
  /// @brief Checks whether the current split solution s is feasible or not. 
  /// @param[in] robot Robot model.
  /// @param[in] contact_status Contact status.
  /// @param[in] data Constraints data. 
  /// @param[in] s Split solution.
  /// @return true if s is feasible. false if not.
  ///
  bool isFeasible(Robot& robot, const ContactStatus& contact_status,
                  ConstraintsData& data, const SplitSolution& s) const;

  ///
  /// @brief Checks whether the current impact split solution s is feasible or 
  /// not. 
  /// @param[in] robot Robot model.
  /// @param[in] impact_status Impact status.
  /// @param[in] data Constraints data. 
  /// @param[in] s Split solution.
  /// @return true if s is feasible. false if not.
  ///
  bool isFeasible(Robot& robot, const ImpactStatus& impact_status, 
                  ConstraintsData& data, const SplitSolution& s) const;

  ///
  /// @brief Sets the slack and dual variables of each constraint components. 
  /// @param[in] robot Robot model.
  /// @param[in] contact_status Contact status.
  /// @param[out] data Constraints data. 
  /// @param[in] s Split solution.
  ///
  void setSlackAndDual(Robot& robot, const ContactStatus& contact_status, 
                       ConstraintsData& data, const SplitSolution& s) const;

  ///
  /// @brief Sets the slack and dual variables of each impact constraint 
  /// components. 
  /// @param[in] robot Robot model.
  /// @param[in] impact_status Impact status.
  /// @param[out] data Constraints data. 
  /// @param[in] s Split solution.
  ///
  void setSlackAndDual(Robot& robot, const ImpactStatus& impact_status, 
                       ConstraintsData& data, 
                       const SplitSolution& s) const;

  ///
  /// @brief Computes the primal residual, residual in the complementary 
  /// slackness, and the log-barrier function of the slack varible.
  /// @brief Computes the primal and dual residuals of the constraints. 
  /// @param[in] robot Robot model.
  /// @param[in] contact_status Contact status.
  /// @param[in] data Constraints data.
  /// @param[in] s Split solution.
  ///
  void evalConstraint(Robot& robot, const ContactStatus& contact_status, 
                      ConstraintsData& data, const SplitSolution& s) const;

  ///
  /// @brief Computes the primal residual, residual in the complementary 
  /// slackness, and the log-barrier function of the slack varible.
  /// @param[in] robot Robot model.
  /// @param[in] impact_status Impact status.
  /// @param[in] data Constraints data.
  /// @param[in] s Impact split solution.
  ///
  void evalConstraint(Robot& robot, const ImpactStatus& impact_status, 
                      ConstraintsData& data, 
                      const SplitSolution& s) const;

  ///
  /// @brief Evaluates the constraints (i.e., calls evalConstraint()) and adds 
  /// the products of the Jacobian of the constraints and Lagrange multipliers.
  /// @param[in] robot Robot model.
  /// @param[in] contact_status Contact status.
  /// @param[in] data Constraints data. 
  /// @param[in] s Split solution.
  /// @param[out] kkt_residual KKT residual.
  ///
  void linearizeConstraints(Robot& robot, const ContactStatus& contact_status, 
                            ConstraintsData& data, const SplitSolution& s, 
                            SplitKKTResidual& kkt_residual) const;

  ///
  /// @brief Evaluates the constraints (i.e., calls evalConstraint()) and adds 
  /// the products of the Jacobian of the constraints and Lagrange multipliers.
  /// @param[in] robot Robot model.
  /// @param[in] impact_status Impact status.
  /// @param[in] data Constraints data. 
  /// @param[in] s Split solution.
  /// @param[out] kkt_residual KKT residual.
  ///
  void linearizeConstraints(Robot& robot, const ImpactStatus& impact_status, 
                            ConstraintsData& data, const SplitSolution& s,
                            SplitKKTResidual& kkt_residual) const;

  ///
  /// @brief Condenses the slack and dual variables. linearizeConstraints() must 
  /// be called before this function.
  /// @param[in] contact_status Contact status.
  /// @param[in] data Constraints data. 
  /// @param[out] kkt_matrix Split KKT matrix. The condensed Hessians are added  
  /// to this data.
  /// @param[out] kkt_residual Split KKT residual. The condensed residual are 
  /// added to this data.
  ///
  void condenseSlackAndDual(const ContactStatus& contact_status, 
                            ConstraintsData& data, SplitKKTMatrix& kkt_matrix, 
                            SplitKKTResidual& kkt_residual) const;

  ///
  /// @brief Condenses the slack and dual variables. linearizeConstraints() must 
  /// be called before this function.
  /// @param[in] impact_status Impact status.
  /// @param[in] data Constraints data.
  /// @param[out] kkt_matrix Impact split KKT matrix. The condensed Hessians   
  /// are added to this data.
  /// @param[out] kkt_residual Impact split KKT residual. The condensed  
  /// residual are added to this data.
  ///
  void condenseSlackAndDual(const ImpactStatus& impact_status, 
                            ConstraintsData& data, 
                            SplitKKTMatrix& kkt_matrix, 
                            SplitKKTResidual& kkt_residual) const;

  ///
  /// @brief Expands the slack and dual, i.e., computes the directions of the 
  /// slack and dual variables from the directions of the primal variables.
  /// @param[in] contact_status Contact status.
  /// @param[in, out] data Constraints data. 
  /// @param[in] d Split direction.
  ///
  void expandSlackAndDual(const ContactStatus& contact_status, 
                          ConstraintsData& data, const SplitDirection& d) const;

  ///
  /// @brief Expands the slack and dual, i.e., computes the directions of the 
  /// slack and dual variables from the directions of the primal variables.
  /// @param[in] impact_status Impact status.
  /// @param[in, out] data Constraints data. 
  /// @param[in] d Impact split direction.
  ///
  void expandSlackAndDual(const ImpactStatus& impact_status, 
                          ConstraintsData& data, 
                          const SplitDirection& d) const;

  ///
  /// @brief Computes and returns the maximum step size by applying 
  /// fraction-to-boundary-rule to the directions of the slack variables.
  /// @param[in] data Constraints data.
  /// @return Maximum step size regarding the slack variables.
  ///
  double maxSlackStepSize(const ConstraintsData& data) const;

  ///
  /// @brief Computes and returns the maximum step size by applying 
  /// fraction-to-boundary-rule to the directions of the dual variables.
  /// @param[in] data Constraints data.
  /// @return Maximum step size regarding the dual variables.
  ///
  double maxDualStepSize(const ConstraintsData& data) const;

  ///
  /// @brief Updates the slack variables according to step_size.
  /// @param[in, out] data Constraints data. 
  /// @param[in] step_size Step size. 
  ///
  static void updateSlack(ConstraintsData& data, const double step_size);

  ///
  /// @brief Updates the dual variables according to step_size.
  /// @param[in, out] data Constraints data. 
  /// @param[in] step_size Step size. 
  ///
  static void updateDual(ConstraintsData& data, const double step_size);

  ///
  /// @brief Sets the barrier parameter for all the constraint components.
  /// @param[in] barrier_param Barrier parameter. Must be positive. Should be small.
  ///
  void setBarrierParam(const double barrier_param);

  ///
  /// @brief Sets the parameter of the fraction-to-boundary-rule for all the 
  /// constraint components.
  /// @param[in] fraction_to_boundary_rule Must be larger than 0 and smaller 
  /// than 1. Should be between 0.9 and 0.995.
  ///
  void setFractionToBoundaryRule(const double fraction_to_boundary_rule);

  ///
  /// @brief Gets the barrier parameter.
  /// @return Barrier parameter. 
  ///
  double getBarrierParam() const;

  ///
  /// @brief Gets the parameter of the fraction-to-boundary-rule. 
  /// @return The parameter of the fraction-to-boundary-rule. 
  ///
  double getFractionToBoundaryRule() const;

private:
  std::vector<ConstraintComponentBasePtr> position_level_constraints_, 
                                          velocity_level_constraints_, 
                                          acceleration_level_constraints_;
  std::vector<ImpactConstraintComponentBasePtr> impact_level_constraints_;
  double barrier_, fraction_to_boundary_rule_;
};

} // namespace robotoc

#endif // ROBOTOC_CONSTRAINTS_HPP_