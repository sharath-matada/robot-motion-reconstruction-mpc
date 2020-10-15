#ifndef IDOCP_CONSTRAINTS_HPP_
#define IDOCP_CONSTRAINTS_HPP_

#include <vector>
#include <memory>

#include "Eigen/Core"

#include "idocp/robot/robot.hpp"
#include "idocp/ocp/split_solution.hpp"
#include "idocp/ocp/split_direction.hpp"
#include "idocp/constraints/constraint_component_base.hpp"
#include "idocp/constraints/constraint_component_data.hpp"
#include "idocp/constraints/constraints_data.hpp"
#include "idocp/ocp/kkt_residual.hpp"
#include "idocp/ocp/kkt_matrix.hpp"


namespace idocp {

///
/// @class Constraints 
/// @brief Stack of the inequality constraints. Composed by constraint 
/// components that inherits ConstraintComponentBase.
///
class Constraints {
public:
  ///
  /// @brief Default constructor. 
  ///
  Constraints();

  ///
  /// @brief Destructor. 
  ///
  ~Constraints();

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
  /// @brief Append a constraint component to the cost function.
  /// @param[in] constraint shared pointer to the constraint component appended 
  /// to the constraints.
  ///
  void push_back(const std::shared_ptr<ConstraintComponentBase>& constraint);

  ///
  /// @brief Clear constraints by removing all components.
  ///
  void clear();

  ///
  /// @brief Check whether the constraints is empty or not.
  /// @return true if the constraints is empty. false if not.
  ///
  bool isEmpty() const;

  ///
  /// @brief Check if the constraints component requres kinematics of robot 
  /// model.
  /// @return true if the constraints component requres kinematics of 
  /// Robot model. false if not.
  ///
  bool useKinematics() const;

  ///
  /// @brief Creates ConstraintsData according to robot model and constraint 
  /// components. 
  /// @param[in] robot Robot model.
  /// @return Constraints data.
  ///
  ConstraintsData createConstraintsData(const Robot& robot, 
                                        const int time_stage) const;

  ///
  /// @brief Check whether the current solution s is feasible or not. 
  /// @param[in] robot Robot model.
  /// @param[in] data Constraints data generated by 
  /// Constraints::createConstraintsData().
  /// @param[in] s Split solution.
  /// @return true if s is feasible. false if not.
  ///
  bool isFeasible(Robot& robot, ConstraintsData& data,
                  const SplitSolution& s) const;

  ///
  /// @brief Set the slack and dual variables of each constraint components. 
  /// @param[in] robot Robot model.
  /// @param[out] data Constraints data generated by 
  /// Constraints::createConstraintsData().
  /// @param[in] dtau Time step.
  /// @param[in] s Split solution.
  ///
  void setSlackAndDual(Robot& robot, ConstraintsData& data, 
                       const double dtau, const SplitSolution& s) const;

  ///
  /// @brief Augment the dual residual of the constraints to the KKT residual 
  /// with respect to the configuration, velocity, acceleration, and contact 
  /// forces.
  /// @param[in] robot Robot model.
  /// @param[in] data Constraints data generated by 
  /// Constraints::createConstraintsData().
  /// @param[in] dtau Time step.
  /// @param[in] s Split solution.
  /// @param[out] kkt_residual KKT residual.
  ///
  void augmentDualResidual(Robot& robot, ConstraintsData& data,
                           const double dtau, const SplitSolution& s,
                           KKTResidual& kkt_residual) const;

  ///const Eigen::VectorXd& u,
  /// @brief Augment the dual residual of the constraints to the KKT residual
  /// with respect to the control input torques.
  /// @param[in] robot Robot model.
  /// @param[in] data Constraints data generated by 
  /// Constraints::createConstraintsData().
  /// @param[in] dtau Time step.
  /// @param[in] u Control input torques. Size must be Robot::dimv().
  /// @param[out] lu KKT residual with respect to the control input torques. 
  /// Size must be Robot::dimv().
  ///
  void augmentDualResidual(const Robot& robot, ConstraintsData& data,
                           const double dtau, const Eigen::VectorXd& u,
                           Eigen::VectorXd& lu) const;

  ///
  /// @brief Consense slack and dual of the constraints and factorize condensed
  /// KKT Hessian and residual with respect to the configuration, velocity, 
  /// acceleration, and contact forces. 
  /// @param[in] robot Robot model.
  /// @param[in] data Constraints data generated by 
  /// Constraints::createConstraintsData(). residual and duality are also 
  /// computed.
  /// @param[in] dtau Time step.
  /// @param[in] s Split solution.
  /// @param[out] kkt_matrix The KKT matrix. The condensed Hessians are added  
  /// to this data.
  /// @param[out] kkt_residual KKT residual. The condensed residual are added 
  /// to this data.
  ///
  void condenseSlackAndDual(Robot& robot, ConstraintsData& data,
                            const double dtau, const SplitSolution& s,
                            KKTMatrix& kkt_matrix, 
                            KKTResidual& kkt_residual) const;

  ///
  /// @brief Consense slack and dual of the constraints and factorize condensed
  /// KKT Hessian and residual with respect to the configuration, velocity, 
  /// acceleration, and contact forces. 
  /// @param[in] robot Robot model.
  /// @param[in, out] data Constraints data generated by 
  /// Constraints::createConstraintsData(). residual and duality are also 
  /// computed.
  /// @param[in] dtau Time step.
  /// @param[in] u Control input torques.
  /// @param[out] Quu The KKT matrix with respect to the control input torques. 
  /// The condensed Hessians are added to this data. Size must be 
  /// Robot::dimv() x Robot::dimv().
  /// @param[out] lu KKT residual with respect to the control input torques. 
  /// The condensed residual are added to this data. Size must be Robot::dimv().
  ///
  void condenseSlackAndDual(const Robot& robot, ConstraintsData& data,
                            const double dtau, const Eigen::VectorXd& u,
                            Eigen::MatrixXd& Quu, Eigen::VectorXd& lu) const;

  ///
  /// @brief Compute directions of slack and dual.
  /// @param[in] robot Robot model.
  /// @param[in, out] data Constraints data generated by 
  /// Constraints::createConstraintsData().
  /// @param[in] dtau Time step.
  /// @param[in] s Split solution.
  /// @param[in] d Split direction.
  ///
  void computeSlackAndDualDirection(Robot& robot, ConstraintsData& data, 
                                    const double dtau, const SplitSolution& s,
                                    const SplitDirection& d) const;

  ///
  /// @brief Compute and returns the maximum step size by applying 
  /// fraction-to-boundary-rule to the direction of slack.
  /// @param[in] data Constraints data generated by 
  /// Constraints::createConstraintsData().
  /// @return Maximum step size of the slack.
  ///
  double maxSlackStepSize(const ConstraintsData& data) const;

  ///
  /// @brief Compute and returns the maximum step size by applying 
  /// fraction-to-boundary-rule to the direction of dual.
  /// @param[in] data Constraints data generated by 
  /// Constraints::createConstraintsData().
  /// @return Maximum step size of the dual.
  ///
  double maxDualStepSize(const ConstraintsData& data) const;

  ///
  /// @brief Updates the slack with step_size.
  /// @param[in, out] data Constraints data generated by 
  /// Constraints::createConstraintsData().
  /// @param[in] step_size Step size. 
  ///
  void updateSlack(ConstraintsData& data, const double step_size) const;

  ///
  /// @brief Updates the dual with step_size.
  /// @param[in, out] data Constraints data generated by 
  /// Constraints::createConstraintsData().
  /// @param[in] step_size Step size. 
  ///
  void updateDual(ConstraintsData& data, const double step_size) const;

  ///
  /// @brief Computes and returns the value of the barrier function for slack 
  /// variables.
  /// @param[in] data Constraints data generated by 
  /// Constraints::createConstraintsData().
  /// @return Value of the barrier function. 
  ///
  double costSlackBarrier(const ConstraintsData& data) const;

  ///
  /// @brief Computes and returns the value of the barrier function for slack 
  /// variables with the step_size.
  /// @param[in] data Constraints data generated by 
  /// Constraints::createConstraintsData().
  /// @param[in] step_size Step size. 
  /// @return Value of the barrier function. 
  ///
  double costSlackBarrier(const ConstraintsData& data, 
                          const double step_size) const;

  ///
  /// @brief Computes the primal and dual residual of the constraints. 
  /// @param[in] robot Robot model.
  /// @param[in] data Constraints data generated by 
  /// Constraints::createConstraintsData().
  /// @param[in] dtau Time step.
  /// @param[in] s Split solution.
  ///
  void computePrimalAndDualResidual(Robot& robot, ConstraintsData& data, 
                                    const double dtau, 
                                    const SplitSolution& s) const;

  ///
  /// @brief Return the L1-norm of the primal residual of the constraints.
  /// before calling this function, Constraints::computePrimalAndDualResidual 
  /// or Constraints::condenseSlackAndDual must be called.
  /// @param[in] data Constraints data generated by 
  /// @return L1 norm of the primal residual and duality of the constraints. 
  ///
  double l1NormPrimalResidual(const ConstraintsData& data) const;

  ///
  /// @brief Returns the squared norm of the primal residual and duality of the 
  /// constraints. Before call this function, 
  /// Constraints::computePrimalAndDualResidual or 
  /// Constraints::condenseSlackAndDual must be called.
  /// @param[in] data Constraints data generated by 
  /// Constraints::createConstraintsData().
  /// @return Squared norm of the primal residual and duality of the constraints. 
  ///
  double squaredNormPrimalAndDualResidual(const ConstraintsData& data) const;

private:
  std::vector<std::shared_ptr<ConstraintComponentBase>> position_level_constraints_, 
                                                        velocity_level_constraints_, 
                                                        acceleration_level_constraints_;

  static void clear_impl(
      std::vector<std::shared_ptr<ConstraintComponentBase>>& constraints);

  static bool isEmpty_impl(
      const std::vector<std::shared_ptr<ConstraintComponentBase>>& constraints);

  static bool useKinematics_impl(
      const std::vector<std::shared_ptr<ConstraintComponentBase>>& constraints);

  static bool isFeasible_impl(
    const std::vector<std::shared_ptr<ConstraintComponentBase>>& constraints,
    Robot& robot, std::vector<ConstraintComponentData>& data, 
    const SplitSolution& s);

  static void setSlackAndDual_impl(
    const std::vector<std::shared_ptr<ConstraintComponentBase>>& constraints,
    Robot& robot, std::vector<ConstraintComponentData>& data, 
    const double dtau, const SplitSolution& s);

  static void augmentDualResidual_impl(
    const std::vector<std::shared_ptr<ConstraintComponentBase>>& constraints,
    Robot& robot, std::vector<ConstraintComponentData>& data, 
    const double dtau, const SplitSolution& s, KKTResidual& kkt_residual);

  static void condenseSlackAndDual_impl(
    const std::vector<std::shared_ptr<ConstraintComponentBase>>& constraints,
    Robot& robot, std::vector<ConstraintComponentData>& data, 
    const double dtau, const SplitSolution& s, KKTMatrix& kkt_matrix, 
    KKTResidual& kkt_residual);

  static void computeSlackAndDualDirection_impl(
    const std::vector<std::shared_ptr<ConstraintComponentBase>>& constraints,
    Robot& robot, std::vector<ConstraintComponentData>& data, 
    const double dtau, const SplitSolution& s, const SplitDirection& d);

  static double maxSlackStepSize_impl(
     const std::vector<std::shared_ptr<ConstraintComponentBase>>& constraints,
     const std::vector<ConstraintComponentData>& data);

  static double maxDualStepSize_impl(
     const std::vector<std::shared_ptr<ConstraintComponentBase>>& constraints,
     const std::vector<ConstraintComponentData>& data);

  static void updateSlack_impl(
     const std::vector<std::shared_ptr<ConstraintComponentBase>>& constraints,
     std::vector<ConstraintComponentData>& data, const double step_size);

  static void updateDual_impl(
     const std::vector<std::shared_ptr<ConstraintComponentBase>>& constraints,
     std::vector<ConstraintComponentData>& data, const double step_size);

  static double costSlackBarrier_impl(
     const std::vector<std::shared_ptr<ConstraintComponentBase>>& constraints,
     const std::vector<ConstraintComponentData>& data);

  static double costSlackBarrier_impl(
     const std::vector<std::shared_ptr<ConstraintComponentBase>>& constraints,
     const std::vector<ConstraintComponentData>& data, 
     const double step_size);

  static void computePrimalAndDualResidual_impl(
     const std::vector<std::shared_ptr<ConstraintComponentBase>>& constraints,
     Robot& robot, std::vector<ConstraintComponentData>& data, 
     const double dtau, const SplitSolution& s);

  static double l1NormPrimalResidual_impl(
     const std::vector<std::shared_ptr<ConstraintComponentBase>>& constraints,
     const std::vector<ConstraintComponentData>& data);

  static double squaredNormPrimalAndDualResidual_impl(
     const std::vector<std::shared_ptr<ConstraintComponentBase>>& constraints,
     const std::vector<ConstraintComponentData>& data);

};

} // namespace idocp

#include "idocp/constraints/constraints.hxx"

#endif // IDOCP_CONSTRAINTS_HPP_