#ifndef IDOCP_PARNMPC_HPP_
#define IDOCP_PARNMPC_HPP_

#include <vector>
#include <memory>

#include "Eigen/Core"

#include "idocp/robot/robot.hpp"
#include "idocp/ocp/contact_sequence.hpp"
#include "idocp/ocp/split_parnmpc.hpp"
#include "idocp/ocp/split_solution.hpp"
#include "idocp/ocp/split_direction.hpp"
#include "idocp/ocp/line_search_filter.hpp"
#include "idocp/cost/cost_function.hpp"
#include "idocp/constraints/constraints.hpp"


namespace idocp {

///
/// @class ParNMPC
/// @brief Optimal control problem solver by ParNMPC algorithm.
///
class ParNMPC {
public:
  ///
  /// @brief Construct ParNMPC solver.
  /// @param[in] robot Robot model. Must be initialized by URDF or XML.
  /// @param[in] cost Shared ptr to the cost function.
  /// @param[in] constraints Shared ptr to the constraints.
  /// @param[in] T Length of the horizon. Must be positive.
  /// @param[in] N Number of discretization of the horizon. Must be more than 1. 
  /// @param[in] nthreads Number of the threads in solving the OCP. Must be 
  /// positive. Default is 1.
  ///
  ParNMPC(const Robot& robot, const std::shared_ptr<CostFunction>& cost,
          const std::shared_ptr<Constraints>& constraints, 
          const double T, const int N, const int nthreads=1);

  ///
  /// @brief Default constructor. Does not construct any datas. 
  ///
  ParNMPC();

  ///
  /// @brief Destructor. 
  ///
  ~ParNMPC();

  ///
  /// @brief Use default copy constructor. 
  ///
  ParNMPC(const ParNMPC&) = default;

  ///
  /// @brief Use default copy assign operator. 
  ///
  ParNMPC& operator=(const ParNMPC&) = default;

  ///
  /// @brief Use default move constructor. 
  ///
  ParNMPC(ParNMPC&&) noexcept = default;

  ///
  /// @brief Use default move assign operator. 
  ///
  ParNMPC& operator=(ParNMPC&&) noexcept = default;

  ///
  /// @brief Updates solution by computing the primal-dual Newon direction 
  /// approximately.
  /// @param[in] t Initial time of the horizon. Current time in MPC. 
  /// @param[in] q Initial configuration. Size must be Robot::dimq().
  /// @param[in] v Initial velocity. Size must be Robot::dimv().
  /// @param[in] use_line_search If true, line search is utilized. If false, it
  /// is not utilized. Default is true.
  ///
  void updateSolution(const double t, const Eigen::VectorXd& q, 
                      const Eigen::VectorXd& v, 
                      const bool use_line_search=true);

  ///
  /// @brief Get the contorl input torques of a time stage.
  /// @param[in] stage Time stage of interest. Must be more than 0 and less 
  /// than N-1.
  /// @param[out] u The control input torques. Size must be Robot::dimv().
  ///
  void getControlInput(const int stage, Eigen::VectorXd& u) const;

  ///
  /// @brief Gets the state-feedback gain for the control input torques.
  /// @param[in] stage Time stage of interest. Must be more than 0 and less 
  /// than N-1.
  /// @param[out] Kq Gain with respec to the configuration. Size must be 
  /// Robot::dimv() x Robot::dimv().
  /// @param[out] Kv Gain with respec to the velocity. Size must be
  /// Robot::dimv() x Robot::dimv().
  ///
  void getStateFeedbackGain(const int stage, Eigen::MatrixXd& Kq, 
                            Eigen::MatrixXd& Kv) const;

  ///
  /// @brief Sets the configuration and velocity over the horizon uniformly. 
  /// @param[in] q Configuration. Size must be Robot::dimq().
  /// @param[in] v Velocity. Size must be Robot::dimv().
  ///
  bool setStateTrajectory(const Eigen::VectorXd& q, const Eigen::VectorXd& v);

  ///
  /// @brief Sets the configuration and velocity over the horizon by linear 
  //// interpolation. 
  /// @param[in] q0 Initial configuration. Size must be Robot::dimq().
  /// @param[in] v0 Initial velocity. Size must be Robot::dimv().
  /// @param[in] qN Terminal configuration. Size must be Robot::dimq().
  /// @param[in] vN Terminal velocity. Size must be Robot::dimv().
  ///
  bool setStateTrajectory(const Eigen::VectorXd& q0, const Eigen::VectorXd& v0,
                          const Eigen::VectorXd& qN, const Eigen::VectorXd& vN);

  ///
  /// @brief Sets the auxiliary matrix by the hessian of the terminal cost. 
  /// The hessian is computed based on the current solution.
  ///
  void setAuxiliaryMatrixGuessByTerminalCost(const double t);

  ///
  /// @brief Activate a contact over specified time steps 
  /// (from time_stage_begin until time_stage_end). 
  /// @param[in] contact_index Index of a contact of interedted. 
  /// @param[in] time_stage_begin Beginning of time stages to activate. 
  /// @param[in] time_stage_end End of time stages to activate. 
  ///
  void activateContact(const int contact_index, const int time_stage_begin, 
                       const int time_stage_end);

  ///
  /// @brief Deactivate a contact over specified time steps 
  /// (from time_stage_begin until time_stage_end). 
  /// @param[in] contact_index Index of a contact of interedted. 
  /// @param[in] time_stage_begin Beginning of time stages to deactivate. 
  /// @param[in] time_stage_end End of time stages to deactivate. 
  ///
  void deactivateContact(const int contact_index, const int time_stage_begin, 
                         const int time_stage_end);

  ///
  /// @brief Activate contacts over specified time steps 
  /// (from time_stage_begin until time_stage_end). 
  /// @param[in] contact_indices Indices of contacts of interedted. 
  /// @param[in] time_stage_begin Beginning of time stages to activate. 
  /// @param[in] time_stage_end End of time stages to activate. 
  ///
  void activateContacts(const std::vector<int>& contact_indices, 
                        const int time_stage_begin, const int time_stage_end);

  ///
  /// @brief Deactivate contacts over specified time steps 
  /// (from time_stage_begin until time_stage_end). 
  /// @param[in] contact_indices Indices of contacts of interedted. 
  /// @param[in] time_stage_begin Beginning of time stages to deactivate. 
  /// @param[in] time_stage_end End of time stages to deactivate. 
  ///
  void deactivateContacts(const std::vector<int>& contact_indices, 
                          const int time_stage_begin, const int time_stage_end);

  ///
  /// @brief Sets the contact points over the horizon. 
  /// @param[in] contact_points Contact points over the horizon.
  ///
  void setContactPoint(const std::vector<Eigen::Vector3d>& contact_points);

  ///
  /// @brief Sets the contact points over the horizon by the configuration. 
  /// @param[in] q configuration. Size must be Robot::dimq().
  ///
  void setContactPointByKinematics(const Eigen::VectorXd& q);
  
  ///
  /// @brief Clear the line search filter. 
  ///
  void clearLineSearchFilter();

  ///
  /// @brief Returns the squared KKT error norm by using previously computed 
  /// results computed by updateSolution(). The result is not exactly the 
  /// same as the squared KKT error norm of the original OCP. The result is the
  /// squared norm of the condensed residual. However, this variables is 
  /// sufficiently close to the original KKT error norm.
  /// @return The squared norm of the condensed KKT residual.
  ///
  double KKTError();

  ///
  /// @brief Computes and returns the squared KKT error norm of the OCP. 
  /// @param[in] t Current time. 
  /// @param[in] q Initial configuration. Size must be Robot::dimq().
  /// @param[in] v Initial velocity. Size must be Robot::dimv().
  /// @return The squared norm of the kKT residual.
  ///
  void computeKKTResidual(const double t, const Eigen::VectorXd& q, 
                          const Eigen::VectorXd& v);

  ///
  /// @brief Prints the solution into console. 
  ///
  void printSolution() const;

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

private:

  ///
  /// @brief Return true if the current solution is feasible under the 
  /// inequality constraints. Return false if it is not feasible.
  /// @return true if the current solution is feasible under the inequality 
  /// constraints. false if it is not feasible.
  ///
  bool isCurrentSolutionFeasible();

  ///
  /// @brief Initializes the inequality constraints, i.e., set slack variables 
  /// and the Lagrange multipliers of inequality constraints. Based on the 
  /// current solution.
  ///
  void initConstraints();

  std::vector<SplitParNMPC> split_ocps_;
  std::vector<Robot> robots_;
  ContactSequence contact_sequence_;
  LineSearchFilter filter_;
  double T_, dtau_, step_size_reduction_rate_, min_step_size_;
  int N_, nthreads_;
  std::vector<SplitSolution> s_, s_new_;
  std::vector<SplitDirection> d_;
  std::vector<Eigen::MatrixXd> aux_mat_old_;
  Eigen::VectorXd primal_step_sizes_, dual_step_sizes_, costs_, violations_;
};

} // namespace idocp 


#endif // IDOCP_PARNMPC_HPP_