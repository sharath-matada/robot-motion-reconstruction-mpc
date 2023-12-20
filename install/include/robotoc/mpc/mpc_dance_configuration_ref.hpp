#ifndef ROBOTOC_MPC_DANCE_CONFIGURATION_REF_HPP_
#define ROBOTOC_MPC_DANCE_CONFIGURATION_REF_HPP_

#include <vector>
#include <memory>

#include "Eigen/Core"
#include "Eigen/Geometry"

#include "robotoc/robot/robot.hpp"
#include "robotoc/cost/configuration_space_ref_base.hpp"
#include "robotoc/planner/contact_sequence.hpp"
#include "robotoc/mpc/contact_planner_base.hpp"
#include "robotoc/utils/aligned_vector.hpp"


namespace robotoc {
///
/// @class MPCPeriodicConfigurationRef
/// @brief Reference configuration for MPC Dancing. 
///
class MPCDanceConfigurationRef final : public ConfigurationSpaceRefBase {
public:
  ///
  /// @brief Constructor. 
  /// @param[in] q Reference configuration.
  /// @param[in] swing_start_time Start time of the reference tracking.
  /// @param[in] period_active Period where the tracking is active.
  /// @param[in] period_inactive Period where the tracking is inactive.
  /// @param[in] num_phases_in_period Number of phases in a period. Must be 
  /// positive. Default is 1.
  ///
  MPCDanceConfigurationRef(const Eigen::VectorXd& q,
                           const std::vector<Eigen::VectorXd>& q_array, 
                           const double t0, 
                           const double N);

  ///
  /// @brief Destructor. 
  ///
  ~MPCDanceConfigurationRef();

  ///
  /// @brief Set the reference positions of CoM from the contact positions of 
  /// the contact sequence. Also, the CoM refs of the first and last contact 
  /// phases are defined by user.
  /// @param[in] contact_sequence Contact sequence.
  /// @param[in] foot_step_planner Foot step planner.
  ///
  void setConfigurationRef();

  void updateRef(const Robot& robot, const GridInfo& grid_info,
                    Eigen::VectorXd& q_ref) const override;

  bool isActive(const GridInfo& grid_info) const override;


private:
  Eigen::VectorXd q_;
  std::vector<Eigen::VectorXd> q_array_;
  double t0_, N_;
};

} // namespace robotoc

#endif // RROBOTOC_MPC_PERIODIC_CONFIGURATION_REF_HPP_