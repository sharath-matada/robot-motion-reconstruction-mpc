#ifndef ROBOTOC_FOOT_REF_HPP_
#define ROBOTOC_FOOT_REF_HPP_

#include "Eigen/Core"

#include "robotoc/robot/robot.hpp"
#include "robotoc/cost/task_space_3d_cost.hpp"


namespace robotoc {

///
/// @class PeriodicCoM6dRef
/// @brief 3D Periodic reference of the swing foot position. 
///
class FootRef : public TaskSpace3DRefBase {
public:
  ///
  /// @brief Constructor. 
  /// @param[in] t0 Start time of the reference tracking.
  ///
  FootRef(const Eigen::Vector3d& x3d_ref0, const std::vector<Eigen::Vector3d>& x3d_ref_array,
                       const double t0, const int N, const std::vector<bool> inMotion);

  ///
  /// @brief Destructor. 
  ///
  ~FootRef();

  ///
  /// @brief Sets parameters.
  /// @param[in] x3d_ref0 Initial CoM position reference(SE3)
  /// @param[in] t0 Start time of the reference tracking.
  ///
  void setCoMTrackRef(const Eigen::VectorXd& x3d_ref0, const double t0);

  void updateRef(const GridInfo& grid_info, Eigen::VectorXd& x3d_ref) const override;

  bool isActive(const GridInfo& grid_info) const override;

private:
  std::vector<Eigen::Vector3d> x3d_ref_array_;
  Eigen::Vector3d x3d_ref0_;
  double t0_;
  int N_;
  const std::vector<bool> inMotion_;

};

} // namespace robotoc

#endif // ROBOTOC_PERIODIC_REF_6D_HPP_