#ifndef ROBOTOC_PERIODIC_REF_6D_HPP_
#define ROBOTOC_PERIODIC_REF_6D_HPP_

#include "Eigen/Core"

#include "robotoc/robot/robot.hpp"
#include "robotoc/robot/se3.hpp"
#include "robotoc/cost/task_space_6d_cost.hpp"


namespace robotoc {

///
/// @class PeriodicCoM6dRef
/// @brief 6D Periodic reference of the swing foot position. 
///
class PeriodicComRef6D : public TaskSpace6DRefBase {
public:
  ///
  /// @brief Constructor. 
  /// @param[in] t0 Start time of the reference tracking.
  ///
  PeriodicComRef6D(const SE3& x6d_ref0, const std::vector<SE3>& x6d_ref_array, const double t0, const int N, const std::vector<bool> inMotion);

  ///
  /// @brief Destructor. 
  ///
  ~PeriodicComRef6D();

  ///
  /// @brief Sets parameters.
  /// @param[in] x6d_ref0 Initial CoM position reference(SE3)
  /// @param[in] t0 Start time of the reference tracking.
  ///
  void setCoMTrackRef(const SE3& x6d_ref0, const double t0);

  void updateRef(const GridInfo& grid_info, SE3& x6d_ref) const override;

  bool isActive(const GridInfo& grid_info) const override;

private:
  std::vector<SE3> x6d_ref_array_;
  SE3 x6d_ref0_;
  double t0_;
  int N_;
  std::vector<bool> inMotion_;

};

} // namespace robotoc

#endif // ROBOTOC_PERIODIC_REF_6D_HPP_