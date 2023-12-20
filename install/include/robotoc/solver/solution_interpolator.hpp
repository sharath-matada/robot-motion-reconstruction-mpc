#ifndef ROBOTOC_SOLUTION_INTERPOLATOR_HPP_
#define ROBOTOC_SOLUTION_INTERPOLATOR_HPP_

#include "robotoc/robot/robot.hpp"
#include "robotoc/core/solution.hpp"
#include "robotoc/ocp/time_discretization.hpp"
#include "robotoc/solver/interpolation_order.hpp"
#include "robotoc/utils/numerics.hpp"


namespace robotoc {

///
/// @class SolutionInterpolator
/// @brief Solution interpolator. 
///
class SolutionInterpolator {
public:
  ///
  /// @brief Constructor. 
  /// @param[in] order Order of the interpolation.
  ///
  SolutionInterpolator(const InterpolationOrder order=InterpolationOrder::Linear);

  ///
  /// @brief Default destructor. 
  ///
  ~SolutionInterpolator() = default;

  ///
  /// @brief Default copy constructor. 
  ///
  SolutionInterpolator(const SolutionInterpolator&) = default;

  ///
  /// @brief Default copy assign operator. 
  ///
  SolutionInterpolator& operator=(const SolutionInterpolator&) = default;

  ///
  /// @brief Default move constructor. 
  ///
  SolutionInterpolator(SolutionInterpolator&&) noexcept = default;

  ///
  /// @brief Default move assign operator. 
  ///
  SolutionInterpolator& operator=(SolutionInterpolator&&) noexcept = default;

  void setInterpolationOrder(const InterpolationOrder order);

  ///
  /// @brief Stores the current time discretization and solution. 
  /// @param[in] time_discretization Time discretization. 
  /// @param[out] solution Solution. 
  ///
  void store(const TimeDiscretization& time_discretization,
             const Solution& solution);

  ///
  /// @brief Interpolates the solution. 
  /// @param[in] robot Robot model.
  /// @param[in] time_discretization Time discretization. 
  /// @param[out] solution Solution. 
  ///
  void interpolate(const Robot& robot, 
                   const TimeDiscretization& time_discretization, 
                   Solution& solution) const;

  ///
  /// @brief Check if this has a stored solution. 
  /// @return true if this has a stored solution. 
  ///
  bool hasStoredSolution() const { return has_stored_solution_; }

private:
  InterpolationOrder order_;
  TimeDiscretization stored_time_discretization_;
  Solution stored_solution_;
  bool has_stored_solution_;

  int findStoredGridIndexAtImpactByTime(const double t) const {
    const int N = stored_time_discretization_.size() - 1;
    constexpr double eps = 1.0e-06;
    for (int i=1; i<N; ++i) {
      if ((stored_time_discretization_[i].type == GridType::Impact)
            && (numerics::isApprox(t, stored_time_discretization_[i].t, eps))) {
        return i;
      }
    }
    return -1;
  }

  int findStoredGridIndexAtLiftByTime(const double t) const {
    const int N = stored_time_discretization_.size() - 1;
    constexpr double eps = 1.0e-06;
    for (int i=1; i<N; ++i) {
      if ((stored_time_discretization_[i].type == GridType::Lift)
            && (numerics::isApprox(t, stored_time_discretization_[i].t, eps))) {
        return i;
      }
    }
    return -1;
  }

  int findStoredGridIndexBeforeTime(const double t) const {
    if (t < stored_time_discretization_[0].t) return -1;
    const int N = stored_time_discretization_.size() - 1;
    for (int i=0; i<N; ++i) {
      if (t < stored_time_discretization_[i+1].t) {
        if (stored_time_discretization_[i].type == GridType::Impact) {
          return i+1;
        }
        else {
          return i;
        }
      }
    }
    return N;
  }

  static void interpolate(const Robot& robot, const SplitSolution& s1, 
                          const SplitSolution& s2, const double alpha, 
                          SplitSolution& s);

  static void interpolatePartial(const Robot& robot, const SplitSolution& s1, 
                                 const SplitSolution& s2, const double alpha, 
                                 SplitSolution& s);

  static void initEventSolution(const Robot& robot, const SplitSolution& s1, 
                                const SplitSolution& s2, const double alpha, 
                                SplitSolution& s);

  static void modifyImpactSolution(SplitSolution& s);

  static void modifyTerminalSolution(SplitSolution& s);
};

} // namespace robotoc

#endif // ROBOTOC_SOLUTION_INTERPOLATOR_HPP_