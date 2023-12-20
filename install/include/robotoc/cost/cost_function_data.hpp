#ifndef ROBOTOC_COST_FUNCTION_DATA_HPP_
#define ROBOTOC_COST_FUNCTION_DATA_HPP_

#include "Eigen/Core"

#include "robotoc/robot/robot.hpp"
#include "robotoc/robot/se3.hpp"


namespace robotoc {

///
/// @class CostFunctionData
/// @brief Composed of data used to compute the cost function and its 
/// derivatives. 
///
struct CostFunctionData {
public:
  ///
  /// @brief Constructor. 
  /// @param[in] robot Robot model. 
  ///
  CostFunctionData(const Robot& robot);

  ///
  /// @brief Default constructor. 
  ///
  CostFunctionData();

  ///
  /// @brief Default destructor. 
  ///
  ~CostFunctionData() = default;

  ///
  /// @brief Default copy constructor. 
  ///
  CostFunctionData(const CostFunctionData&) = default;

  ///
  /// @brief Default copy operator. 
  ///
  CostFunctionData& operator=(const CostFunctionData&) = default;

  ///
  /// @brief Default move constructor. 
  ///
  CostFunctionData(CostFunctionData&&) noexcept = default;

  ///
  /// @brief Default move assign operator. 
  ///
  CostFunctionData& operator=(CostFunctionData&&) noexcept = default;

  ///
  /// @brief Vector used for computing the difference of the configurations in
  /// JointSpaceCost. 
  /// Be allocated only when Robot::hasFloatingBase() is true. Then the size 
  /// is Robot::dimv().
  ///
  Eigen::VectorXd qdiff;

  ///
  /// @brief Vector used for computing the time-varying reference configuration 
  /// in ConfigurationCost. 
  /// The size is Robot::dimq().
  ///
  Eigen::VectorXd q_ref;

  ///
  /// @brief Vector used for set the reference position of the end-effector in 
  /// TaskSpace3DCost. 
  /// Be allocated only when CostFunction has TaskSpace3DCost. Then 
  /// the size is 3.
  ///
  Eigen::VectorXd x3d_ref;

  ///
  /// @brief Vector used for computing the difference of the position of the 
  /// end-effector in TaskSpace3DCost. 
  /// Be allocated only when CostFunction has TaskSpace3DCost. Then the size 
  /// is 3.
  ///
  Eigen::VectorXd diff_3d;

  ///
  /// @brief Vector used for computing the difference of the position and 
  /// rotation of the end-effector in TaskSpace6DCost. 
  /// Be allocated only when CostFunction has TaskSpace6DCost. Then the size 
  /// is 6.
  ///
  Eigen::VectorXd diff_6d;

  ///
  /// @brief Vector used for computing the reference SE3 of the end-effector in 
  /// TaskSpace6DCost. 
  ///
  SE3 x6d_ref;

  ///
  /// @brief Vector used for computing the inverse of the reference SE3 of the 
  /// end-effector in TaskSpace6DCost. 
  ///
  SE3 x6d_ref_inv;

  ///
  /// @brief Vector used for computing the difference of the SE3 of the 
  /// end-effector in TaskSpace6DCost. 
  /// Be allocated only when CostFunction has TaskSpace6DCost. 
  ///
  SE3 diff_x6d;

  ///
  /// @brief Jacobian of the difference of the configurations used in 
  /// JointSpaceCost. 
  /// Be allocated only when Robot::hasFloatingBase() is true. Then the size 
  /// is Robot::dimv() x Robot::dimv().
  ///
  Eigen::MatrixXd J_qdiff;

  ///
  /// @brief Jacobian used in TaskSpace3DCost and TaskSpace6DCost.
  /// Be allocated only when CostFunction has TaskSpace3DCost or 
  /// TaskSpace6DCost. Size is 6 x Robot::dimv().
  ///
  Eigen::MatrixXd J_6d;

  ///
  /// @brief Jacobian used in TaskSpace3DCost and TaskSpace6DCost.
  /// Be allocated only when CostFunction has TaskSpace3DCost or 
  /// TaskSpace6DCost. Size is 3 x Robot::dimv().
  ///
  Eigen::MatrixXd J_3d;

  ///
  /// @brief Jacobian used in TaskSpace6DCost.  Be allocated only when 
  /// CostFunction has TaskSpace6DCost. Size is 6 x 6.
  ///
  Eigen::MatrixXd J_66;

  ///
  /// @brief Jacobian used in TaskSpace6DCost. Be allocated only when 
  /// CostFunction has TaskSpace6DCost. Size is 6 x Robot::dimv().
  ///
  Eigen::MatrixXd JJ_6d;

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW 
};

} // namespace robotoc

#endif // ROBOTOC_COST_FUNCTION_DATA_HPP_ 