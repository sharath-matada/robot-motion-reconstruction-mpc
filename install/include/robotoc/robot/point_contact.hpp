#ifndef ROBOTOC_POINT_CONTACT_HPP_
#define ROBOTOC_POINT_CONTACT_HPP_

#include "Eigen/Core"
#include "pinocchio/multibody/model.hpp"
#include "pinocchio/multibody/data.hpp"
#include "pinocchio/container/aligned-vector.hpp"
#include "pinocchio/spatial/force.hpp"

#include "robotoc/robot/contact_model_info.hpp"
#include "robotoc/robot/se3.hpp"

#include <iostream>


namespace robotoc {

///
/// @class PointContact
/// @brief Kinematics and dynamic model of a point contact.
///
class PointContact {
public:
  using Matrix6xd = Eigen::Matrix<double, 6, Eigen::Dynamic>;

  ///
  /// @brief Construct a point contact model.
  /// @param[in] model The pinocchio model. Before calling this constructor, 
  /// pinocchio model must be initialized, e.g., by pinocchio::buildModel().
  /// @param[in] contact_model_info Info of the point contact model. 
  ///
  PointContact(const pinocchio::Model& model, 
               const ContactModelInfo& contact_model_info); 

  ///
  /// @brief Default constructor. 
  ///
  PointContact();

  ///
  /// @brief Default destructor. 
  ///
  ~PointContact() = default;

  ///
  /// @brief Default copy constructor. 
  ///
  PointContact(const PointContact&) = default;

  ///
  /// @brief Default copy assign operator. 
  ///
  PointContact& operator=(const PointContact&) = default;

  ///
  /// @brief Default move constructor. 
  ///
  PointContact(PointContact&&) noexcept = default;

  ///
  /// @brief Default move assign operator. 
  ///
  PointContact& operator=(PointContact&&) noexcept = default;

  ///
  /// @brief Converts the 3D contact forces in the local coordinate to the 
  /// corresponding joint spatial forces.
  /// @param[in] contact_force The 3D contact forces in the local frame.
  /// @param[out] joint_forces: The corresponding joint spatial forces.
  ///
  void computeJointForceFromContactForce(
      const Eigen::Vector3d& contact_force, 
      pinocchio::container::aligned_vector<pinocchio::Force>& joint_forces) const;

  ///
  /// @brief Computes the residual of the contact constraints considered by the 
  /// Baumgarte's stabilization method. Before calling this function, kinematics 
  /// of the robot model (frame position, velocity, and acceleration) must be 
  /// updated.
  /// @param[in] model Pinocchio model of the robot.
  /// @param[in] data Pinocchio data of the robot kinematics.
  /// @param[in] desired_contact_position Desired contact position. Size must be 3.
  /// @param[out] baumgarte_residual Residual of the Bamgarte's constraint. 
  /// Size must be 3.
  /// 
  template <typename VectorType1, typename VectorType2>
  void computeBaumgarteResidual(
      const pinocchio::Model& model, const pinocchio::Data& data, 
      const Eigen::MatrixBase<VectorType1>& desired_contact_position,
      const Eigen::MatrixBase<VectorType2>& baumgarte_residual) const;

  ///
  /// @brief Computes the partial derivatives of the contact constraints
  /// considered by the Baumgarte's stabilization method. Before calling this 
  /// function, kinematics derivatives of the robot model (position, velocity, 
  /// and acceleration) must be updated.
  /// @param[in] model Pinocchio model of the robot.
  /// @param[in] data Pinocchio data of the robot kinematics.
  /// @param[out] baumgarte_partial_dq The partial derivative with respect to 
  /// the configuaration. Size must be 3 x Robot::dimv().
  /// @param[out] baumgarte_partial_dv The partial derivative with respect to 
  /// the velocity. Size must be 3 x Robot::dimv().
  /// @param[out] baumgarte_partial_da The partial derivative  with respect to 
  /// the acceleration. Size must be 3 x Robot::dimv().
  /// 
  template <typename MatrixType1, typename MatrixType2, typename MatrixType3>
  void computeBaumgarteDerivatives(
      const pinocchio::Model& model, pinocchio::Data& data, 
      const Eigen::MatrixBase<MatrixType1>& baumgarte_partial_dq, 
      const Eigen::MatrixBase<MatrixType2>& baumgarte_partial_dv, 
      const Eigen::MatrixBase<MatrixType3>& baumgarte_partial_da);

  ///
  /// @brief Computes the residual of the contact velocity constraints.
  /// Before calling this function, kinematics of the robot model 
  /// (frame position and velocity) must be updated.
  /// @param[in] model Pinocchio model of the robot.
  /// @param[in] data Pinocchio data of the robot kinematics.
  /// @param[out] velocity_residual Residual of the contact velocity constraint.
  /// 
  template <typename VectorType>
  void computeContactVelocityResidual(
      const pinocchio::Model& model, const pinocchio::Data& data, 
      const Eigen::MatrixBase<VectorType>& velocity_residual) const;

  ///
  /// @brief Computes the partial derivatives of the contact velocity 
  /// constraints. Before calling this function, kinematics derivatives of the 
  /// robot model (position and velocity) must be updated.
  /// @param[in] model Pinocchio model of the robot.
  /// @param[in] data Pinocchio data of the robot kinematics.
  /// @param[out] velocity_partial_dq The partial derivative with respect to the 
  /// configuaration. Size must be 3 x Robot::dimv().
  /// @param[out] velocity_partial_dv The partial derivative with respect to the 
  /// velocity. Size must be 3 x Robot::dimv().
  ///
  template <typename MatrixType1, typename MatrixType2>
  void computeContactVelocityDerivatives(
      const pinocchio::Model& model, pinocchio::Data& data,
      const Eigen::MatrixBase<MatrixType1>& velocity_partial_dq, 
      const Eigen::MatrixBase<MatrixType2>& velocity_partial_dv);

  ///
  /// @brief Computes the residual of the contact position constraints. Before 
  /// calling this function, kinematics of the robot model (frame position) must 
  /// be updated.
  /// @param[in] model Pinocchio model of the robot.
  /// @param[in] data Pinocchio data of the robot kinematics.
  /// @param[in] desired_contact_position Desired contact position. Size must be 3.
  /// @param[out] position_residual Residual of the contact constraint. Size must 
  /// be 3.
  /// 
  template <typename VectorType1, typename VectorType2>
  void computeContactPositionResidual(
      const pinocchio::Model& model, const pinocchio::Data& data, 
      const Eigen::MatrixBase<VectorType1>& desired_contact_position,
      const Eigen::MatrixBase<VectorType2>& position_residual) const;

  ///
  /// @brief Computes the partial derivative of the contact position  
  /// constraint with respect to the configuration. Before calling this 
  /// function, kinematics derivative of the robot model with respect to the 
  /// frame position must be updated.
  /// @param[in] model Pinocchio model of the robot.
  /// @param[in] data Pinocchio data of the robot kinematics.
  /// @param[out] position_partial_dq The result of the partial derivative  
  /// with respect to the configuaration. Size must be 3 x Robot::dimv().
  /// 
  template <typename MatrixType>
  void computeContactPositionDerivative(
      const pinocchio::Model& model, pinocchio::Data& data,
      const Eigen::MatrixBase<MatrixType>& position_partial_dq);

  ///
  /// @brief Returns the contact position at the current kinematics of the robot. 
  /// Before calling this function, kinematics of the robot model (frame 
  /// position) must be updated.
  /// @param[in] data Pinocchio data of the robot kinematics.
  /// @return Const reference to the contact position.
  ///
  const Eigen::Vector3d& contactPosition(const pinocchio::Data& data) const;

  ///
  /// @brief Sets the gain parameters of the Baumgarte's stabilization method.
  /// @param[in] baumgarte_position_gain The position gain of the Baumgarte's 
  /// stabilization method. Must be non-negative.
  /// @param[in] baumgarte_velocity_gain The velocity gain of the Baumgarte's 
  /// stabilization method. Must be non-negative.
  /// 
  void setBaumgarteGains(const double baumgarte_position_gain, 
                         const double baumgarte_velocity_gain);

  ///
  /// @brief Returns contact frame id, i.e., the index of the contact frame.
  /// @return Contact frame id.
  /// 
  int contactFrameId() const;

  ///
  /// @brief Returns parent joint id, i.e., the index of the parent joint of 
  /// the contact frame.
  /// @return Parent joint id.
  /// 
  int parentJointId() const;

  ///
  /// @brief Gets the contact model info.
  /// @return const reference to the contact model info.
  /// 
  const ContactModelInfo& contactModelInfo() const;

  ///
  /// @brief Displays the point contact onto a ostream.
  ///
  void disp(std::ostream& os) const;

  friend std::ostream& operator<<(std::ostream& os, 
                                  const PointContact& point_contact);

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

private:
  ContactModelInfo info_;
  int contact_frame_id_, parent_joint_id_, dimv_;
  SE3 jXf_;
  pinocchio::Motion v_frame_;
  Eigen::Matrix3d v_linear_skew_, v_angular_skew_;
  Matrix6xd J_frame_, frame_v_partial_dq_, frame_a_partial_dq_, 
            frame_a_partial_dv_, frame_a_partial_da_;
};

} // namespace robotoc

#include "robotoc/robot/point_contact.hxx"

#endif // ROBOTOC_POINT_CONTACT_HPP_