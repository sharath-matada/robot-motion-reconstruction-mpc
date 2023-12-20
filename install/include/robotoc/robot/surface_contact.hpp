#ifndef ROBOTOC_SURFACE_CONTACT_HPP_
#define ROBOTOC_SURFACE_CONTACT_HPP_

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
/// @class SurfaceContact
/// @brief Kinematics and dynamic model of a surface contact.
///
class SurfaceContact {
public:
  using Vector6d = Eigen::Matrix<double, 6, 1>;
  using Matrix66d = Eigen::Matrix<double, 6, 6>;
  using Matrix6xd = Eigen::Matrix<double, 6, Eigen::Dynamic>;

  ///
  /// @brief Construct a surface contact model.
  /// @param[in] model The pinocchio model. Before calling this constructor, 
  /// pinocchio model must be initialized, e.g., by pinocchio::buildModel().
  /// @param[in] contact_model_info Info of the point contact model. 
  ///
  SurfaceContact(const pinocchio::Model& model, 
                 const ContactModelInfo& contact_model_info); 

  ///
  /// @brief Default constructor. 
  ///
  SurfaceContact();

  ///
  /// @brief Destructor. 
  ///
  ~SurfaceContact() = default;

  ///
  /// @brief Default copy constructor. 
  ///
  SurfaceContact(const SurfaceContact&) = default;

  ///
  /// @brief Default copy assign operator. 
  ///
  SurfaceContact& operator=(const SurfaceContact&) = default;

  ///
  /// @brief Default move constructor. 
  ///
  SurfaceContact(SurfaceContact&&) noexcept = default;

  ///
  /// @brief Default move assign operator. 
  ///
  SurfaceContact& operator=(SurfaceContact&&) noexcept = default;

  ///
  /// @brief Converts the 6D contact wrench in the local coordinate to the 
  /// corresponding joint spatial forces.
  /// @param[in] contact_wrench The 6D contact wrench in the local frame.
  /// @param[out] joint_forces: The corresponding joint spatial forces.
  ///
  void computeJointForceFromContactWrench(
      const Vector6d& contact_wrench, 
      pinocchio::container::aligned_vector<pinocchio::Force>& joint_forces) const;

  ///
  /// @brief Computes the residual of the contact constraints considered by the 
  /// Baumgarte's stabilization method. Before calling this function, kinematics 
  /// of the robot model (frame position, velocity, and acceleration) must be 
  /// updated.
  /// @param[in] model Pinocchio model of the robot.
  /// @param[in] data Pinocchio data of the robot kinematics.
  /// @param[in] desired_contact_placement Desired contact placement.
  /// @param[out] baumgarte_residual Residual of the Bamgarte's constraint. 
  /// Size must be 6.
  /// 
  template <typename VectorType>
  void computeBaumgarteResidual(
      const pinocchio::Model& model, const pinocchio::Data& data, 
      const SE3& desired_contact_placement,
      const Eigen::MatrixBase<VectorType>& baumgarte_residual);

  ///
  /// @brief Computes the partial derivatives of the contact constraints
  /// considered by the Baumgarte's stabilization method. Before calling this 
  /// function, kinematics derivatives of the robot model (position, velocity, 
  /// and acceleration) must be updated.
  /// @param[in] model Pinocchio model of the robot.
  /// @param[in] data Pinocchio data of the robot kinematics.
  /// @param[out] baumgarte_partial_dq The partial derivative with respect to 
  /// the configuaration. Size must be 6 x Robot::dimv().
  /// @param[out] baumgarte_partial_dv The partial derivative with respect to 
  /// the velocity. Size must be 6 x Robot::dimv().
  /// @param[out] baumgarte_partial_da The partial derivative  with respect to 
  /// the acceleration. Size must be 6 x Robot::dimv().
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
  /// configuaration. Size must be 6 x Robot::dimv().
  /// @param[out] velocity_partial_dv The partial derivative with respect to the 
  /// velocity. Size must be 6 x Robot::dimv().
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
  /// @param[in] desired_contact_placement Desired contact placement.
  /// @param[out] position_residual Residual of the contact constraint. Size must 
  /// be 6.
  /// 
  template <typename VectorType>
  void computeContactPositionResidual(
      const pinocchio::Model& model, const pinocchio::Data& data, 
      const SE3& desired_contact_placement,
      const Eigen::MatrixBase<VectorType>& position_residual);

  ///
  /// @brief Computes the partial derivative of the contact position  
  /// constraint with respect to the configuration. Before calling this 
  /// function, kinematics derivative of the robot model with respect to the 
  /// frame position must be updated.
  /// @param[in] model Pinocchio model of the robot.
  /// @param[in] data Pinocchio data of the robot kinematics.
  /// @param[out] position_partial_dq The result of the partial derivative  
  /// with respect to the configuaration. Size must be 6 x Robot::dimv().
  /// 
  template <typename MatrixType>
  void computeContactPositionDerivative(
      const pinocchio::Model& model, pinocchio::Data& data,
      const Eigen::MatrixBase<MatrixType>& position_partial_dq);

  ///
  /// @brief Returns the contact placement at the current kinematics of the  
  /// robot. Before calling this function, kinematics of the robot model (frame 
  /// position) must be updated.
  /// @param[in] data Pinocchio data of the robot kinematics.
  /// @return Const reference to the contact placmenet.
  ///
  const SE3& contactPlacement(const pinocchio::Data& data) const;

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
  /// @brief Displays the surface contact onto a ostream.
  ///
  void disp(std::ostream& os) const;

  friend std::ostream& operator<<(std::ostream& os, 
                                  const SurfaceContact& surface_contact);

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

private:
  ContactModelInfo info_;
  int contact_frame_id_, parent_joint_id_, dimv_;
  pinocchio::SE3 jXf_, X_diff_;
  Matrix6xd J_frame_, frame_v_partial_dq_, frame_a_partial_dq_, 
            frame_a_partial_dv_, frame_a_partial_da_;
  Matrix66d Jlog6_;
};

} // namespace robotoc

#include "robotoc/robot/surface_contact.hxx"

#endif // ROBOTOC_SURFACE_CONTACT_HPP_ 