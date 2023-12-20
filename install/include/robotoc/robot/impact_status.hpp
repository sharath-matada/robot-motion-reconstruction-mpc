#ifndef ROBOTOC_IMPACT_STATUS_HPP_
#define ROBOTOC_IMPACT_STATUS_HPP_

#include <vector>
#include <iostream>

#include "Eigen/Core"

#include "robotoc/robot/contact_status.hpp"
#include "robotoc/robot/se3.hpp"
#include "robotoc/utils/aligned_vector.hpp"


namespace robotoc {

  ///
  /// @class ImpactStatus
  /// @brief Impact status of robot model. Wrapper of ContactStatus to treat
  /// impacts.
  ///
class ImpactStatus {
public:
  ///
  /// @brief Constructor. 
  /// @param[in] contact_types Types of contacts. 
  /// @param[in] contact_frame_names Names of contact frames. 
  /// @param[in] default_friction_coefficient Default friction coefficitn. 
  /// Must be positive. Default is 0.7.
  ///
  ImpactStatus(const std::vector<ContactType>& contact_types, 
               const std::vector<std::string>& contact_frame_names,
               const double default_friction_coefficient=0.7);

  ///
  /// @brief Default constructor. 
  ///
  ImpactStatus();

  ///
  /// @brief Default destructor. 
  ///
  ~ImpactStatus() = default;

  ///
  /// @brief Default copy constructor. 
  ///
  ImpactStatus(const ImpactStatus&) = default;

  ///
  /// @brief Default copy assign operator. 
  ///
  ImpactStatus& operator=(const ImpactStatus&) = default;

  ///
  /// @brief Default move constructor. 
  ///
  ImpactStatus(ImpactStatus&&) noexcept = default;

  ///
  /// @brief Default move assign operator. 
  ///
  ImpactStatus& operator=(ImpactStatus&&) noexcept = default;

  ///
  /// @brief Defines a comparison operator. 
  ///
  bool operator==(const ImpactStatus& other) const;

  ///
  /// @brief Defines a comparison operator. 
  ///
  bool operator!=(const ImpactStatus& other) const;

  ///
  /// @brief Returns the type of the contact.
  /// @param[in] contact_index Index of a contact of interedted. 
  /// @return Contact type. 
  ///
  ContactType contactType(const int contact_index) const;

  ///
  /// @brief Returns the types of the contacts.
  /// @return Contact types. 
  ///
  const std::vector<ContactType>& contactTypes() const;

  ///
  /// @brief Returns true if a contact is active and false if not.
  /// @param[in] contact_index Index of a contact of interedted. 
  /// @return true if a contact is active and false if not. 
  ///
  bool isImpactActive(const int contact_index) const;

  ///
  /// @brief Returns the activity of the impacts.
  /// @return Const reference to the activity of the impacts. 
  ///
  const std::vector<bool>& isImpactActive() const;

  ///
  /// @brief Returns true if there are active impacts and false if not.
  /// @return true if there are active impacts and false if not. 
  ///
  bool hasActiveImpact() const;

  ///
  /// @brief Returns the dimension of the active impact forces.
  /// @return Dimension of the active impact forces.  
  ///
  int dimf() const;

  ///
  /// @brief Returns the maximum number of the contacts.
  /// @return The maximum number of the contacts. 
  ///
  int maxNumContacts() const;

  ///
  /// @brief Activates a impact.
  /// @param[in] contact_index Index of the contact that is activated.
  ///
  void activateImpact(const int contact_index);

  ///
  /// @brief Deactivates a impact.
  /// @param[in] contact_index Index of the contact that is deactivated.
  ///
  void deactivateImpact(const int contact_index);

  ///
  /// @brief Activates impacts.
  /// @param[in] impact_indices Indices of the impacts that are activated.
  ///
  void activateImpacts(const std::vector<int>& impact_indices);

  ///
  /// @brief Deactivates impacts.
  /// @param[in] impact_indices Indices of the impacts that are deactivated.
  ///
  void deactivateImpacts(const std::vector<int>& impact_indices);

  ///
  /// @brief Sets a contact placement, that is, the position and rotation of 
  /// the contact. The contact rotation is set to Eigen::Matrix3d::Identity(), 
  /// which represents the vertical direction to the ground. For the point 
  /// contacts, the rotation is only used in the friction cone constraints.
  /// For the surface contacts, the rotation represents the rotational contact
  /// constraints on the contact frame of the robot.
  /// @param[in] contact_index Index of the contact.
  /// @param[in] contact_position Contact position.
  ///
  void setContactPlacement(const int contact_index, 
                           const Eigen::Vector3d& contact_position);

  ///
  /// @brief Sets a contact placement, that is, the position and rotation of 
  /// the contact. For the point contacts, the rotation is only used in the 
  /// friction cone constraints.
  /// For the surface contacts, the rotation represents the rotational contact
  /// constraints on the contact frame of the robot.
  /// @param[in] contact_index Index of the contact.
  /// @param[in] contact_position Contact position.
  /// @param[in] contact_rotation Contact rotation.
  ///
  void setContactPlacement(const int contact_index, 
                           const Eigen::Vector3d& contact_position, 
                           const Eigen::Matrix3d& contact_rotation);

  ///
  /// @brief Sets a contact placement, that is, the position and rotation of 
  /// the contact. For the point contacts, the rotation is only used in the 
  /// friction cone constraints.
  /// For the surface contacts, the rotation represents the rotational contact
  /// constraints on the contact frame of the robot.
  /// @param[in] contact_index Index of the contact.
  /// @param[in] contact_placement Contact placement.
  ///
  void setContactPlacement(const int contact_index, 
                           const SE3& contact_placement);

  ///
  /// @brief Sets contact placements. The rotation of each contact is set to
  /// Eigen::Matrix3d::Identity(), which represents the vertical direction
  /// to the ground.
  /// @param[in] contact_positions Contact positions. Size must be 
  /// ImpactStatus::maxNumContacts().
  ///
  void setContactPlacements(
      const std::vector<Eigen::Vector3d>& contact_positions);

  ///
  /// @brief Sets contact placements.
  /// @param[in] contact_positions Contact positions. Size must be 
  /// ImpactStatus::maxNumContacts().
  /// @param[in] contact_rotations Contact rotations. Size must be 
  /// ImpactStatus::maxNumContacts().
  ///
  void setContactPlacements(
      const std::vector<Eigen::Vector3d>& contact_positions,
      const std::vector<Eigen::Matrix3d>& contact_rotations);

  ///
  /// @brief Sets contact placements.
  /// @param[in] contact_placements Contact placements. Size must be 
  /// ImpactStatus::maxNumContacts().
  ///
  void setContactPlacements(const aligned_vector<SE3>& contact_placements);

  ///
  /// @brief Gets the contact placement.
  /// @param[in] contact_index Index of the contact .
  /// @return const reference to the contact placement. 
  ///
  const SE3& contactPlacement(const int contact_index) const;

  ///
  /// @brief Gets the contact position.
  /// @param[in] contact_index Index of the contact .
  /// @return const reference to the contact position. 
  ///
  const Eigen::Vector3d& contactPosition(const int contact_index) const;

  ///
  /// @brief Gets the contact rotation.
  /// @param[in] contact_index Index of the contact .
  /// @return const reference to the contact rotation. 
  ///
  const Eigen::Matrix3d& contactRotation(const int contact_index) const;

  ///
  /// @brief Gets the contact placements.
  /// @return const reference to the contact placements. 
  ///
  const aligned_vector<SE3>& contactPlacements() const;

  ///
  /// @brief Gets the contact positions.
  /// @return const reference to the contact positions. 
  ///
  const std::vector<Eigen::Vector3d>& contactPositions() const;

  ///
  /// @brief Gets the contact rotations.
  /// @return const reference to the contact rotations. 
  ///
  const std::vector<Eigen::Matrix3d>& contactRotations() const;

  ///
  /// @brief Gets the friction coefficint.
  /// @param[in] contact_index Index of the contact.
  /// @param[in] friction_coefficient Friction coefficient. Must be positive.
  ///
  void setFrictionCoefficient(const int contact_index, 
                              const double friction_coefficient);

  ///
  /// @brief Gets the friction coefficint.
  /// @param[in] contact_frame_name Name of the contact frame.
  /// @param[in] friction_coefficient Friction coefficient. Must be positive.
  ///
  void setFrictionCoefficient(const std::string& contact_frame_name, 
                              const double friction_coefficient);

  ///
  /// @brief Sets the friction coefficints.
  /// @param[in] friction_coefficients Friction coefficients. 
  /// Size must be ContactStatus::maxNumContacts() and each element must be positive.
  ///
  void setFrictionCoefficients(const std::vector<double>& friction_coefficients);

  ///
  /// @brief Sets the friction coefficints.
  /// @param[in] friction_coefficients Friction coefficients. 
  /// Size must be ContactStatus::maxNumContacts() and each element must be positive.
  ///
  void setFrictionCoefficients(
      const std::unordered_map<std::string, double>& friction_coefficients);

  ///
  /// @brief Gets the friction coefficint. Default value is 0.7.
  /// @param[in] contact_index Index of the contact.
  /// @return Friction coefficient of the contact. 
  ///
  double frictionCoefficient(const int contact_index) const;

  ///
  /// @brief Gets the friction coefficint. Default value is 0.7.
  /// @param[in] contact_frame_name Name of the contact frame.
  /// @return Friction coefficient of the contact. 
  ///
  double frictionCoefficient(const std::string& contact_frame_name) const;

  ///
  /// @brief Gets the friction coefficints. Default value is 0.7.
  /// @return Friction coefficients of the contacts. 
  ///
  const std::vector<double>& frictionCoefficients() const;

  ///
  /// @brief Fills impact status randomly.
  ///
  void setRandom();

  ///
  /// @brief Displays the impact status onto a ostream.
  ///
  void disp(std::ostream& os) const;

  friend std::ostream& operator<<(std::ostream& os, 
                                  const ImpactStatus& impact_status);

private:
  ContactStatus contact_status_;

};

} // namespace robotoc

#include "robotoc/robot/impact_status.hxx"

#endif // ROBOTOC_IMPACT_STATUS_HPP_ 