#ifndef IDOCP_CONTACT_DISTANCE_HPP_
#define IDOCP_CONTACT_DISTANCE_HPP_

#include "idocp/robot/robot.hpp"
#include "idocp/ocp/split_solution.hpp"
#include "idocp/ocp/split_direction.hpp"
#include "idocp/constraints/constraint_component_base.hpp"
#include "idocp/constraints/constraint_component_data.hpp"
#include "idocp/ocp/split_kkt_residual.hpp"
#include "idocp/ocp/split_kkt_matrix.hpp"


namespace idocp {

class ContactDistance final : public ConstraintComponentBase {
public:
  ContactDistance(const Robot& robot, const double barrier=1.0e-04,
                  const double fraction_to_boundary_rate=0.995);

  ContactDistance();

  ~ContactDistance();

  // Use default copy constructor.
  ContactDistance(const ContactDistance&) = default;

  // Use default copy coperator.
  ContactDistance& operator=(const ContactDistance&) = default;

  // Use default move constructor.
  ContactDistance(ContactDistance&&) noexcept = default;

  // Use default move assign coperator.
  ContactDistance& operator=(ContactDistance&&) noexcept = default;

  bool useKinematics() const override;

  KinematicsLevel kinematicsLevel() const override;

  void allocateExtraData(ConstraintComponentData& data) const;

  bool isFeasible(Robot& robot, ConstraintComponentData& data, 
                  const SplitSolution& s) const override;

  void setSlackAndDual(Robot& robot, ConstraintComponentData& data, 
                       const SplitSolution& s) const override;

  void augmentDualResidual(Robot& robot, ConstraintComponentData& data, 
                           const double dtau, const SplitSolution& s,
                           SplitKKTResidual& kkt_residual) const override;

  void condenseSlackAndDual(Robot& robot, ConstraintComponentData& data, 
                            const double dtau, const SplitSolution& s,
                            SplitKKTMatrix& kkt_matrix,
                            SplitKKTResidual& kkt_residual) const override;

  void computeSlackAndDualDirection(Robot& robot, ConstraintComponentData& data, 
                                    const SplitSolution& s,
                                    const SplitDirection& d) const override; 

  void computePrimalAndDualResidual(Robot& robot, ConstraintComponentData& data, 
                                    const SplitSolution& s) const override;
  
  int dimc() const override;

private:
  int dimv_, dimc_;
  std::vector<int> contact_frames_;
  double fraction_to_boundary_rate_;

};

} // namespace idocp

#endif // IDOCP_CONTACT_DISTANCE_HPP_ 