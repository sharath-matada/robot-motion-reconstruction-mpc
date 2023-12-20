#ifndef ROBOTOC_SPLIT_CONSTRAINTED_RICCATI_FACTORIZATION_HPP_ 
#define ROBOTOC_SPLIT_CONSTRAINTED_RICCATI_FACTORIZATION_HPP_

#include <vector>

#include "Eigen/Core"

#include "robotoc/robot/robot.hpp"
#include "robotoc/robot/impact_status.hpp"

namespace robotoc {

///
/// @class SplitConstrainedRiccatiFactorization
/// @brief Riccati factorization matrix and vector for the switching constraint.
///
class SplitConstrainedRiccatiFactorization {
public:
  ///
  /// @brief Constructs Riccati factorization matrix and vector for the 
  /// switching constraint.
  /// @param[in] robot Robot model. 
  ///
  SplitConstrainedRiccatiFactorization(const Robot& robot);

  ///
  /// @brief Default constructor. 
  ///
  SplitConstrainedRiccatiFactorization();

  ///
  /// @brief Destructor. 
  ///
  ~SplitConstrainedRiccatiFactorization();

  ///
  /// @brief Default copy constructor. 
  ///
  SplitConstrainedRiccatiFactorization(
      const SplitConstrainedRiccatiFactorization&) = default;

  ///
  /// @brief Default copy operator. 
  ///
  SplitConstrainedRiccatiFactorization& operator=(
      const SplitConstrainedRiccatiFactorization&) = default;

  ///
  /// @brief Default move constructor. 
  ///
  SplitConstrainedRiccatiFactorization(
      SplitConstrainedRiccatiFactorization&&) noexcept = default;

  ///
  /// @brief Default move assign operator. 
  ///
  SplitConstrainedRiccatiFactorization& operator=(
      SplitConstrainedRiccatiFactorization&&) noexcept = default;

  void setConstraintDimension(const int dimi=0);

  int dims() const;

  Eigen::Block<Eigen::MatrixXd> DGinv();

  const Eigen::Block<const Eigen::MatrixXd> DGinv() const; 

  Eigen::Block<Eigen::MatrixXd> S();

  const Eigen::Block<const Eigen::MatrixXd> S() const;

  Eigen::Block<Eigen::MatrixXd> Sinv();

  const Eigen::Block<const Eigen::MatrixXd> Sinv() const;

  Eigen::Block<Eigen::MatrixXd> SinvDGinv();

  const Eigen::Block<const Eigen::MatrixXd> SinvDGinv() const; 

  Eigen::MatrixXd Ginv;

  Eigen::MatrixXd DtM;

  Eigen::MatrixXd KtDtM;

  bool isApprox(const SplitConstrainedRiccatiFactorization& other) const;

  bool hasNaN() const;

  ///
  /// @brief Displays the split constrained Riccati factorization onto a ostream.
  ///
  void disp(std::ostream& os) const;

  friend std::ostream& operator<<(
      std::ostream& os, const SplitConstrainedRiccatiFactorization& c_riccati);

private:
  Eigen::MatrixXd DGinv_full_, S_full_, Sinv_full_, SinvDGinv_full_;
  int dimv_, dimx_, dimu_, dims_;

};

} // namespace robotoc

#include "robotoc/riccati/split_constrained_riccati_factorization.hxx"

#endif // ROBOTOC_SPLIT_CONSTRAINTED_RICCATI_FACTORIZATION_HPP_ 