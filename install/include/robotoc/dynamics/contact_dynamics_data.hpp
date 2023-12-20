#ifndef ROBOTOC_CONTACT_DYNAMICS_DATA_HPP_
#define ROBOTOC_CONTACT_DYNAMICS_DATA_HPP_

#include "Eigen/Core"
#include "robotoc/robot/robot.hpp"


namespace robotoc {

///
/// @class ContactDynamicsData
/// @brief Data used in ContactDynamics.
///
class ContactDynamicsData {
public:
  using Vector6d = Eigen::Matrix<double, 6, 1>;

  ///
  /// @brief Constructs a data.
  /// @param[in] robot Robot model. 
  ///
  ContactDynamicsData(const Robot& robot);

  ///
  /// @brief Default constructor. 
  ///
  ContactDynamicsData();

  ///
  /// @brief Default destructor. 
  ///
  ~ContactDynamicsData() = default;

  ///
  /// @brief Default copy constructor. 
  ///
  ContactDynamicsData(const ContactDynamicsData&) = default;

  ///
  /// @brief Default copy operator. 
  ///
  ContactDynamicsData& operator=(const ContactDynamicsData&) = default;

  ///
  /// @brief Default move constructor. 
  ///
  ContactDynamicsData(ContactDynamicsData&&) noexcept = default;

  ///
  /// @brief Default move assign operator. 
  ///
  ContactDynamicsData& operator=(ContactDynamicsData&&) noexcept = default;

  void setContactDimension(const int dimf);

  void setSwitchingConstraintDimension(const int dims);

  int dimv() const;

  int dimu() const;

  int dimf() const;

  int dimvf() const;

  int dims() const;

  int dim_passive() const;

  bool hasFloatingBase() const;

  Eigen::MatrixXd Qxu_passive;

  Eigen::MatrixXd Quu_passive_topRight;

  Eigen::VectorXd lu_passive;

  Eigen::MatrixXd dIDda;

  Eigen::MatrixXd dIDddv;

  Eigen::Block<Eigen::MatrixXd> dCda();

  const Eigen::Block<const Eigen::MatrixXd> dCda() const;

  Eigen::Block<Eigen::MatrixXd> dIDCdqv();

  const Eigen::Block<const Eigen::MatrixXd> dIDCdqv() const;

  Eigen::Block<Eigen::MatrixXd> dIDdq();

  const Eigen::Block<const Eigen::MatrixXd> dIDdq() const;

  Eigen::Block<Eigen::MatrixXd> dIDdv();

  const Eigen::Block<const Eigen::MatrixXd> dIDdv() const;

  Eigen::Block<Eigen::MatrixXd> dCdq();

  const Eigen::Block<const Eigen::MatrixXd> dCdq() const;

  Eigen::Block<Eigen::MatrixXd> dCdv();

  const Eigen::Block<const Eigen::MatrixXd> dCdv() const;

  Eigen::Block<Eigen::MatrixXd> MJtJinv();

  const Eigen::Block<const Eigen::MatrixXd> MJtJinv() const;

  Eigen::Block<Eigen::MatrixXd> MJtJinv_dIDCdqv();

  const Eigen::Block<const Eigen::MatrixXd> MJtJinv_dIDCdqv() const;

  Eigen::Block<Eigen::MatrixXd> Qafqv();

  const Eigen::Block<const Eigen::MatrixXd> Qafqv() const;

  Eigen::Block<Eigen::MatrixXd> Qdvfqv() { return Qafqv(); }

  const Eigen::Block<const Eigen::MatrixXd> Qdvfqv() const { return Qafqv(); }

  Eigen::Block<Eigen::MatrixXd> Qafu_full();

  const Eigen::Block<const Eigen::MatrixXd> Qafu_full() const;

  Eigen::Block<Eigen::MatrixXd> Qafu_passive();

  const Eigen::Block<const Eigen::MatrixXd> Qafu_passive() const;

  Eigen::Block<Eigen::MatrixXd> Qafu();

  const Eigen::Block<const Eigen::MatrixXd> Qafu() const;

  Eigen::VectorBlock<Eigen::VectorXd> IDC();

  const Eigen::VectorBlock<const Eigen::VectorXd> IDC() const;

  Eigen::VectorBlock<Eigen::VectorXd> ID_full();

  const Eigen::VectorBlock<const Eigen::VectorXd> ID_full() const;

  Eigen::VectorBlock<Eigen::VectorXd> ID_passive();

  const Eigen::VectorBlock<const Eigen::VectorXd> ID_passive() const;

  Eigen::VectorBlock<Eigen::VectorXd> ID();

  const Eigen::VectorBlock<const Eigen::VectorXd> ID() const;

  Eigen::VectorBlock<Eigen::VectorXd> C();

  const Eigen::VectorBlock<const Eigen::VectorXd> C() const;

  Eigen::VectorBlock<Eigen::VectorXd> MJtJinv_IDC();

  const Eigen::VectorBlock<const Eigen::VectorXd> MJtJinv_IDC() const;

  Eigen::Block<Eigen::MatrixXd> Phia();

  const Eigen::Block<const Eigen::MatrixXd> Phia() const;

  Eigen::VectorBlock<Eigen::VectorXd> laf();

  const Eigen::VectorBlock<const Eigen::VectorXd> laf() const;

  Eigen::VectorBlock<Eigen::VectorXd> la();

  const Eigen::VectorBlock<const Eigen::VectorXd> la() const;

  Eigen::VectorBlock<Eigen::VectorXd> ldvf() { return laf(); }

  const Eigen::VectorBlock<const Eigen::VectorXd> ldvf() const { return laf(); }

  Eigen::VectorBlock<Eigen::VectorXd> ldv() { return la(); }

  const Eigen::VectorBlock<const Eigen::VectorXd> ldv() const { return la(); }

  Eigen::VectorBlock<Eigen::VectorXd> lf();

  const Eigen::VectorBlock<const Eigen::VectorXd> lf() const;

  Eigen::VectorBlock<Eigen::VectorXd> haf();

  const Eigen::VectorBlock<const Eigen::VectorXd> haf() const;

  Eigen::VectorBlock<Eigen::VectorXd> ha();

  const Eigen::VectorBlock<const Eigen::VectorXd> ha() const;

  Eigen::VectorBlock<Eigen::VectorXd> hf();

  const Eigen::VectorBlock<const Eigen::VectorXd> hf() const;

  template <int p=1>
  double primalFeasibility() const {
    return IDC().template lpNorm<p>();
  }

  template <int p=1>
  double dualFeasibility() const {
    return lu_passive.template lpNorm<p>();
  }

  double KKTError() const {
    return (IDC().squaredNorm() + lu_passive.squaredNorm());
  }

private:
  Eigen::MatrixXd dCda_full_, dIDCdqv_full_, MJtJinv_full_, 
                  MJtJinv_dIDCdqv_full_, Qafqv_full_, 
                  Qafu_full_full_, Phia_full_;
  Eigen::VectorXd IDC_full_, MJtJinv_IDC_full_, laf_full_, haf_full_;
  int dimv_, dimu_, dimf_, dimvf_, dims_, dim_passive_;
  bool has_floating_base_;

};

} // namespace robotoc 

#include "robotoc/dynamics/contact_dynamics_data.hxx"

#endif // ROBOTOC_CONTACT_DYNAMICS_DATA_HPP_ 