#include "robotoc/core/split_solution.hpp"

#include <random>

namespace robotoc {

SplitSolution::SplitSolution(const Robot& robot) 
  : q(Eigen::VectorXd::Zero(robot.dimq())),
    v(Eigen::VectorXd::Zero(robot.dimv())),
    a(Eigen::VectorXd::Zero(robot.dimv())),
    dv(Eigen::VectorXd::Zero(robot.dimv())),
    u(Eigen::VectorXd::Zero(robot.dimu())),
    f(robot.maxNumContacts(), Vector6d::Zero()),
    lmd(Eigen::VectorXd::Zero(robot.dimv())),
    gmm(Eigen::VectorXd::Zero(robot.dimv())),
    beta(Eigen::VectorXd::Zero(robot.dimv())),
    mu(robot.maxNumContacts(), Vector6d::Zero()),
    nu_passive(Eigen::VectorXd::Zero(robot.dim_passive())),
    f_stack_(Eigen::VectorXd::Zero(robot.max_dimf())),
    mu_stack_(Eigen::VectorXd::Zero(robot.max_dimf())),
    xi_stack_(Eigen::VectorXd::Zero(robot.max_dimf())),
    has_floating_base_(robot.hasFloatingBase()),
    contact_types_(robot.contactTypes()),
    is_contact_active_(robot.maxNumContacts(), false),
    dimf_(0),
    dims_(0),
    max_num_contacts_(robot.maxNumContacts()) {
  if (robot.hasFloatingBase()) {
    q.coeffRef(6) = 1.0;
  }
}


SplitSolution::SplitSolution() 
  : q(),
    v(),
    a(),
    dv(),
    u(),
    f(),
    lmd(),
    gmm(),
    beta(),
    mu(),
    nu_passive(),
    f_stack_(),
    mu_stack_(),
    xi_stack_(),
    has_floating_base_(false),
    contact_types_(),
    is_contact_active_(),
    dimf_(0),
    dims_(0),
    max_num_contacts_(0) {
}


void SplitSolution::integrate(const Robot& robot, const double step_size, 
                              const SplitDirection& d, const bool impact) {
  assert(f_stack().size() == d.df().size());
  assert(mu_stack().size() == d.dmu().size());
  robot.integrateConfiguration(d.dq(), step_size, q);
  v.noalias() += step_size * d.dv();
  if (!impact) {
    a.noalias() += step_size * d.da();
    dv.setZero();
    u.noalias() += step_size * d.du;
  }
  else {
    a.setZero();
    dv.noalias() += step_size * d.ddv();
    u.setZero();
  }
  lmd.noalias() += step_size * d.dlmd();
  gmm.noalias() += step_size * d.dgmm();
  beta.noalias() += step_size * d.dbeta();
  if (has_floating_base_ && !impact) {
    nu_passive.noalias() += step_size * d.dnu_passive;
  }
  if (dimf() > 0) {
    f_stack().noalias() += step_size * d.df();
    set_f_vector();
    mu_stack().noalias() += step_size * d.dmu();
    set_mu_vector();
  }
  if ((dims() > 0) && !impact) {
    assert(xi_stack().size() == d.dxi().size());
    xi_stack().noalias() += step_size * d.dxi();
  }
}


void SplitSolution::copyPrimal(const SplitSolution& other) {
  setContactStatus(other);
  q = other.q;
  v = other.v;
  a = other.a;
  dv = other.dv;
  u = other.u;
  for (int i=0; i<f.size(); ++i) {
    f[i] = other.f[i];
  }
  set_f_stack();
}


void SplitSolution::copyDual(const SplitSolution& other) {
  setContactStatus(other);
  setSwitchingConstraintDimension(other.dims());
  lmd = other.lmd;
  gmm = other.gmm;
  beta = other.beta;
  if (has_floating_base_) {
    nu_passive = other.nu_passive;
  }
  for (int i=0; i<f.size(); ++i) {
    mu[i] = other.mu[i];
  }
  set_mu_stack();
  if (dims() > 0) {
    xi_stack() = other.xi_stack();
  }
}


double SplitSolution::lagrangeMultiplierLinfNorm() const {
  const double lmd_linf = lmd.template lpNorm<Eigen::Infinity>();
  const double gmm_linf = gmm.template lpNorm<Eigen::Infinity>();
  const double beta_linf = beta.template lpNorm<Eigen::Infinity>();
  const double nu_passive_linf = (has_floating_base_ ? nu_passive.template lpNorm<Eigen::Infinity>() : 0);
  const double mu_linf = ((dimf_ > 0) ? mu_stack().template lpNorm<Eigen::Infinity>() : 0);
  const double xi_linf = ((dims_ > 0) ? xi_stack().template lpNorm<Eigen::Infinity>() : 0);
  return std::max({lmd_linf, gmm_linf, beta_linf, nu_passive_linf, mu_linf, xi_linf});
}


bool SplitSolution::isApprox(const SplitSolution& other) const {
  if (!q.isApprox(other.q)) {
    return false;
  }
  if (!v.isApprox(other.v)) {
    return false;
  }
  if (!a.isApprox(other.a)) {
    return false;
  }
  if (!dv.isApprox(other.dv)) {
    return false;
  }
  if (!u.isApprox(other.u)) {
    return false;
  }
  if (!lmd.isApprox(other.lmd)) {
    return false;
  }
  if (!gmm.isApprox(other.gmm)) {
    return false;
  }
  if (!beta.isApprox(other.beta)) {
    return false;
  }
  if (has_floating_base_) {
    if (!nu_passive.isApprox(other.nu_passive)) {
      return false;
    }
  }
  if (dimf() > 0) {
    if (!f_stack().isApprox(other.f_stack())) {
      return false;
    }
    if (!mu_stack().isApprox(other.mu_stack())) {
      return false;
    }
    for (int i=0; i<is_contact_active_.size(); ++i) {
      if (is_contact_active_[i]) {
        if (!other.isContactActive(i)) {
          return false;
        }
        if (!f[i].isApprox(other.f[i])) {
          return false;
        }
        if (!mu[i].isApprox(other.mu[i])) {
          return false;
        }
      }
      else {
        if (other.isContactActive(i)) {
          return false;
        }
      }
    }
  }
  if (dims() > 0) {
    if (!xi_stack().isApprox(other.xi_stack())) {
      return false;
    }
  }
  return true;
}


void SplitSolution::setRandom(const Robot& robot) {
  q.setRandom(); 
  robot.normalizeConfiguration(q);
  v.setRandom();
  a.setRandom(); 
  dv.setRandom(); 
  u.setRandom(); 
  lmd.setRandom();
  gmm.setRandom(); 
  beta.setRandom(); 
  if (robot.hasFloatingBase()) {
    nu_passive.setRandom();
  }
}


void SplitSolution::setRandom(const Robot& robot, 
                              const ContactStatus& contact_status) {
  setContactStatus(contact_status);
  setRandom(robot);
  if (contact_status.hasActiveContacts()) {
    f_stack().setRandom();
    mu_stack().setRandom();
    set_f_vector();
    set_mu_vector();
  }
}


void SplitSolution::setRandom(const Robot& robot, 
                              const ImpactStatus& impact_status) {
  setContactStatus(impact_status);
  setSwitchingConstraintDimension(impact_status.dimf());
  setRandom(robot);
  if (impact_status.hasActiveImpact()) {
    xi_stack().setRandom();
  }
}


void SplitSolution::setRandom(const Robot& robot, 
                              const ContactStatus& contact_status,
                              const ImpactStatus& impact_status) {
  setRandom(robot, contact_status);
  setSwitchingConstraintDimension(impact_status.dimf());
  if (impact_status.hasActiveImpact()) {
    xi_stack().setRandom();
  }
}


SplitSolution SplitSolution::Random(const Robot& robot) {
  SplitSolution s(robot);
  s.setRandom(robot);
  return s;
}


SplitSolution SplitSolution::Random(const Robot& robot, 
                                    const ContactStatus& contact_status) {
  SplitSolution s(robot);
  s.setRandom(robot, contact_status);
  return s;
}


SplitSolution SplitSolution::Random(const Robot& robot, 
                                    const ImpactStatus& impact_status) {
  SplitSolution s(robot);
  s.setRandom(robot, impact_status);
  return s;
}


SplitSolution SplitSolution::Random(const Robot& robot, 
                                    const ContactStatus& contact_status, 
                                    const ImpactStatus& impact_status) {
  SplitSolution s(robot);
  s.setRandom(robot, contact_status, impact_status);
  return s;
}


void SplitSolution::disp(std::ostream& os) const {
  os << "SplitSolution:" << "\n";
  os << "  q = " << q.transpose() << "\n";
  os << "  v = " << v.transpose() << "\n";
  os << "  u = " << u.transpose() << "\n";
  os << "  a = " << a.transpose() << "\n";
  if (dimf() > 0) {
    os << "  f = " << f_stack().transpose() << "\n";
  }
  os << "  lmd = " << lmd.transpose() << "\n";
  os << "  gmm = " << gmm.transpose() << "\n";
  os << "  beta = " << beta.transpose() << "\n";
  if (dimf() > 0) {
    os << "  mu = " << mu_stack().transpose() << "\n";
  }
  if (dims() > 0) {
    os << "  xi = " << xi_stack().transpose() << std::flush;
  }
}


std::ostream& operator<<(std::ostream& os, const SplitSolution& s) {
  s.disp(os);
  return os;
}

} // namespace robotoc 