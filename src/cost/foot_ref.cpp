#include "robotoc/cost/foot_ref.hpp"


namespace robotoc {

FootRef::FootRef(const Eigen::Vector3d& x3d_ref0, const std::vector<Eigen::Vector3d>& x3d_ref_array,
                       const double t0, const int N, const std::vector<bool> inMotion)
  : TaskSpace3DRefBase(),
    x3d_ref0_(x3d_ref0),
    x3d_ref_array_(x3d_ref_array),
    t0_(t0),
    N_(N),
    inMotion_(inMotion)
     {
}

FootRef::~FootRef() {
}

void FootRef::setCoMTrackRef(const Eigen::VectorXd& x3d_ref0,
                                      const double t0) {
  x3d_ref0_ = x3d_ref0;
  t0_ = t0;
}

void FootRef::updateRef(const GridInfo& grid_info, Eigen::VectorXd& x3d_ref) const{
  if (grid_info.t < t0_){
   x3d_ref = x3d_ref0_;
  }
  else{
    int j = (grid_info.t - t0_)/(0.04*0.4);
    if (j>N_){
      j = N_-1;
    }
    x3d_ref = x3d_ref_array_[j];
  }
}

bool FootRef::isActive(const GridInfo& grid_info) const {
   if (grid_info.t < t0_){
   return false;
  }
  else{
    int j = (grid_info.t - t0_)/(0.04*0.4);
    if (j>N_){
      j = N_-1;
    }
    return inMotion_[j];
  }
}

} // namespace robotoc