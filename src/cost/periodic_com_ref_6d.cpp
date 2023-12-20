#include "robotoc/cost/periodic_com_ref_6d.hpp"


namespace robotoc {

PeriodicComRef6D::PeriodicComRef6D(const SE3& x6d_ref0, const std::vector<SE3>& x6d_ref_array,
                       const double t0, const int N, const std::vector<bool> inMotion)
  : TaskSpace6DRefBase(),
    x6d_ref0_(x6d_ref0),
    x6d_ref_array_(x6d_ref_array),
    t0_(t0),
    N_(N),
    inMotion_(inMotion)
     {
}

PeriodicComRef6D::~PeriodicComRef6D() {
}


void PeriodicComRef6D::setCoMTrackRef(const SE3& x6d_ref0,
                                      const double t0) {
  x6d_ref0_ = x6d_ref0;
  t0_ = t0;
}

void PeriodicComRef6D::updateRef(const GridInfo& grid_info, SE3& x6d_ref) const{
  if (grid_info.t < t0_){
   x6d_ref = x6d_ref0_;
  }
  else{
    int j = (grid_info.t - t0_)/0.04;
    if (j>N_){
      j = N_-1;
    }
    x6d_ref = x6d_ref_array_[j];
  }
}

bool PeriodicComRef6D::isActive(const GridInfo& grid_info) const {
   if (grid_info.t < t0_){
   return false;
  }
  else{
    int j = (grid_info.t - t0_)/0.04;
    if (j>N_){
      j = N_-1;
    }
    return inMotion_[j];
  }
}

} // namespace robotoc