#ifndef ROBOTOC_UTILS_OCP_BENCHMARKER_HXX_
#define ROBOTOC_UTILS_OCP_BENCHMARKER_HXX_ 

#include "robotoc/utils/ocp_benchmarker.hxx"

#include <iostream>
#include <chrono>


namespace robotoc {
namespace benchmark {

template <typename OCPSolverType>
inline void CPUTime(OCPSolverType& ocp_solver, const double t, 
                    const Eigen::VectorXd& q, const Eigen::VectorXd& v, 
                    const int num_iteration) {
  std::chrono::system_clock::time_point start_clock, end_clock;
  start_clock = std::chrono::system_clock::now();
  for (int i=0; i<num_iteration; ++i) {
    ocp_solver.updateSolution(t, q, v);
  }
  end_clock = std::chrono::system_clock::now();
  std::cout << "---------- OCP benchmark : CPU time ----------" << std::endl;
  std::cout << "total CPU time: " 
            << 1e-03 * std::chrono::duration_cast<std::chrono::microseconds>(
                  end_clock-start_clock).count() 
            << "[ms]" << std::endl;
  std::cout << "CPU time per update: " 
            << 1e-03 * std::chrono::duration_cast<std::chrono::microseconds>(
                  end_clock-start_clock).count() / num_iteration 
            << "[ms]" << std::endl;
  std::cout << "-----------------------------------" << std::endl;
  std::cout << std::endl;
}

} // namespace benchmark
} // namespace robotoc 

#endif // ROBOTOC_UTILS_OCP_BENCHMARKER_HXX_