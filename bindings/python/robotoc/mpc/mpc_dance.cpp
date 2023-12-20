#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <pybind11/eigen.h>
#include <pybind11/numpy.h>

#include "robotoc/mpc/mpc_dance.hpp"
#include "robotoc/utils/pybind11_macros.hpp"


namespace robotoc {
namespace python {

namespace py = pybind11;

PYBIND11_MODULE(mpc_dance, m) {
  py::class_<MPCDance>(m, "MPCDance")
    .def(py::init<const Robot&, const double, const int, 
         const Eigen::VectorXd&, const Eigen::Vector3d&, const Eigen::Vector3d&, const Eigen::Vector3d&, const Eigen::Vector3d&>(),
      //    const Eigen::VectorXd&, const Eigen::Vector3d&, const Eigen::Vector3d&, const Eigen::Vector3d&, const Eigen::Vector3d&>(),
         py::arg("quadruped_robot"), py::arg("T"), py::arg("N"), py::arg("com_ref0"),
         py::arg(" x3d0_LF"), py::arg("x3d0_LH"), py::arg("x3d0_RF"), py::arg("x3d0_RH"))
    .def("set_gait_pattern", &MPCDance::setGaitPattern,
         py::arg("planner"), py::arg("CoM_t0"), py::arg("LF_t0"), py::arg("LH_t0"), py::arg("RF_t0"), py::arg("RH_t0"), 
         py::arg("q_array"), py::arg("x3d_LF_array"), py::arg("x3d_LH_array"),py::arg("x3d_RF_array"), py::arg("x3d_RH_array"),
         py::arg("LF_inMotion"), py::arg("LH_inMotion"),py::arg("RF_inMotion"),py::arg("RH_inMotion"),py::arg("size"))
    .def("init", &MPCDance::init,
          py::arg("t"), py::arg("q"), py::arg("v"), py::arg("solver_options"))
    .def("reset", 
          static_cast<void (MPCDance::*)()>(&MPCDance::reset))
    .def("reset", 
          static_cast<void (MPCDance::*)(const Eigen::VectorXd&, const Eigen::VectorXd&)>(&MPCDance::reset),
          py::arg("q"), py::arg("v"))
    .def("set_solver_options", &MPCDance::setSolverOptions,
          py::arg("solver_options"))
    .def("update_solution", &MPCDance::updateSolution,
          py::arg("t"), py::arg("dt"), py::arg("q"), py::arg("v"))
    .def("get_initial_control_input", &MPCDance::getInitialControlInput)
    .def("get_solution", &MPCDance::getSolution)
    .def("KKT_error", 
          static_cast<double (MPCDance::*)(const double, const Eigen::VectorXd&, const Eigen::VectorXd&)>(&MPCDance::KKTError),
          py::arg("t"), py::arg("q"), py::arg("v"))
    .def("KKT_error", 
          static_cast<double (MPCDance::*)() const>(&MPCDance::KKTError))
//     .def("get_cost_handle", &MPCDance::getCostHandle)
    .def("get_swing_foot_cost_handle", &MPCDance::getSwingFootCostHandle)
    .def("get_com_cost_handle", &MPCDance::getCoMCostHandle)
    .def("get_constraints_handle", &MPCDance::getConstraintsHandle)
    .def("get_friction_cone_handle", &MPCDance::getFrictionConeHandle)
    .def("get_solver", &MPCDance::getSolver)
    .def("get_contact_sequence", &MPCDance::getContactSequence)
    .def("set_robot_properties", &MPCDance::setRobotProperties)
    DEFINE_ROBOTOC_PYBIND11_CLASS_CLONE(MPCDance);
}

} // namespace python
} // namespace robotoc