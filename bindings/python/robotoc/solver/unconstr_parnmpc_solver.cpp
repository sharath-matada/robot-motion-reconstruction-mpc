#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <pybind11/eigen.h>
#include <pybind11/numpy.h>

#include "robotoc/solver/unconstr_parnmpc_solver.hpp"
#include "robotoc/utils/pybind11_macros.hpp"


namespace robotoc {
namespace python {

namespace py = pybind11;

PYBIND11_MODULE(unconstr_parnmpc_solver, m) {
  py::class_<UnconstrParNMPCSolver>(m, "UnconstrParNMPCSolver")
    .def(py::init<const OCP&, const SolverOptions&>(),
          py::arg("ocp"), py::arg("solver_options")=SolverOptions())
    .def("set_solver_options", &UnconstrParNMPCSolver::setSolverOptions,
          py::arg("solver_options"))
    .def("discretize", &UnconstrParNMPCSolver::discretize,
          py::arg("t"))
    .def("init_constraints", &UnconstrParNMPCSolver::initConstraints)
    .def("init_backward_correction", &UnconstrParNMPCSolver::initBackwardCorrection)
    .def("solve", &UnconstrParNMPCSolver::solve,
          py::arg("t"), py::arg("q"), py::arg("v"), py::arg("init_solver")=true)
    .def("get_solver_statistics", &UnconstrParNMPCSolver::getSolverStatistics)
    .def("get_solution", 
          static_cast<const SplitSolution& (UnconstrParNMPCSolver::*)(const int) const>(&UnconstrParNMPCSolver::getSolution))
    .def("get_solution", 
          static_cast<std::vector<Eigen::VectorXd> (UnconstrParNMPCSolver::*)(const std::string&) const>(&UnconstrParNMPCSolver::getSolution))
    .def("set_solution", &UnconstrParNMPCSolver::setSolution,
          py::arg("name"), py::arg("value"))
    .def("KKT_error", 
          static_cast<double (UnconstrParNMPCSolver::*)(const double, const Eigen::VectorXd&, const Eigen::VectorXd&)>(&UnconstrParNMPCSolver::KKTError),
          py::arg("t"), py::arg("q"), py::arg("v"))
    .def("KKT_error", 
          static_cast<double (UnconstrParNMPCSolver::*)() const>(&UnconstrParNMPCSolver::KKTError))
    .def("get_time_discretization", &UnconstrParNMPCSolver::getTimeDiscretization)
    .def("set_robot_properties", &UnconstrParNMPCSolver::setRobotProperties,
          py::arg("properties"))
    DEFINE_ROBOTOC_PYBIND11_CLASS_CLONE(UnconstrParNMPCSolver);
}

} // namespace python
} // namespace robotoc