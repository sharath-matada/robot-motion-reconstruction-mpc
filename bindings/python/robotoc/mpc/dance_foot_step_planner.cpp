#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <pybind11/eigen.h>
#include <pybind11/numpy.h>

#include "robotoc/mpc/dance_foot_step_planner.hpp"
#include "robotoc/utils/pybind11_macros.hpp"


namespace robotoc {
namespace python {

namespace py = pybind11;

PYBIND11_MODULE(dance_foot_step_planner, m) {
  py::class_<DanceFootStepPlanner, ContactPlannerBase, 
             std::shared_ptr<DanceFootStepPlanner>>(m, "DanceFootStepPlanner")
    .def(py::init<const Robot&>(),
         py::arg("quadruped_robot"))
    .def("set_gait_pattern", &DanceFootStepPlanner::setGaitPattern,
          py::arg("step_length"), py::arg("step_yaw"), py::arg("enable_stance_phase")) 
    .def("set_contact_surfaces", 
          static_cast<void (DanceFootStepPlanner::*)(const std::vector<Eigen::Matrix3d>& contact_surfaces)>(&DanceFootStepPlanner::setContactSurfaces),
          py::arg("contact_surfaces"))
    .def("set_contact_surfaces", 
          static_cast<void (DanceFootStepPlanner::*)(const std::vector<std::vector<Eigen::Matrix3d>>& contact_surfaces)>(&DanceFootStepPlanner::setContactSurfaces),
          py::arg("contact_surfaces"))
    .def("init", &DanceFootStepPlanner::init,
          py::arg("q"))
    .def("plan", &DanceFootStepPlanner::plan,
          py::arg("t"), py::arg("q"), py::arg("v"), py::arg("contact_status"), py::arg("planning_steps"))
    .def("contact_placements", &DanceFootStepPlanner::contactPlacements,
          py::arg("step"))
    .def("contact_positions", &DanceFootStepPlanner::contactPositions,
          py::arg("step"))
    .def("contact_surfaces", &DanceFootStepPlanner::contactSurfaces,
          py::arg("step"))
    .def("com", &DanceFootStepPlanner::CoM,
          py::arg("step"))
    .def("R", &DanceFootStepPlanner::R,
          py::arg("step"))
    .def("size", &DanceFootStepPlanner::size)
    DEFINE_ROBOTOC_PYBIND11_CLASS_CLONE(DanceFootStepPlanner)
    DEFINE_ROBOTOC_PYBIND11_CLASS_PRINT(DanceFootStepPlanner);
}

} // namespace python
} // namespace robotoc