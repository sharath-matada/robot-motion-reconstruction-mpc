#include <pybind11/pybind11.h>
#include <pybind11/eigen.h>
#include <pybind11/numpy.h>
#include <pybind11/stl.h>
#include <pybind11/complex.h>
#include <pybind11/chrono.h>

#include "robotoc/cost/foot_ref.hpp"
#include "robotoc/utils/pybind11_macros.hpp"


namespace robotoc {
namespace python {

namespace py = pybind11;

PYBIND11_MODULE(foot_ref, m) {
  py::class_<FootRef, TaskSpace3DRefBase,
             std::shared_ptr<FootRef>>(m, "FootRef")
    .def(py::init<const Eigen::Vector3d&, const std::vector<Eigen::Vector3d>&, const double, const int, const std::vector<bool>>(),
          py::arg("x3d_ref0"),py::arg("x3d_ref_array"),py::arg("t0"),py::arg("N"),py::arg("inMotion"))
    .def("set_ref", &FootRef::setCoMTrackRef,
          py::arg("x3d_ref0"), py::arg("t0"))
    .def("updateRef", &FootRef::updateRef,
          py::arg("grid_info"), py::arg("x3d_ref"))
    .def("is_active", &FootRef::isActive,
          py::arg("grid_info"));
}

} // namespace python
} // namespace robotoc