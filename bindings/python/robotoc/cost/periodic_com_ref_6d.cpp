#include <pybind11/pybind11.h>
#include <pybind11/eigen.h>
#include <pybind11/numpy.h>
#include <pybind11/stl.h>
#include <pybind11/complex.h>
#include <pybind11/chrono.h>

#include "robotoc/cost/periodic_com_ref_6d.hpp"
#include "robotoc/utils/pybind11_macros.hpp"


namespace robotoc {
namespace python {

namespace py = pybind11;

PYBIND11_MODULE(periodic_com_ref_6d, m) {
  py::class_<PeriodicComRef6D, TaskSpace6DRefBase,
             std::shared_ptr<PeriodicComRef6D>>(m, "PeriodicComRef6D")
    .def(py::init<const SE3&, const std::vector<SE3>&, const double, const int, const std::vector<bool>>(),
          py::arg("x6d_ref0"),py::arg("x6d_ref_array"),py::arg("t0"),py::arg("N"),py::arg("inMotion"))
    .def("set_ref", &PeriodicComRef6D::setCoMTrackRef,
          py::arg("x6d_ref0"), py::arg("t0"))
    .def("updateRef", &PeriodicComRef6D::updateRef,
          py::arg("grid_info"), py::arg("x6d_ref"))
    .def("is_active", &PeriodicComRef6D::isActive,
          py::arg("grid_info"));
}

} // namespace python
} // namespace robotoc