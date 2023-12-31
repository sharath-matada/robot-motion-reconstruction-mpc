#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <pybind11/eigen.h>
#include <pybind11/numpy.h>

#include "robotoc/core/performance_index.hpp"
#include "robotoc/utils/pybind11_macros.hpp"


namespace robotoc {
namespace python {

namespace py = pybind11;

PYBIND11_MODULE(performance_index, m) {
  py::class_<PerformanceIndex>(m, "PerformanceIndex")
    .def(py::init<>())
    .def_readwrite("cost", &PerformanceIndex::cost)
    .def_readwrite("cost_barrier", &PerformanceIndex::cost_barrier)
    .def_readwrite("primal_feasibility", &PerformanceIndex::primal_feasibility)
    .def_readwrite("dual_feasibility", &PerformanceIndex::dual_feasibility)
    .def_readwrite("kkt_error", &PerformanceIndex::kkt_error)
    .def("set_zero", &PerformanceIndex::setZero)
    DEFINE_ROBOTOC_PYBIND11_CLASS_CLONE(PerformanceIndex)
    DEFINE_ROBOTOC_PYBIND11_CLASS_PRINT(PerformanceIndex);
}

} // namespace python
} // namespace robotoc