#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <pybind11/eigen.h>
#include <pybind11/numpy.h>

#include "robotoc/hybrid/hybrid_ocp_discretization.hpp"

#include <iostream>


namespace robotoc {
namespace python {

namespace py = pybind11;

PYBIND11_MODULE(hybrid_ocp_discretization, m) {
  py::class_<HybridOCPDiscretization>(m, "HybridOCPDiscretization")
    .def(py::init<const double, const int, const int>())
    .def("discretize", &HybridOCPDiscretization::discretize)
    .def("N", &HybridOCPDiscretization::N)
    .def("N_impulse", &HybridOCPDiscretization::N_impulse)
    .def("N_lift", &HybridOCPDiscretization::N_lift)
    .def("N_ideal", &HybridOCPDiscretization::N_ideal)
    .def("contact_phase", &HybridOCPDiscretization::contactPhase)
    .def("contact_phase_after_impulse", &HybridOCPDiscretization::contactPhaseAfterImpulse)
    .def("contact_phase_after_lift", &HybridOCPDiscretization::contactPhaseAfterLift)
    .def("impulse_index_after_time_stage", &HybridOCPDiscretization::impulseIndexAfterTimeStage)
    .def("lift_index_after_time_stage", &HybridOCPDiscretization::liftIndexAfterTimeStage)
    .def("time_stage_before_impulse", &HybridOCPDiscretization::timeStageBeforeImpulse)
    .def("time_stage_after_impulse", &HybridOCPDiscretization::timeStageAfterImpulse)
    .def("time_stage_before_lift", &HybridOCPDiscretization::timeStageBeforeLift)
    .def("time_stage_after_lift", &HybridOCPDiscretization::timeStageAfterLift)
    .def("is_time_stage_before_impulse", &HybridOCPDiscretization::isTimeStageBeforeImpulse)
    .def("is_time_stage_after_impulse", &HybridOCPDiscretization::isTimeStageAfterImpulse)
    .def("is_time_stage_before_lift", &HybridOCPDiscretization::isTimeStageBeforeLift)
    .def("is_time_stage_after_lift", &HybridOCPDiscretization::isTimeStageAfterLift)
    .def("t", &HybridOCPDiscretization::t)
    .def("t_impulse", &HybridOCPDiscretization::t_impulse)
    .def("t_lift", &HybridOCPDiscretization::t_lift)
    .def("dt", &HybridOCPDiscretization::dt)
    .def("dt_aux", &HybridOCPDiscretization::dt_aux)
    .def("dt_lift", &HybridOCPDiscretization::dt_lift)
    .def("dt_ideal", &HybridOCPDiscretization::dt_ideal)
    .def("is_STO_enabled_impulse", &HybridOCPDiscretization::isSTOEnabledImpulse)
    .def("is_STO_enabled_lift", &HybridOCPDiscretization::isSTOEnabledLift)
    .def("event_index_impulse", &HybridOCPDiscretization::eventIndexImpulse)
    .def("event_index_lift", &HybridOCPDiscretization::eventIndexLift)
    .def("event_type", &HybridOCPDiscretization::eventType)
    .def("is_formulation_tractable", &HybridOCPDiscretization::isFormulationTractable)
    .def("is_switching_time_consistent", &HybridOCPDiscretization::isSwitchingTimeConsistent)
    .def("__str__", [](const HybridOCPDiscretization& self) {
        std::stringstream ss;
        ss << self;
        return ss.str();
      });
}

} // namespace python
} // namespace robotoc