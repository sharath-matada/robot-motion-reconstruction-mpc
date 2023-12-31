#include <pybind11/pybind11.h>

#include "robotoc/cost/cost_function_component_base.hpp"


namespace robotoc {
namespace python {

class PyCostFunctionComponentBase : public CostFunctionComponentBase {
public:
  // Inherit the constructors
  using CostFunctionComponentBase::CostFunctionComponentBase;

  double evalStageCost(Robot& robot, const ContactStatus& contact_status, 
                       CostFunctionData& data, const GridInfo& grid_info, 
                       const SplitSolution& s) const override {
    PYBIND11_OVERRIDE_PURE(double, CostFunctionComponentBase, 
                           evalStageCost, 
                           robot, contact_status, data, grid_info, s);
  }

  void evalStageCostDerivatives(Robot& robot, const ContactStatus& contact_status, 
                                CostFunctionData& data, const GridInfo& grid_info, 
                                const SplitSolution& s, 
                                SplitKKTResidual& kkt_residual) const override {
    PYBIND11_OVERRIDE_PURE(void, CostFunctionComponentBase, 
                           evalStageCostDerivatives, 
                           robot, contact_status, data, grid_info, s, kkt_residual);
  }

  void evalStageCostHessian(Robot& robot, const ContactStatus& contact_status, 
                            CostFunctionData& data, const GridInfo& grid_info, 
                            const SplitSolution& s, 
                            SplitKKTMatrix& kkt_matrix) const override {
    PYBIND11_OVERRIDE_PURE(void, CostFunctionComponentBase, 
                           evalStageCostHessian, 
                           robot, contact_status, data, grid_info, s, kkt_matrix);
  }

  double evalTerminalCost(Robot& robot, CostFunctionData& data, 
                          const GridInfo& grid_info, 
                          const SplitSolution& s) const override {
    PYBIND11_OVERRIDE_PURE(double, CostFunctionComponentBase, 
                           evalTerminalCost, 
                           robot, data, grid_info, s);
  }

  void evalTerminalCostDerivatives(Robot& robot, CostFunctionData& data, 
                                   const GridInfo& grid_info, const SplitSolution& s, 
                                   SplitKKTResidual& kkt_residual) const override {
    PYBIND11_OVERRIDE_PURE(void, CostFunctionComponentBase, 
                           evalTerminalCostDerivatives, 
                           robot, data, grid_info, s, kkt_residual);
  }

  void evalTerminalCostHessian(Robot& robot, CostFunctionData& data, 
                               const GridInfo& grid_info, const SplitSolution& s, 
                               SplitKKTMatrix& kkt_matrix) const override {
    PYBIND11_OVERRIDE_PURE(void, CostFunctionComponentBase, 
                           evalTerminalCostHessian, 
                           robot, data, grid_info, s, kkt_matrix);
  }

  double evalImpactCost(Robot& robot, const ImpactStatus& impact_status, 
                         CostFunctionData& data, const GridInfo& grid_info, 
                         const SplitSolution& s) const override {
    PYBIND11_OVERRIDE_PURE(double, CostFunctionComponentBase, 
                           evalImpactCost, 
                           robot, impact_status, data, grid_info, s);
  }

  void evalImpactCostDerivatives(Robot& robot, const ImpactStatus& impact_status, 
                                  CostFunctionData& data, const GridInfo& grid_info,
                                  const SplitSolution& s, 
                                  SplitKKTResidual& kkt_residual) const override {
    PYBIND11_OVERRIDE_PURE(void, CostFunctionComponentBase, 
                           evalImpactCostDerivatives, 
                           robot, impact_status, data, grid_info, s, kkt_residual);
  }

  void evalImpactCostHessian(Robot& robot, const ImpactStatus& impact_status, 
                              CostFunctionData& data, const GridInfo& grid_info,
                              const SplitSolution& s, 
                              SplitKKTMatrix& kkt_matrix) const override {
    PYBIND11_OVERRIDE_PURE(void, CostFunctionComponentBase, 
                           evalImpactCostHessian, 
                           robot, impact_status, data, grid_info, s, kkt_matrix);
  }
};


namespace py = pybind11;

PYBIND11_MODULE(cost_function_component_base, m) {
  py::class_<CostFunctionComponentBase, 
             PyCostFunctionComponentBase, 
             std::shared_ptr<CostFunctionComponentBase>>(m, "CostFunctionComponentBase")
    .def(py::init<>())
    .def("evalStageCost", &CostFunctionComponentBase::evalStageCost,
          py::arg("robot"), py::arg("contact_status"), py::arg("data"), 
          py::arg("grid_info"), py::arg("s"))
    .def("evalStageCostDerivatives", &CostFunctionComponentBase::evalStageCostDerivatives,
          py::arg("robot"), py::arg("contact_status"), py::arg("data"), 
          py::arg("grid_info"), py::arg("s"), py::arg("kkt_residual"))
    .def("evalStageCostHessian", &CostFunctionComponentBase::evalStageCostHessian,
          py::arg("robot"), py::arg("contact_status"), py::arg("data"), 
          py::arg("grid_info"), py::arg("s"), py::arg("kkt_matrix"))
    .def("evalTerminalCost", &CostFunctionComponentBase::evalTerminalCost,
          py::arg("robot"), py::arg("data"), py::arg("grid_info"), py::arg("s"))
    .def("evalTerminalCostDerivatives", &CostFunctionComponentBase::evalTerminalCostDerivatives,
          py::arg("robot"), py::arg("data"), py::arg("grid_info"), py::arg("s"), 
          py::arg("kkt_residual"))
    .def("evalTerminalCostHessian", &CostFunctionComponentBase::evalTerminalCostHessian,
          py::arg("robot"), py::arg("data"), py::arg("grid_info"), py::arg("s"), 
          py::arg("kkt_matrix"))
    .def("evalImpactCost", &CostFunctionComponentBase::evalImpactCost,
          py::arg("robot"), py::arg("impact_status"), py::arg("data"), 
          py::arg("grid_info"), py::arg("s"))
    .def("evalImpactCostDerivatives", &CostFunctionComponentBase::evalImpactCostDerivatives,
          py::arg("robot"), py::arg("impact_status"), py::arg("data"), 
          py::arg("grid_info"), py::arg("s"), py::arg("kkt_residual"))
    .def("evalImpactCostHessian", &CostFunctionComponentBase::evalImpactCostHessian,
          py::arg("robot"), py::arg("impact_status"), py::arg("data"), 
          py::arg("grid_info"), py::arg("s"), py::arg("kkt_matrix"));

}

} // namespace python
} // namespace robotoc