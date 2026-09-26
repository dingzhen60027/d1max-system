#include "ele_planner/offline_ele_planner.h"
#include "pybind11/eigen.h"
#include "pybind11/pybind11.h"

namespace py = pybind11;

PYBIND11_MODULE(ele_planner, m) {
  auto pyOfflineElePlanner =
      py::class_<OfflineElePlanner>(m, "OfflineElePlanner");
  pyOfflineElePlanner
      .def(py::init<double, bool>(), py::arg("max_heading_rate"),
           py::arg("use_quintic") = false)
      .def("init_map", &OfflineElePlanner::InitMap)
      // Eigen Ref cannot represent negative strides. Preserve the previous
      // API for reversed arrays via a copying fallback; normal C/F/strided
      // float64 arrays select the zero-temporary Ref overload above.
      .def("init_map", [](OfflineElePlanner& planner, double threshold, double margin,
                           double resolution, int layers, double weight,
                           const Eigen::MatrixXd& cost, const Eigen::MatrixXd& height,
                           const Eigen::MatrixXd& ceiling, const Eigen::MatrixXd& gateway,
                           const Eigen::MatrixXd& grad_x, const Eigen::MatrixXd& grad_y) {
        planner.InitMap(threshold, margin, resolution, layers, weight,
                        cost, height, ceiling, gateway, grad_x, grad_y);
      })
      .def("plan", &OfflineElePlanner::Plan)
      .def("debug", &OfflineElePlanner::Debug)
      .def("set_reference_height", &OfflineElePlanner::SetReferenceHeight)
      .def("set_max_iterations", &OfflineElePlanner::set_max_iterations)
      .def("set_optimizer_sample_interval", [](OfflineElePlanner& planner, int interval) {
        if (interval < 1 || interval > 100) throw py::value_error("sample interval must be in [1, 100]");
        planner.set_optimizer_sample_interval(interval);
      })
      .def("get_path_finder", &OfflineElePlanner::get_path_finder,
           py::return_value_policy::reference_internal)
      .def("get_search_result_size", &OfflineElePlanner::GetSearchResultSize)
      .def("get_map", &OfflineElePlanner::get_map)
      .def("get_trajectory_optimizer",
           &OfflineElePlanner::get_trajectory_optimizer,
           py::return_value_policy::reference_internal)
      .def("get_trajectory_optimizer_wnoj",
           &OfflineElePlanner::get_trajectory_optimizer_wnoj,
           py::return_value_policy::reference_internal)
      .def("get_debug_path", &OfflineElePlanner::GetDebugPath);
}
