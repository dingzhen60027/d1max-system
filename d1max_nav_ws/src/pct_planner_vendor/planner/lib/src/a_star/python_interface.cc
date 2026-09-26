#include "a_star/a_star_search.h"
#include "pybind11/eigen.h"
#include "pybind11/pybind11.h"

namespace py = pybind11;

PYBIND11_MODULE(a_star, m) {
  // enum
  py::enum_<HeuristicType>(m, "HeuristicType")
      .value("EUCLIDEAN", HeuristicType::kEuclidean)
      .value("MANHATTAN", HeuristicType::kManhattan)
      .value("DIAGONAL", HeuristicType::kDiagonal)
      .export_values();

  auto pyAstar = py::class_<Astar>(m, "Astar");
  pyAstar
      .def(py::init<HeuristicType>(),
           py::arg("h_type") = HeuristicType::kDiagonal)
      .def("init", &Astar::Init)
      .def("search", &Astar::Search)
      .def("debug", &Astar::Debug)
      .def("get_result_matrix", &Astar::GetResultMatrix)
      .def("get_result_size", &Astar::GetResultSize)
      .def("get_touched_count", &Astar::GetTouchedCount)
      .def("get_last_reset_count", &Astar::GetLastResetCount)
      .def("get_search_count", &Astar::GetSearchCount)
      .def("get_memory_stats", [](const Astar& planner) {
        py::dict stats;
        stats["dense_cells"] = planner.GetDenseCellCount();
        stats["allocated_query_nodes"] = planner.GetAllocatedNodeCount();
        stats["last_query_nodes"] = planner.GetLastQueryNodeCount();
        stats["peak_query_nodes"] = planner.GetPeakQueryNodeCount();
        stats["retained_path_nodes"] = planner.GetResultSize();
        stats["node_size_bytes"] = planner.GetNodeSizeBytes();
        stats["retained_query_buckets"] = planner.GetQueryBucketCount();
        stats["grid_shared_with_optimizer"] = planner.SharesGridStorage();
        stats["owned_dense_grid_bytes"] = planner.SharesGridStorage() ? 0 :
            planner.GetDenseCellCount() * sizeof(double) * 3;
        return stats;
      })
      .def("get_cost_layer", &Astar::GetCostLayer)
      .def("get_ele_layer", &Astar::GetEleLayer)
      .def("get_visited_set", &Astar::GetVisitedSet);
}
