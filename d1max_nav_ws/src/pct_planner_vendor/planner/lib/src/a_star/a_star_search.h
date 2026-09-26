#pragma once

#include <Eigen/Core>
#include <memory>
#include <unordered_map>
#include <vector>

#include "common/data_types.h"

enum HeuristicType : int { kEuclidean = 0, kManhattan = 1, kDiagonal = 2 };

class Node {
 public:
  Node() = default;
  Node(Eigen::Vector3i idx, Node* parent) : idx(idx), parent(parent) {}
  ~Node() = default;

  bool operator==(const Node& other) const { return idx == other.idx; }

  void Reset() {
    f = 0.0;
    g = 1e9;
    parent = nullptr;
  }

  double f = 1e9;
  double g = 1e9;
  double height = 0.0;
  double ele = 0;
  double cost = 0.0;
  int layer = 0;
  Eigen::Vector3i idx = Eigen::Vector3i(0, 0, 0);  // layer, row, col
  Node* parent = nullptr;
};

struct NodeCompare {
  bool operator()(const Node* a, const Node* b) const { return a->f > b->f; }
};

class Astar {
 public:
  Astar(const HeuristicType h_type = kDiagonal) : h_type_(h_type) {
    switch (h_type) {
      case kEuclidean:
        printf("Euclidean heuristic is used\n");
        break;
      case kManhattan:
        printf("Manhattan heuristic is used\n");
        break;
      case kDiagonal:
        printf("Diagonal heuristic is used\n");
        break;
    };
  }
  ~Astar() = default;
  // Search results/parents point into this map. Copying is both expensive and
  // unsafe: a shallow pointer copy would still refer to the source's nodes.
  Astar(const Astar&) = delete;
  Astar& operator=(const Astar&) = delete;

  void Init(const double cost_threshold, const int num_layers,
            const double resolution, const double step_cost_weight,  const Eigen::MatrixXd& cost_map,
            const Eigen::MatrixXd& height_map, const Eigen::MatrixXd& ele_map);
  // Immutable matrices can be shared with the optimizer without another full
  // map copy. The shared pointers pin their owner for the entire A* lifetime.
  void InitShared(const double cost_threshold, const int num_layers,
                  const double resolution, const double step_cost_weight,
                  std::shared_ptr<const Eigen::MatrixXd> cost_map,
                  std::shared_ptr<const Eigen::MatrixXd> height_map,
                  std::shared_ptr<const Eigen::MatrixXd> ele_map);

  void Reset();

  void Debug() { debug_ = true; }

  bool Search(const Eigen::Vector3i& start, const Eigen::Vector3i& goal);

  std::vector<PathPoint> GetPathPoints() const;

  Eigen::MatrixXd GetResultMatrix() const;
  size_t GetResultSize() const { return search_result_.size(); }
  size_t GetTouchedCount() const { return touched_count_; }
  size_t GetLastResetCount() const { return last_reset_count_; }
  size_t GetSearchCount() const { return search_count_; }
  size_t GetDenseCellCount() const { return cost_map_ ? cost_map_->size() : 0; }
  size_t GetAllocatedNodeCount() const { return query_nodes_.size(); }
  size_t GetLastQueryNodeCount() const { return last_query_nodes_; }
  size_t GetPeakQueryNodeCount() const { return peak_query_nodes_; }
  size_t GetNodeSizeBytes() const { return sizeof(Node); }
  size_t GetQueryBucketCount() const { return query_nodes_.bucket_count(); }
  bool SharesGridStorage() const { return shares_grid_storage_; }
  Eigen::MatrixXi GetVisitedSet() const { return visited_set_; }

  Eigen::MatrixXd GetCostLayer(int layer) const;
  Eigen::MatrixXd GetEleLayer(int layer) const;

 private:
  Node* GetOrCreateNode(int layer, int row, int col);
  void ReleaseQueryNodes();
  double CalculateStepCost(const Node* node1, const Node* node2) const;

  int DecideLayer(const Node* cur_node) const;

  int GetHash(const Eigen::Vector3i& idx) const;

  std::vector<Eigen::Vector3i> GetNeighbors(Node* node) const;

  double GetHeuristic(const Node* node1, const Node* node2) const;

  Eigen::MatrixXd PathToMatrix(const std::vector<Eigen::Vector3i>& path);

  void ToPathPoints(const std::vector<Eigen::Vector3i>& path,
                    std::vector<PathPoint>& path_points);

  void ConvertClosedSetToMatrix(
      const std::unordered_map<int, Node*>& closed_set);

 private:
  HeuristicType h_type_ = kDiagonal;

  int max_x_ = 0;
  int max_y_ = 0;
  int max_layers_ = 0;
  int xy_size_ = 0;
  double resolution_ = 0;
  std::shared_ptr<const Eigen::MatrixXd> cost_map_;
  std::shared_ptr<const Eigen::MatrixXd> height_map_;
  std::shared_ptr<const Eigen::MatrixXd> ele_map_;
  // std::unordered_map preserves references/pointers when rehashing. Node
  // addresses are stable for the entire search, including queue and parents.
  std::unordered_map<size_t, Node> query_nodes_;
  bool shares_grid_storage_ = false;
  double cost_threshold_ = 35;
  double step_cost_weight_ = 1.0;

  int search_layer_depth_ = 1;
  std::vector<int> search_layers_offset_;

  bool debug_ = false;
  Eigen::MatrixXi visited_set_;

  // Results own the few path nodes, not references into the query workspace.
  // All search workspace is released on success/failure, not held until reuse.
  std::vector<Node> search_result_;
  size_t touched_count_ = 0;
  size_t last_reset_count_ = 0;
  size_t search_count_ = 0;
  size_t last_query_nodes_ = 0;
  size_t peak_query_nodes_ = 0;
};
