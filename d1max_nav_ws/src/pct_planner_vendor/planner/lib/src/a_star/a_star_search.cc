#include "a_star/a_star_search.h"

#include <algorithm>
#include <chrono>
#include <iostream>
#include <queue>
#include <stdexcept>
#include <unordered_map>
#include <unordered_set>

using std::cout;
using std::endl;

// 9 neighbors in 2d
static std::vector<Eigen::Vector2i> kNeighbors = std::vector<Eigen::Vector2i>{
    Eigen::Vector2i(-1, -1), Eigen::Vector2i(-1, 0), Eigen::Vector2i(-1, 1),
    Eigen::Vector2i(0, -1),  Eigen::Vector2i(0, 1),  Eigen::Vector2i(1, -1),
    Eigen::Vector2i(1, 0),   Eigen::Vector2i(1, 1),
};

void Astar::Init(const double cost_threshold, const int num_layers,
                 const double resolution,  const double step_cost_weight, const Eigen::MatrixXd& cost_map,
                 const Eigen::MatrixXd& height_map,
                 const Eigen::MatrixXd& ele_map) {
  InitShared(cost_threshold, num_layers, resolution, step_cost_weight,
             std::make_shared<const Eigen::MatrixXd>(cost_map),
             std::make_shared<const Eigen::MatrixXd>(height_map),
             std::make_shared<const Eigen::MatrixXd>(ele_map));
  shares_grid_storage_ = false;
}

void Astar::InitShared(const double cost_threshold, const int num_layers,
                       const double resolution, const double step_cost_weight,
                       std::shared_ptr<const Eigen::MatrixXd> cost_map,
                       std::shared_ptr<const Eigen::MatrixXd> height_map,
                       std::shared_ptr<const Eigen::MatrixXd> ele_map) {
  auto t0 = std::chrono::high_resolution_clock::now();
  if (!cost_map || !height_map || !ele_map ||
      num_layers <= 0 || cost_map->rows() <= 0 || cost_map->cols() <= 0 ||
      cost_map->rows() % num_layers != 0 ||
      height_map->rows() != cost_map->rows() || height_map->cols() != cost_map->cols() ||
      ele_map->rows() != cost_map->rows() || ele_map->cols() != cost_map->cols() ||
      !std::isfinite(resolution) || resolution <= 0) {
    throw std::invalid_argument("Invalid A* map dimensions or resolution");
  }
  // Do not retain pointers across map reinitialization/reallocation.
  ReleaseQueryNodes();
  touched_count_ = 0;
  search_result_.clear();
  visited_set_.resize(0, 3);
  last_reset_count_ = 0;
  search_count_ = 0;
  last_query_nodes_ = peak_query_nodes_ = 0;
  cost_threshold_ = cost_threshold;
  step_cost_weight_ = step_cost_weight;
  resolution_ = resolution;

  max_x_ = cost_map->cols();
  max_y_ = cost_map->rows() / num_layers;
  max_layers_ = num_layers;
  xy_size_ = max_x_ * max_y_;

  cost_map_ = std::move(cost_map);
  height_map_ = std::move(height_map);
  ele_map_ = std::move(ele_map);
  shares_grid_storage_ = true;
  auto duration = std::chrono::duration_cast<std::chrono::microseconds>(
      std::chrono::high_resolution_clock::now() - t0);

  search_layers_offset_.clear();
  search_layers_offset_.emplace_back(0);
  for (int i = 0; i < search_layer_depth_; ++i) {
    search_layers_offset_.emplace_back(-(i + 1));
    search_layers_offset_.emplace_back(i + 1);
  }

  printf(
      "Astar initialized, max_x: %d, max_y: %d, max_layers: %d, time elapsed: "
      "%f ms\n",
      max_x_, max_y_, max_layers_, duration.count() / 1000.0);
}

void Astar::Reset() {
  last_reset_count_ = touched_count_;
  touched_count_ = 0;
  search_result_.clear();
  visited_set_.resize(0, 3);
  ReleaseQueryNodes();
  last_query_nodes_ = 0;
}

Node* Astar::GetOrCreateNode(int layer, int row, int col) {
  const size_t index = (static_cast<size_t>(layer) * max_y_ + row) * max_x_ + col;
  const auto found = query_nodes_.find(index);
  if (found != query_nodes_.end()) return &found->second;
  const int matrix_row = layer * max_y_ + row;
  const double height = (*height_map_)(matrix_row, col);
  Node node(Eigen::Vector3i(static_cast<int>(height / resolution_), row, col), nullptr);
  node.cost = (*cost_map_)(matrix_row, col);
  node.height = height;
  node.ele = (*ele_map_)(matrix_row, col);
  node.layer = layer;
  return &query_nodes_.emplace(index, std::move(node)).first->second;
}

void Astar::ReleaseQueryNodes() {
  last_query_nodes_ = query_nodes_.size();
  peak_query_nodes_ = std::max(peak_query_nodes_, last_query_nodes_);
  query_nodes_.clear();
  // A difficult/failed query must not retain a map-sized bucket table. Keep
  // only a small bucket capacity; no live Node survives a completed query.
  if (query_nodes_.bucket_count() > 4096) query_nodes_.rehash(1024);
}

int Astar::GetHash(const Eigen::Vector3i& idx) const {
  return idx[0] * 10000000 + idx[1] * max_x_ + idx[2];
}

bool Astar::Search(const Eigen::Vector3i& start, const Eigen::Vector3i& goal) {
  auto t0 = std::chrono::high_resolution_clock::now();

  Reset();
  ++search_count_;
  const auto in_bounds = [this](const Eigen::Vector3i& p) {
    return p[0] >= 0 && p[0] < max_layers_ && p[1] >= 0 &&
           p[1] < max_x_ && p[2] >= 0 && p[2] < max_y_;
  };
  if (!in_bounds(start) || !in_bounds(goal)) {
    throw std::out_of_range("A* endpoint is outside the initialized map");
  }

  auto start_node = GetOrCreateNode(start[0], start[2], start[1]);
  auto goal_node = GetOrCreateNode(goal[0], goal[2], goal[1]);
  ++touched_count_;
  start_node->g = 0.0;

  if (goal_node->cost > cost_threshold_) {
    printf("goal node is not reachable, cost: %f", goal_node->cost);
    ReleaseQueryNodes();
    return false;
  }

  std::priority_queue<Node*, std::vector<Node*>, NodeCompare> open_set;
  std::unordered_map<int, Node*> closed_set;
  const auto state_key = [this](const Node* node) {
    // Equal measured heights in different PCT slices are not equal search
    // states: their future traversability can differ at a ceiling or boundary.
    return node->layer * xy_size_ + node->idx[1] * max_x_ + node->idx[2];
  };

  open_set.push(start_node);

  printf("start searching\n");

  while (!open_set.empty()) {
    Node* current_node = open_set.top();
    open_set.pop();

    if (current_node->idx == goal_node->idx && current_node->layer == goal_node->layer) {
      while (current_node->parent != nullptr) {
        // search_result_.emplace_back(Eigen::Vector3i(
        //     current_node->layer, current_node->idx[1],
        //     current_node->idx[2]));
        search_result_.push_back(*current_node);
        search_result_.back().parent = nullptr;
        current_node = current_node->parent;
      }
      std::reverse(search_result_.begin(), search_result_.end());
      if (debug_) ConvertClosedSetToMatrix(closed_set);
      ReleaseQueryNodes();
      auto duration = std::chrono::duration_cast<std::chrono::microseconds>(
          std::chrono::high_resolution_clock::now() - t0);
      printf("path found, time elapsed: %f ms\n",
             duration.count() / 1000.0);
      return true;
    }

    closed_set[state_key(current_node)] = current_node;

    // int layer = current_node->layer;
    // if (current_node->ele > 0.5) {
    //   layer = std::min(layer + 1, max_layers_ - 1);
    // } else if (current_node->ele < -0.5) {
    //   layer = std::max(layer - 1, 0);
    // }
    // A greedy DecideLayer discarded a valid same-floor continuation when
    // an overlapping thinner slice led to a dead end. Explore the incumbent
    // slice and genuine adjacent measured overlap states independently.
    std::vector<int> layers{current_node->layer};
    for (int other : {current_node->layer - 1, current_node->layer + 1}) {
      if (other < 0 || other >= max_layers_) continue;
      const int row = other * max_y_ + current_node->idx[1];
      const int col = current_node->idx[2];
      if ((*cost_map_)(row,col) <= cost_threshold_ &&
          std::abs((*height_map_)(row,col) - current_node->height) <= 0.1)
        layers.push_back(other);
    }
    for (const int layer : layers) {
    int i, j = 0;
    double tentative_g = 0.0;
    for (const auto& neighbor : kNeighbors) {
      i = current_node->idx[1] + neighbor[0];
      j = current_node->idx[2] + neighbor[1];

      if (i < 0 || i >= max_y_ || j < 0 || j >= max_x_) {
        continue;
      }

      // A diagonal traverses the shared corner of four cells. Checking only
      // its destination lets A* seed GPMP through an occupied/unknown corner;
      // the downstream continuous-curve validator then correctly rejects it.
      // Keep the same native cost threshold and require both swept side cells
      // to be admissible in the selected slice. Do not weaken blocked cells
      // via the gateway exception (a gateway is a slice transition, not free
      // space on the side of a diagonal).
      if (neighbor[0] != 0 && neighbor[1] != 0 &&
          ((*cost_map_)(layer * max_y_ + i, current_node->idx[2]) > cost_threshold_ ||
           (*cost_map_)(layer * max_y_ + current_node->idx[1], j) > cost_threshold_)) {
        continue;
      }

      const int matrix_row = layer * max_y_ + i;
      if ((*cost_map_)(matrix_row, j) > cost_threshold_) {
        // A gateway annotates a real overlap; it is not permission to insert
        // a blocked cell into the global route. Slice changes are selected at
        // the current measured overlap in DecideLayer below.
        continue;
      }
      auto neighbor_node = GetOrCreateNode(layer, i, j);

      // if ((neighbor_node->cost > cost_threshold_) ||
      //     std::abs(neighbor_node->height - current_node->height) > 0.3) {
      //   continue;
      // }

      auto diff = neighbor_node->idx - current_node->idx;
      double step_cost = step_cost_weight_ * neighbor_node->cost;
      if (step_cost < 5) step_cost = 0.0;
      tentative_g =
          current_node->g +
          std::sqrt(diff[0] * diff[0] + diff[1] * diff[1] + diff[2] * diff[2]) +
          step_cost + (layer == current_node->layer ? 0.0 : 0.05);

      auto p_neighbor = closed_set.find(state_key(neighbor_node));
      if (p_neighbor != closed_set.end()) {
        if (tentative_g >= p_neighbor->second->g) {
          continue;
        }
      }

      if (tentative_g < neighbor_node->g) {
        if (neighbor_node->g == 1e9) {
          ++touched_count_;
        }
        neighbor_node->g = tentative_g;
        neighbor_node->f = tentative_g + GetHeuristic(neighbor_node, goal_node);
        neighbor_node->parent = current_node;
        open_set.push(neighbor_node);
      }
    }
    }
  }

  auto duration = std::chrono::duration_cast<std::chrono::microseconds>(
      std::chrono::high_resolution_clock::now() - t0);
  printf("path not found\n, time elapsed: %f ms\n",
         duration.count() / 1000.0);
  if (debug_) {
    ConvertClosedSetToMatrix(closed_set);
  }
  ReleaseQueryNodes();
  return false;
}

int Astar::DecideLayer(const Node* cur_node) const {
  int layer = cur_node->layer;
  int i = cur_node->idx[1];
  int j = cur_node->idx[2];
  double cur_height = cur_node->height;

  int true_layer = layer;

  for (const auto offset : search_layers_offset_) {
    int cur_layer = layer + offset;

    if (cur_layer < 0 || cur_layer >= max_layers_) {
      continue;
    }

    const int matrix_row = cur_layer * max_y_ + i;
    if (abs((*height_map_)(matrix_row, j) - cur_height) > 0.2) {
      continue;
    }

    if ((*ele_map_)(matrix_row, j) > 0.5) {
      const int next = std::min(cur_layer + 1, max_layers_ - 1);
      if ((*cost_map_)(next * max_y_ + i, j) <= cost_threshold_ &&
          std::abs((*height_map_)(next * max_y_ + i, j) - cur_height) <= 0.1) {
        true_layer = next;
        break;
      }
    } else if ((*ele_map_)(matrix_row, j) < -0.5) {
      const int next = std::max(cur_layer - 1, 0);
      if ((*cost_map_)(next * max_y_ + i, j) <= cost_threshold_ &&
          std::abs((*height_map_)(next * max_y_ + i, j) - cur_height) <= 0.1) {
        true_layer = next;
        break;
      }
    }
  }

  return true_layer;
}

double Astar::CalculateStepCost(const Node* node1, const Node* node2) const {}

double Astar::GetHeuristic(const Node* node1, const Node* node2) const {
  double cost = 0.0;

  if (h_type_ == kEuclidean) {
    // l2 distance
    cost = (node1->idx - node2->idx).norm();
  } else if (h_type_ == kDiagonal) {
    // octile distance
    Eigen::Vector3i d = node1->idx - node2->idx;
    int dx = abs(d(0)), dy = abs(d(1)), dz = abs(d(2));
    int dmin = std::min(dx, std::min(dy, dz));
    int dmax = std::max(dx, std::max(dy, dz));
    int dmid = dx + dy + dz - dmin - dmax;
    double h =
        std::sqrt(3) * dmin + std::sqrt(2) * (dmid - dmin) + (dmax - dmid);
    cost = h;
  } else if (h_type_ == kManhattan) {
    cost = (node1->idx - node2->idx).lpNorm<1>();
  } else {
    assert(false && "not implemented");
  }

  // cost += std::abs(node1->idx[0] - node2->idx[0]) * 10;
  return cost;
}

std::vector<PathPoint> Astar::GetPathPoints() const {
  std::vector<PathPoint> path_points;

  auto size = search_result_.size();
  path_points.resize(size);

  if (size == 0) {
    printf("path is empty\n, convert to path points failed\n");
    return path_points;
  }

  for (size_t i = 0; i < size; ++i) {
    // path_points[i].layer = search_result_[i][0];
    // path_points[i].x = search_result_[i][2];
    // path_points[i].y = search_result_[i][1];
    // if (i > 0) {
    //   path_points[i].heading =
    //       std::atan2(search_result_[i][1] - search_result_[i - 1][1],
    //                  search_result_[i][2] - search_result_[i - 1][2]);
    // }
    path_points[i].layer = search_result_[i].layer;
    path_points[i].x = search_result_[i].idx(2);
    path_points[i].y = search_result_[i].idx(1);
    path_points[i].height = search_result_[i].height;
    if (i > 0) {
      path_points[i].heading =
          std::atan2(search_result_[i].idx(1) - search_result_[i - 1].idx(1),
                     search_result_[i].idx(2) - search_result_[i - 1].idx(2));
    }
  }

  if (size > 1) {
    path_points[0].heading = path_points[1].heading;
  }

  return path_points;
}

Eigen::MatrixXd Astar::GetResultMatrix() const {
  if (search_result_.empty()) {
    printf("path is empty\n, convert to matrix failed\n");
    return Eigen::MatrixXd();
  }

  Eigen::MatrixXd path_matrix(search_result_.size(), 3);
  for (size_t i = 0; i < search_result_.size(); ++i) {
    path_matrix(i, 0) = search_result_[i].layer;
    path_matrix(i, 1) = search_result_[i].idx[1];
    path_matrix(i, 2) = search_result_[i].idx[2];
  }
  return path_matrix;
}

void Astar::ConvertClosedSetToMatrix(
    const std::unordered_map<int, Node*>& closed_set) {
  visited_set_ = Eigen::MatrixXi(closed_set.size(), 3);
  int count = 0;
  for (auto i = closed_set.begin(); i != closed_set.end(); ++i) {
    visited_set_(count, 0) = i->second->layer;
    visited_set_(count, 1) = i->second->idx[1];
    visited_set_(count, 2) = i->second->idx[2];
    count += 1;
  }
}

std::vector<Eigen::Vector3i> Astar::GetNeighbors(Node* node) const {}

Eigen::MatrixXd Astar::GetCostLayer(int layer) const {
  if (layer < 0 || layer >= max_layers_) throw std::out_of_range("Invalid A* layer");
  return cost_map_->middleRows(layer * max_y_, max_y_);
}
Eigen::MatrixXd Astar::GetEleLayer(int layer) const {
  if (layer < 0 || layer >= max_layers_) throw std::out_of_range("Invalid A* layer");
  return ele_map_->middleRows(layer * max_y_, max_y_);
}
