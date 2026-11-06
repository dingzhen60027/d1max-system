#ifndef _DYN_A_STAR_H_
#define _DYN_A_STAR_H_

#include <rclcpp/rclcpp.hpp>
#include <Eigen/Eigen>
#include <plan_env/grid_map.h>
#include <chrono>
#include <cstdint>
#include <functional>
#include <limits>
#include <optional>
#include <string>
#include <vector>
#include <path_searching/search_lattice.hpp>
#include <plan_env/solve_budget.hpp>

constexpr double inf=std::numeric_limits<double>::infinity();
struct GridNode;
using GridNodePtr=GridNode *;
enum ASTAR_RET {
  SUCCESS, INIT_ERR, SEARCH_ERR, INIT_START_OCCUPIED, INIT_TARGET_OCCUPIED,
  INIT_LATTICE_OCCUPIED, INIT_UNOBSERVED, INIT_OUTSIDE_MAP, INIT_CONNECTOR_COLLISION
};
// Only immutable solve boundaries need incident-edge heading recovery. A
// movable intermediate reference anchor retains chord rejection so its owner
// can first retreat/advance it using the established seed-selection policy.
struct ExactReferenceHeadingRecovery {
  bool start{false};
  bool target{false};
};
struct GridNode {
  enum enum_state { OPENSET=1,CLOSEDSET=2,UNDEFINED=3 };
  std::uint64_t rounds{0};
  enum_state state{UNDEFINED};
  Eigen::Vector3i index{Eigen::Vector3i::Zero()};
  double gScore{inf},fScore{inf};
  GridNodePtr cameFrom{nullptr};
  std::size_t heap_index{std::numeric_limits<std::size_t>::max()};
};

// One entry per node in a pre-reserved contiguous heap. Decrease-key does not
// accumulate stale entries or allocate an object for every relaxation.
class SearchOpenHeap {
 public:
  void reserve(std::size_t maximum) { maximum_=maximum;nodes_.reserve(maximum); }
  void clear();
  bool pushOrDecrease(GridNodePtr node);
  GridNodePtr pop();
  bool empty() const { return nodes_.empty(); }
  std::size_t size() const { return nodes_.size(); }
  std::size_t capacity() const { return nodes_.capacity(); }
 private:
  void swapNodes(std::size_t a,std::size_t b);
  void up(std::size_t index);
  void down(std::size_t index);
  std::vector<GridNodePtr> nodes_;
  std::size_t maximum_{0};
};

class AStar {
 private:
  GridMap::Ptr grid_map_;
  scan_planner::SolveBudget::Ptr solve_budget_;
  std::optional<double> frozen_body_yaw_;
  using SearchClock=std::chrono::steady_clock;
  using SearchNow=std::function<SearchClock::time_point()>;
  SearchNow search_now_;
  bool last_search_timed_out_{false},last_search_resource_limit_{false};
  std::string last_search_reason_;
  bool map_aligned_{false},initialized_{false};
  Eigen::Vector3d configured_map_size_{Eigen::Vector3d::Zero()};
  double configured_resolution_{0.};
  std::size_t peak_open_nodes_{0},expanded_nodes_{0},planning_query_count_{0};
  double getDiagHeu(GridNodePtr node1,GridNodePtr node2);
  double getManhHeu(GridNodePtr node1,GridNodePtr node2);
  double getEuclHeu(GridNodePtr node1,GridNodePtr node2);
  inline double getHeu(GridNodePtr node1,GridNodePtr node2);
  bool ConvertToIndexAndAdjustStartEndPoints(Eigen::Vector3d &start_pt,Eigen::Vector3d &end_pt,
      Eigen::Vector3i &start_idx,Eigen::Vector3i &end_idx,const Eigen::Vector3d &direction,
      int &remaining_shifts,const SearchClock::time_point &deadline);
  inline Eigen::Vector3d Index2Coord(const Eigen::Vector3i &index) const;
  inline bool Coord2Index(const Eigen::Vector3d &pt,Eigen::Vector3i &idx) const;
  inline int checkOccupancy(const Eigen::Vector3d &pos,double yaw) {
    ++planning_query_count_;
    return grid_map_->getPlanningInflateOccupancy(pos,frozen_body_yaw_.value_or(yaw));
  }
  std::vector<GridNodePtr> retrievePath(GridNodePtr current);
  void allocatePlanePool(const Eigen::Vector3i &size);
  bool alignToEnvironment(double step_size);
  GridNodePtr nodeAt(const Eigen::Vector3i &index) {
    return &plane_nodes_[static_cast<std::size_t>(index.x())*POOL_SIZE_.y()+index.y()];
  }
  double step_size_{0.},inv_step_size_{0.};
  Eigen::Vector3d center_{Eigen::Vector3d::Zero()};
  Eigen::Vector3i CENTER_IDX_{Eigen::Vector3i::Zero()},POOL_SIZE_{Eigen::Vector3i::Zero()};
  const double tie_breaker_=1.+1./10000.;
  std::vector<GridNodePtr> gridPath_;
  std::vector<Eigen::Vector3d> connected_reference_path_;
  // Native SCAN expands eight XY neighbours on its endpoint-derived varying-Z
  // SearchPlane. Every XY has exactly one Z in one search attempt. Preserve the
  // full XYZ index, without allocating unused Z nodes. This is not 26-neighbour
  // free-volume search and cannot infer an arbitrary terrain support surface.
  std::vector<GridNode> plane_nodes_;
  SearchOpenHeap openSet_;
  std::uint64_t rounds_{0};
 public:
  using Ptr=std::shared_ptr<AStar>;
  static constexpr std::size_t kMaximumPlaneNodes=262144;
  void setEnvironment(const GridMap::Ptr &map) { grid_map_=map; }
  explicit AStar(SearchNow now=[] {return SearchClock::now();}) : search_now_(std::move(now)) {}
  ~AStar()=default;
  // Historical callers retain their endpoint-midpoint lattice and dimensions.
  void initGridMap(GridMap::Ptr occ_map,const Eigen::Vector3i pool_size);
  // Production searches cover the exact GridMap window, not a second 100^3
  // configuration. This does not change the native 2.5D neighbour algorithm.
  void initGridMapFromEnvironment(GridMap::Ptr occ_map);
  void setSolveBudget(const scan_planner::SolveBudget::Ptr &budget) { solve_budget_=budget; }
  // One immutable solve's body orientation, not the edge/velocity bearing.
  // The owner explicitly resets this to nullopt on each path-tangent attempt.
  void setFrozenBodyYaw(std::optional<double> yaw) {
    if(yaw&&!std::isfinite(*yaw))throw std::invalid_argument("non-finite frozen search heading");
    frozen_body_yaw_=yaw;
  }
  bool lastSearchTimedOut() const { return last_search_timed_out_; }
  bool lastSearchResourceLimited() const { return last_search_resource_limit_; }
  const std::string &lastSearchReason() const { return last_search_reason_; }
  Eigen::Vector3i latticeSize() const { return POOL_SIZE_; }
  std::size_t nodeCapacity() const { return plane_nodes_.size(); }
  std::size_t storageBytes() const {
    return plane_nodes_.capacity()*sizeof(GridNode)+openSet_.capacity()*sizeof(GridNodePtr);
  }
  std::size_t peakOpenNodes() const { return peak_open_nodes_; }
  std::size_t expandedNodes() const { return expanded_nodes_; }
  std::size_t planningQueryCount() const { return planning_query_count_; }
  // The final flag retains its legacy ABI/name. It requests exact reference
  // anchors, not adjustable endpoints or a fictitious drive to voxel centres.
  // Exact direct/first/last edges use their actual bearing and native collision
  // sampling; ordinary interior edges keep the existing eight-neighbour search.
  ASTAR_RET AstarSearch(double step_size,Eigen::Vector3d start_pt,Eigen::Vector3d end_pt,
      bool adjust_endpoints=true,bool recover_reference_lattice=false);
  ASTAR_RET AstarSearch(double step_size,Eigen::Vector3d start_pt,Eigen::Vector3d end_pt,
      bool adjust_endpoints,bool recover_reference_lattice,ExactReferenceHeadingRecovery heading_recovery);
  std::vector<Eigen::Vector3d> getPath();
};
inline double AStar::getHeu(GridNodePtr node1,GridNodePtr node2) {
  return tie_breaker_*getDiagHeu(node1,node2);
}
inline Eigen::Vector3d AStar::Index2Coord(const Eigen::Vector3i &index) const {
  return (index-CENTER_IDX_).cast<double>()*step_size_+center_;
}
inline bool AStar::Coord2Index(const Eigen::Vector3d &pt,Eigen::Vector3i &idx) const {
  if (!scan_planner::nearestSearchIndex(pt,center_,step_size_,CENTER_IDX_,POOL_SIZE_,idx)) {
    RCLCPP_ERROR(rclcpp::get_logger("path_searching"),
        "Search endpoint outside bounded lattice: point=(%.4f %.4f %.4f)",pt.x(),pt.y(),pt.z());
    return false;
  }
  return true;
}
#endif
