#include "path_searching/dyn_a_star.h"
#include <algorithm>
#include <chrono>
#include <cmath>
#include <unordered_map>

using namespace std;
using namespace Eigen;

namespace {
bool insideSearchInterior(const Vector3i &index,const Vector3i &size) {
    return (index.array()>=1).all() && (index.array()<size.array()-1).all();
}

// One definition of the official endpoint-derived search plane for anchor
// viability and the actual search. In particular, retain its Z interpolation.
struct SearchPlane {
    Vector3d start,end,center;
    Vector3i center_index;
    Vector2d xy_delta;
    double step,inv_step,xy_length2;
    int start_z;
    GridMap *support_map;
    Vector3i support_start_index{Vector3i::Zero()},support_end_index{Vector3i::Zero()};
    mutable std::unordered_map<std::uint64_t,int> support_z;
    SearchPlane(const Vector3d &a,const Vector3d &b,const Vector3d &origin,
                const Vector3i &origin_index,double resolution,int z_index,GridMap *map=nullptr)
        : start(a),end(b),center(origin),center_index(origin_index),
          xy_delta(b.head<2>()-a.head<2>()),step(resolution),inv_step(1./resolution),
          xy_length2(xy_delta.squaredNorm()),start_z(z_index),
          support_map(map&&map->hasQuerySupportHeightProvider()?map:nullptr) {
        if(support_map) {
            support_start_index=((start-center)*inv_step).array().round().cast<int>()+center_index.array();
            support_end_index=((end-center)*inv_step).array().round().cast<int>()+center_index.array();
        }
    }
    int zIndex(int x,int y) const {
        if (xy_length2<1e-8) return start_z;
        const Vector2d xy((x-center_index.x())*step+center.x(),
                          (y-center_index.y())*step+center.y());
        const double ratio=std::clamp((xy-start.head<2>()).dot(xy_delta)/xy_length2,0.,1.);
        const double z=start.z()+ratio*(end.z()-start.z());
        const int original=static_cast<int>(std::round((z-center.z())*inv_step))+center_index.z();
        if(!support_map)return original; // Exact historical flat search.
        // The source model guides interior candidates only. Real/reference
        // endpoints and the already collision-checked rebound anchors are
        // not moved onto a different fitted elevation.
        if(x==support_start_index.x()&&y==support_start_index.y())return support_start_index.z();
        if(x==support_end_index.x()&&y==support_end_index.y())return support_end_index.z();
        const auto key=(std::uint64_t(static_cast<std::uint32_t>(x))<<32)|static_cast<std::uint32_t>(y);
        const auto found=support_z.find(key);if(found!=support_z.end())return found->second;
        // One deterministic height per XY preserves the native compact node
        // pool. Only a fit from literal source points associated in XYZ with
        // this SAME endpoint-plane level may guide Z. Missing support keeps the original
        // plane; it is not reclassified as an obstacle or observed-free space.
        const auto supported=support_map->querySupportBodyHeight({xy.x(),xy.y(),z});
        const int result=supported&&std::isfinite(*supported)?
            static_cast<int>(std::round((*supported-center.z())*inv_step))+center_index.z():original;
        support_z.emplace(key,result);return result;
    }
};

struct SearchEdgeCheck {
    int occupancy=0,sample=-1,samples=0;
    Vector3d point=Vector3d::Zero();
    bool interrupted=false;
};

// Preserve the existing destination-first query and complete edge sampling.
// sample=-1 identifies a rejected destination; >=0 identifies an edge sample.
SearchEdgeCheck checkSearchEdge(GridMap &map,const Vector3d &from,const Vector3d &to,
                               double yaw,double step,const std::function<bool()> &allowed,
                               std::size_t &query_count,std::optional<double> frozen_yaw={}) {
    SearchEdgeCheck result;
    yaw=frozen_yaw.value_or(yaw);
    if (!allowed()) {result.interrupted=true;return result;}
    result.point=to;++query_count;result.occupancy=map.getPlanningInflateOccupancy(to,yaw);
    if (result.occupancy!=0) return result;
    result.samples=std::max(1,static_cast<int>(std::ceil((to-from).norm()/(step*.5))));
    for (result.sample=0;result.sample<=result.samples;++result.sample) {
        if (!allowed()) {result.interrupted=true;return result;}
        result.point=scan_planner::searchEdgeSample(from,to,result.sample,result.samples);
        ++query_count;
        result.occupancy=map.getPlanningInflateOccupancy(result.point,yaw);
        if (result.occupancy!=0) return result;
    }
    return result;
}

Vector3d searchCoordinate(const Vector3i &index,const SearchPlane &plane) {
    return (index-plane.center_index).cast<double>()*plane.step+plane.center;
}

// Shared by internal adjustable anchors and reference-only lattice connectors.
// Both test the very same eight directed, fully sampled edges as the search.
bool hasSearchAnchorEdge(GridMap &map,const Vector3i &anchor,bool outgoing,
                         const SearchPlane &plane,const Vector3i &size,
                         const std::chrono::steady_clock::time_point &deadline,
                         const std::function<std::chrono::steady_clock::time_point()> &now,
                         const scan_planner::SolveBudget::Ptr &budget,
                         std::size_t &query_count,std::optional<double> frozen_yaw={}) {
    if (!outgoing && !insideSearchInterior(anchor,size)) return false;
    const auto allowed=[&] {return scan_planner::solveAllowed(budget)&&now()<deadline;};
    for (int dx=-1;dx<=1;++dx) for (int dy=-1;dy<=1;++dy) {
        if (!allowed()) return false;
        if (dx==0 && dy==0) continue;
        Vector3i neighbor(anchor.x()+dx,anchor.y()+dy,0);
        neighbor.z()=plane.zIndex(neighbor.x(),neighbor.y());
        if (!insideSearchInterior(neighbor,size)) continue;
        if (!outgoing && plane.zIndex(anchor.x(),anchor.y())!=anchor.z()) continue;
        const double yaw=std::atan2(static_cast<double>(outgoing?dy:-dy),
                                    static_cast<double>(outgoing?dx:-dx));
        const Vector3d a=searchCoordinate(anchor,plane),b=searchCoordinate(neighbor,plane);
        const auto edge=outgoing ? checkSearchEdge(map,a,b,yaw,plane.step,allowed,query_count,frozen_yaw) :
                                   checkSearchEdge(map,b,a,yaw,plane.step,allowed,query_count,frozen_yaw);
        if (edge.interrupted) return false;
        if (edge.occupancy==0) return true;
    }
    return false;
}

// A reference anchor is a position, not a requirement to orient the body
// toward the distant target. In a narrow entrance that chord can cross a
// wall although the real first/last edge fits. Admit only a fully checked
// incident edge using the SAME exact anchor and support plane as A* below.
bool hasExactReferenceAnchorEdge(GridMap &map,const Vector3i &anchor,
                                const Vector3d &anchor_position,
                                const Vector3i &peer,const Vector3d &peer_position,
                                bool outgoing,const SearchPlane &plane,const Vector3i &size,
                                const std::function<bool()> &allowed,
                                std::size_t &query_count,std::optional<double> frozen_yaw={}) {
    if(!insideSearchInterior(anchor,size))return false;
    for(int dx=-1;dx<=1;++dx)for(int dy=-1;dy<=1;++dy) {
        if(!allowed())return false;
        if(dx==0&&dy==0)continue;
        Vector3i neighbor(anchor.x()+dx,anchor.y()+dy,0);
        neighbor.z()=plane.zIndex(neighbor.x(),neighbor.y());
        if(neighbor.head<2>()==peer.head<2>())neighbor.z()=peer.z();
        if(!insideSearchInterior(neighbor,size))continue;
        const Vector3d other=neighbor==peer?peer_position:searchCoordinate(neighbor,plane);
        const Vector3d from=outgoing?anchor_position:other;
        const Vector3d to=outgoing?other:anchor_position;
        const Vector3d delta=to-from;
        const auto edge=checkSearchEdge(map,from,to,std::atan2(delta.y(),delta.x()),
            plane.step,allowed,query_count,frozen_yaw);
        if(edge.interrupted)return false;
        if(edge.occupancy==0)return true;
    }
    return false;
}
} // namespace

void SearchOpenHeap::swapNodes(std::size_t a,std::size_t b) {
    std::swap(nodes_[a],nodes_[b]);nodes_[a]->heap_index=a;nodes_[b]->heap_index=b;
}
void SearchOpenHeap::up(std::size_t index) {
    while(index>0) {
        const auto parent=(index-1)/2;
        if(nodes_[parent]->fScore<=nodes_[index]->fScore) break;
        swapNodes(parent,index);index=parent;
    }
}
void SearchOpenHeap::down(std::size_t index) {
    for(;;) {
        auto child=2*index+1;if(child>=nodes_.size()) break;
        if(child+1<nodes_.size()&&nodes_[child+1]->fScore<nodes_[child]->fScore) ++child;
        if(nodes_[index]->fScore<=nodes_[child]->fScore) break;
        swapNodes(index,child);index=child;
    }
}
void SearchOpenHeap::clear() {
    for(auto *node:nodes_) node->heap_index=std::numeric_limits<std::size_t>::max();
    nodes_.clear();
}
bool SearchOpenHeap::pushOrDecrease(GridNodePtr node) {
    if(!node) return false;
    if(node->heap_index!=std::numeric_limits<std::size_t>::max()) {
        if(node->heap_index>=nodes_.size()||nodes_[node->heap_index]!=node) return false;
        up(node->heap_index);return true;
    }
    if(nodes_.size()>=maximum_) return false;
    node->heap_index=nodes_.size();nodes_.push_back(node);up(node->heap_index);return true;
}
GridNodePtr SearchOpenHeap::pop() {
    if(nodes_.empty()) return nullptr;
    auto *result=nodes_.front();
    result->heap_index=std::numeric_limits<std::size_t>::max();
    if(nodes_.size()==1) {nodes_.pop_back();return result;}
    nodes_.front()=nodes_.back();nodes_.pop_back();nodes_.front()->heap_index=0;down(0);return result;
}

void AStar::allocatePlanePool(const Vector3i &size) {
    if((size.array()<3).any()||(size.array()>65535).any())
        throw std::invalid_argument("AStar lattice dimensions must be in [3,65535]");
    const auto count=static_cast<std::size_t>(size.x())*static_cast<std::size_t>(size.y());
    POOL_SIZE_=size;CENTER_IDX_=size/2;
    if(count>kMaximumPlaneNodes) {
        last_search_resource_limit_=true;last_search_reason_="plane_node_resource_limit";
        // Never silently shrink a configured map. A search will return a
        // resource failure instead of interpreting the truncated box as map.
        return;
    }
    plane_nodes_.resize(count);openSet_.reserve(count);
}

void AStar::initGridMap(GridMap::Ptr occ_map, const Eigen::Vector3i pool_size)
{
    if(initialized_) throw std::logic_error("AStar node pool is initialized once; use setEnvironment");
    initialized_=true;allocatePlanePool(pool_size);grid_map_=occ_map;
}

void AStar::initGridMapFromEnvironment(GridMap::Ptr occ_map) {
    if(initialized_) throw std::logic_error("AStar node pool is initialized once; use setEnvironment");
    if(!occ_map) throw std::invalid_argument("AStar map-aligned initialization needs GridMap");
    Vector3d origin;occ_map->getRegion(origin,configured_map_size_);
    configured_resolution_=occ_map->getResolution();
    if(!origin.allFinite()||!configured_map_size_.allFinite()||
       (configured_map_size_.array()<=0.).any()||!std::isfinite(configured_resolution_)||
       configured_resolution_<=0.) throw std::invalid_argument("invalid AStar source map window");
    Vector3i cells;
    for(int axis=0;axis<3;++axis) {
        const double count=configured_map_size_[axis]/configured_resolution_;
        if(!std::isfinite(count)||count<1.||count>65533.)
            throw std::invalid_argument("invalid AStar source map voxel dimensions");
        cells[axis]=static_cast<int>(std::ceil(count-1e-9));
    }
    map_aligned_=initialized_=true;grid_map_=std::move(occ_map);
    allocatePlanePool(cells+Vector3i::Constant(2));
    RCLCPP_INFO(rclcpp::get_logger("path_searching"),
        "A* map-aligned native varying-Z plane: lattice=(%d %d %d) capacity=%zu storage_bytes=%zu resource_limit=%s",
        POOL_SIZE_.x(),POOL_SIZE_.y(),POOL_SIZE_.z(),nodeCapacity(),storageBytes(),
        last_search_resource_limit_?"true":"false");
}

bool AStar::alignToEnvironment(double step_size) {
    Vector3d origin,size;grid_map_->getRegion(origin,size);
    if(!origin.allFinite()||!size.allFinite()||
       (size-configured_map_size_).cwiseAbs().maxCoeff()>1e-9||
       std::abs(step_size-configured_resolution_)>1e-9||
       std::abs(grid_map_->getResolution()-configured_resolution_)>1e-9) {
        last_search_reason_="map_window_or_resolution_changed";return false;
    }
    // Interior indices 1..N are precisely the physical map voxel centres.
    // Sentinel cells fall outside the map. A rolling snapshot re-anchors only
    // between searches; no growing pool or per-search volume allocation.
    center_=origin+(CENTER_IDX_.cast<double>()-Vector3d::Constant(.5))*step_size;
    return true;
}

double AStar::getDiagHeu(GridNodePtr node1, GridNodePtr node2)
{
    double dx = abs(node1->index(0) - node2->index(0));
    double dy = abs(node1->index(1) - node2->index(1));
    double dz = abs(node1->index(2) - node2->index(2));

    double h = 0.0;
    int diag = min(min(dx, dy), dz);
    dx -= diag;
    dy -= diag;
    dz -= diag;

    if (dx == 0)
    {
        h = 1.0 * sqrt(3.0) * diag + sqrt(2.0) * min(dy, dz) + 1.0 * abs(dy - dz);
    }
    if (dy == 0)
    {
        h = 1.0 * sqrt(3.0) * diag + sqrt(2.0) * min(dx, dz) + 1.0 * abs(dx - dz);
    }
    if (dz == 0)
    {
        h = 1.0 * sqrt(3.0) * diag + sqrt(2.0) * min(dx, dy) + 1.0 * abs(dx - dy);
    }
    return h;
}

double AStar::getManhHeu(GridNodePtr node1, GridNodePtr node2)
{
    double dx = abs(node1->index(0) - node2->index(0));
    double dy = abs(node1->index(1) - node2->index(1));
    double dz = abs(node1->index(2) - node2->index(2));

    return dx + dy + dz;
}

double AStar::getEuclHeu(GridNodePtr node1, GridNodePtr node2)
{
    return (node2->index - node1->index).norm();
}

vector<GridNodePtr> AStar::retrievePath(GridNodePtr current)
{
    vector<GridNodePtr> path;
    path.push_back(current);

    while (current->cameFrom != NULL)
    {
        current = current->cameFrom;
        path.push_back(current);
    }

    return path;
}

bool AStar::ConvertToIndexAndAdjustStartEndPoints(Vector3d &start_pt, Vector3d &end_pt,
    Vector3i &start_idx,Vector3i &end_idx,const Vector3d &start_to_end,
    int &remaining_shifts,const std::chrono::steady_clock::time_point &deadline)
{
    // These are optimizer-internal rebound helpers, NOT acquired body or
    // reference endpoints. Interior A* nodes already use the finite same-level
    // support; extending the helpers along a steep control-point chord while
    // keeping its extrapolated Z could instead send them under that surface
    // (or above the map) and exhaust the whole solve. Use the identical source
    // association for each helper before its normal native collision query.
    // No source/invalid fit retains the exact historical adjustment, and no
    // occupancy or measured endpoint is changed by this geometric guide.
    const auto supported_index=[&](Vector3d& point,Vector3i& index) {
        if(grid_map_->hasQuerySupportHeightProvider()) {
            const auto height=grid_map_->querySupportBodyHeight(point);
            if(height&&std::isfinite(*height))point.z()=*height;
        }
        return Coord2Index(point,index);
    };
    if (!supported_index(start_pt, start_idx) || !supported_index(end_pt, end_idx))
        return false;

    const double path_yaw = frozen_body_yaw_.value_or(std::atan2(start_to_end(1), start_to_end(0)));

    // Internal rebound anchors may be free at the continuous path heading but
    // trapped for all eight lattice directions. Extend only these adjustable
    // helper anchors, not an external trajectory start or reference endpoint.
    // All anchor shifts and search retries share the caller's budgets.
    const auto shift_start=[&]() {
        if (remaining_shifts<=0) return false;
        --remaining_shifts;
        start_pt-=start_to_end*step_size_;
        return supported_index(start_pt,start_idx);
    };
    const auto shift_end=[&]() {
        if (remaining_shifts<=0) return false;
        --remaining_shifts;
        end_pt+=start_to_end*step_size_;
        return supported_index(end_pt,end_idx);
    };
    for (;;) {
        if (!scan_planner::solveAllowed(solve_budget_) || search_now_()>=deadline) return false;
        const int start_occ=checkOccupancy(Index2Coord(start_idx),path_yaw);
        const int end_occ=checkOccupancy(Index2Coord(end_idx),path_yaw);
        if (start_occ==-1 || end_occ==-1) return false;
        if (start_occ!=0) {if (!shift_start()) return false;continue;}
        if (end_occ!=0) {if (!shift_end()) return false;continue;}

        const SearchPlane plane(Index2Coord(start_idx),Index2Coord(end_idx),
                                center_,CENTER_IDX_,step_size_,start_idx.z(),grid_map_.get());
        if (!hasSearchAnchorEdge(*grid_map_,start_idx,true,plane,POOL_SIZE_,deadline,search_now_,solve_budget_,planning_query_count_,frozen_body_yaw_)) {
            if (!shift_start()) return false;
            continue;
        }
        if (!hasSearchAnchorEdge(*grid_map_,end_idx,false,plane,POOL_SIZE_,deadline,search_now_,solve_budget_,planning_query_count_,frozen_body_yaw_)) {
            if (!shift_end()) return false;
            continue;
        }
        return true;
    }
}

ASTAR_RET AStar::AstarSearch(const double step_size, Vector3d start_pt, Vector3d end_pt,
                           bool adjust_endpoints,bool recover_reference_lattice)
{
    return AstarSearch(step_size,start_pt,end_pt,adjust_endpoints,recover_reference_lattice,{});
}

ASTAR_RET AStar::AstarSearch(const double step_size, Vector3d start_pt, Vector3d end_pt,
                           bool adjust_endpoints,bool recover_reference_lattice,
                           ExactReferenceHeadingRecovery heading_recovery)
{
    last_search_timed_out_=false;last_search_resource_limit_=false;
    last_search_reason_.clear();peak_open_nodes_=expanded_nodes_=planning_query_count_=0;
    const auto time_1 = search_now_();
    // Formal solves share the ORIGINAL absolute search/optimization/validation
    // deadline. Do not add a second shorter quota, or renew a consumed budget.
    // Only historical standalone callers keep their original 200 ms bound.
    const auto deadline=solve_budget_ ? solve_budget_->deadline() :
        time_1+std::chrono::milliseconds(200);
    gridPath_.clear();
    connected_reference_path_.clear();
    openSet_.clear();
    const auto search_failure=[&]() {
        // Cancellation is never recoverable as a compute timeout. Preserve
        // ASTAR_RET compatibility while distinguishing an exhausted quota
        // from a geometrically exhausted open set for the planning owner.
        const auto stop=solve_budget_?solve_budget_->stop():scan_planner::SolveBudget::Stop::None;
        last_search_timed_out_=stop!=scan_planner::SolveBudget::Stop::Cancelled &&
            (stop==scan_planner::SolveBudget::Stop::Deadline || search_now_()>=deadline);
        if(last_search_reason_.empty()) last_search_reason_=
            stop==scan_planner::SolveBudget::Stop::Cancelled?
                "solve_cancelled":last_search_timed_out_?"search_deadline_exceeded":"open_set_exhausted";
        return ASTAR_RET::SEARCH_ERR;
    };
    if (!scan_planner::solveAllowed(solve_budget_)) return search_failure();
    if (!std::isfinite(step_size) || step_size <= 0. ||
        !start_pt.allFinite() || !end_pt.allFinite() ||
        (adjust_endpoints && recover_reference_lattice)||!initialized_||!grid_map_)
        return ASTAR_RET::INIT_ERR;
    if(plane_nodes_.empty()) {
        last_search_resource_limit_=true;last_search_reason_="plane_node_resource_limit";
        return search_failure();
    }
    step_size_ = step_size;
    inv_step_size_ = 1 / step_size;
    if(map_aligned_) {
        if(!alignToEnvironment(step_size)) return ASTAR_RET::INIT_ERR;
        if(!adjust_endpoints&&(!grid_map_->isInMap(start_pt)||!grid_map_->isInMap(end_pt))) {
            last_search_reason_="endpoint_outside_source_map";
            return ASTAR_RET::INIT_OUTSIDE_MAP;
        }
    } else center_=(start_pt+end_pt)/2;
    const auto allowed=[&] {return scan_planner::solveAllowed(solve_budget_)&&search_now_()<deadline;};

    const Vector3d original_delta=end_pt-start_pt;
    if (adjust_endpoints && original_delta.norm()<1e-6) return ASTAR_RET::INIT_ERR;
    const Vector3d adjustment_direction=original_delta.norm()>0. ?
        Vector3d(original_delta.normalized()):Vector3d::Zero();
    // A straight line crosses at most sum(pool dimensions) cells per endpoint.
    // The same budget covers initial adjustment and every exhausted-search retry.
    int remaining_shifts=2*POOL_SIZE_.sum()+4;
    bool searched=false;
    // Reference endpoints are actual robot/reference poses, not extra motions
    // to a voxel centre. Use their existing lattice nodes as exact anchors.
    // The first/last REAL edge is queried and returned with the same geometry
    // and bearing; a millimetre snap cannot invent a ninety-degree body turn.
    const bool exact_reference=!adjust_endpoints && recover_reference_lattice;
    if(exact_reference && original_delta.norm()<=1e-6) {
        // The reference consumer requires a nonzero segment. Two identical
        // anchors are not a usable route and must not reuse the last result.
        last_search_reason_="zero_length_reference";
        return ASTAR_RET::INIT_ERR;
    }
    for (int attempt=0;;++attempt) {
    if (!scan_planner::solveAllowed(solve_budget_) || search_now_()>=deadline) return search_failure();
    Vector3i start_idx, end_idx;
    const double endpoint_yaw=frozen_body_yaw_.value_or(std::atan2(end_pt.y()-start_pt.y(), end_pt.x()-start_pt.x()));
    bool endpoints_valid=false;
    bool start_admissible=false,end_admissible=false;
    int exact_start=3,exact_end=3,grid_start=3,grid_end=3;
    if (adjust_endpoints) {
        endpoints_valid=ConvertToIndexAndAdjustStartEndPoints(start_pt,end_pt,start_idx,end_idx,
            adjustment_direction,remaining_shifts,deadline);
    } else if (Coord2Index(start_pt, start_idx) && Coord2Index(end_pt, end_idx)) {
        // Do not short-circuit these four bounded queries: distinguish an
        // actual blocked endpoint from a lattice or missing-observation issue.
        exact_start=checkOccupancy(start_pt,endpoint_yaw);
        exact_end=checkOccupancy(end_pt,endpoint_yaw);
        grid_start=checkOccupancy(Index2Coord(start_idx),endpoint_yaw);
        grid_end=checkOccupancy(Index2Coord(end_idx),endpoint_yaw);
        start_admissible=exact_start==0;end_admissible=exact_end==0;
        // A replaced lattice centre is diagnostic only. It is not traversed.
        // Every real edge from/to the exact anchor still queries the native
        // GridMap, including the whole oriented box and strict-unknown policy.
        endpoints_valid=start_admissible && end_admissible &&
            (exact_reference || (grid_start==0 && grid_end==0));
        if(exact_reference&&!endpoints_valid&&start_idx!=end_idx) {
            const SearchPlane plane(start_pt,end_pt,center_,CENTER_IDX_,step_size_,
                start_idx.z(),grid_map_.get());
            start_admissible=start_admissible||(heading_recovery.start&&hasExactReferenceAnchorEdge(
                *grid_map_,start_idx,start_pt,end_idx,end_pt,true,plane,POOL_SIZE_,
                allowed,planning_query_count_,frozen_body_yaw_));
            end_admissible=end_admissible||(heading_recovery.target&&hasExactReferenceAnchorEdge(
                *grid_map_,end_idx,end_pt,start_idx,start_pt,false,plane,POOL_SIZE_,
                allowed,planning_query_count_,frozen_body_yaw_));
            if(!allowed())return search_failure();
            endpoints_valid=start_admissible&&end_admissible;
        }
    }
    if (!endpoints_valid)
    {
        if (searched || !allowed()) {
            RCLCPP_WARN(rclcpp::get_logger("path_searching"),
                "A-star internal retry stopped: attempt=%d remaining_shifts=%d total_seconds=%.6f",
                attempt,remaining_shifts,
                std::chrono::duration<double>(search_now_()-time_1).count());
            return search_failure();
        }
        if (!adjust_endpoints && exact_start!=3 && grid_map_->planningRequiresObservedFree()) {
            // Failure semantics must not mistake the first unknown cell for
            // an entirely empty envelope. Inspect all endpoint cells only on
            // this strict-policy failure path; occupied takes precedence.
            // In official mode these are raw-evidence diagnostics, not the
            // inflated-center predicate. Preserve the actual query results.
            exact_start=grid_map_->inspectInflateOccupancy(start_pt,endpoint_yaw).state();
            exact_end=grid_map_->inspectInflateOccupancy(end_pt,endpoint_yaw).state();
            grid_start=grid_map_->inspectInflateOccupancy(Index2Coord(start_idx),endpoint_yaw).state();
            grid_end=grid_map_->inspectInflateOccupancy(Index2Coord(end_idx),endpoint_yaw).state();
        }
        const auto now=search_now_();
        static thread_local std::chrono::steady_clock::time_point last_endpoint_warning{};
        if (now-last_endpoint_warning>=std::chrono::seconds(2)) {
            last_endpoint_warning=now;
            RCLCPP_WARN(rclcpp::get_logger("path_searching"),
                "A* endpoint rejected: start=(%.4f %.4f %.4f) exact=%s lattice=%s; "
                "end=(%.4f %.4f %.4f) exact=%s lattice=%s; yaw=%.4f resolution=%.3f",
                start_pt.x(),start_pt.y(),start_pt.z(),scan_planner::occupancyStateName(exact_start),
                scan_planner::occupancyStateName(grid_start),end_pt.x(),end_pt.y(),end_pt.z(),
                scan_planner::occupancyStateName(exact_end),scan_planner::occupancyStateName(grid_end),
                endpoint_yaw,step_size_);
            RCLCPP_WARN(rclcpp::get_logger("path_searching"),"A* start evidence: %s; end evidence: %s",
                grid_map_->describeInflateOccupancy(start_pt,endpoint_yaw).c_str(),
                grid_map_->describeInflateOccupancy(end_pt,endpoint_yaw).c_str());
        }
        // A chord-only rejection at an anchor with a proven incident edge
        // must not blame that anchor when the OTHER endpoint is blocked.
        if (!start_admissible && exact_start==1) return ASTAR_RET::INIT_START_OCCUPIED;
        if (!end_admissible && exact_end==1) return ASTAR_RET::INIT_TARGET_OCCUPIED;
        if (!exact_reference && (grid_start==1 || grid_end==1)) return ASTAR_RET::INIT_LATTICE_OCCUPIED;
        if ((!start_admissible && exact_start==-1) || (!end_admissible && exact_end==-1) ||
            (!exact_reference && (grid_start==-1 || grid_end==-1)))
            return ASTAR_RET::INIT_OUTSIDE_MAP;
        if ((!start_admissible && exact_start==2) || (!end_admissible && exact_end==2) ||
            (!exact_reference && (grid_start==2 || grid_end==2)))
            return ASTAR_RET::INIT_UNOBSERVED;
        return ASTAR_RET::INIT_ERR;
    }

    if((!exact_reference && start_idx.head<2>()==end_idx.head<2>()&&start_idx.z()!=end_idx.z()) ||
       (exact_reference && (start_pt-end_pt).head<2>().norm()<1e-9 &&
        std::abs(start_pt.z()-end_pt.z())>1e-9)) {
        // The native eight-neighbour plane cannot travel vertically with no
        // horizontal segment. Exact nonzero-XY slopes can alias to one XY
        // cell while crossing Z cells; validate their literal edge below
        // instead of mistaking quantization for a purely vertical request.
        last_search_reason_="vertical_only_search_unsupported";
        return ASTAR_RET::SEARCH_ERR;
    }

    ++rounds_;
    searched=true;
    const Eigen::Vector3d search_start = exact_reference ? start_pt:Index2Coord(start_idx);
    const Eigen::Vector3d search_end = exact_reference ? end_pt:Index2Coord(end_idx);
    const auto node_coordinate=[&](const Vector3i &index) -> Vector3d {
        if(exact_reference && index==start_idx)return start_pt;
        if(exact_reference && index==end_idx)return end_pt;
        return Index2Coord(index);
    };
    if(exact_reference) {
        // The exact straight edge is a legitimate search edge too. A free
        // narrow corridor must not require first turning a long body toward
        // one of eight quantized voxel bearings. Check every point with the
        // native body predicate; a blocked direct edge still goes to A* below.
        // In one cell this also avoids driving to the centre and back.
        const auto edge=checkSearchEdge(*grid_map_,start_pt,end_pt,endpoint_yaw,
            step_size_,allowed,planning_query_count_,frozen_body_yaw_);
        if(edge.interrupted || !allowed())return search_failure();
        if(edge.occupancy==0) {
            connected_reference_path_={start_pt,end_pt};last_search_reason_="success";
            return ASTAR_RET::SUCCESS;
        }
        if(start_idx.head<2>()==end_idx.head<2>()) {
            // A blocked same-XY edge (including different Z cells) cannot be
            // recovered by this compact XY pool. Keep its actual native
            // collision/support result; never alias its endpoints to success.
            last_search_reason_="exact_reference_short_edge_blocked";
            RCLCPP_WARN(rclcpp::get_logger("path_searching"),
                "A* exact reference connector rejected: point=(%.6f %.6f %.6f) yaw=%.6f state=%d sample=%d/%d",
                edge.point.x(),edge.point.y(),edge.point.z(),endpoint_yaw,edge.occupancy,edge.sample,edge.samples);
            return edge.occupancy==2 ? ASTAR_RET::INIT_UNOBSERVED:
                edge.occupancy==-1 ? ASTAR_RET::INIT_OUTSIDE_MAP:ASTAR_RET::INIT_CONNECTOR_COLLISION;
        }
    }
    const SearchPlane plane(search_start,search_end,center_,CENTER_IDX_,step_size_,start_idx.z(),grid_map_.get());

    // if ( start_pt(0) > -1 && start_pt(0) < 0 )
    //     cout << "start_pt=" << start_pt.transpose() << " end_pt=" << end_pt.transpose() << endl;

    // Keep the target descriptor distinct even for equal XY / different Z.
    // Compact storage must never convert an unsupported vertical-only route
    // into the zero-length success of an accidentally aliased target node.
    GridNodePtr startPtr=nodeAt(start_idx);
    GridNode end_node;GridNodePtr endPtr=&end_node;
    openSet_.clear();

    GridNodePtr neighborPtr = NULL;
    GridNodePtr current = NULL;

    endPtr->index = end_idx;

    startPtr->index = start_idx;
    startPtr->rounds = rounds_;
    startPtr->gScore = 0;
    startPtr->fScore = getHeu(startPtr, endPtr);
    startPtr->state = GridNode::OPENSET; //put start node in open set
    startPtr->cameFrom = NULL;
    if(!openSet_.pushOrDecrease(startPtr)) {
        last_search_resource_limit_=true;last_search_reason_="open_heap_resource_limit";
        return search_failure();
    }
    peak_open_nodes_=std::max(peak_open_nodes_,openSet_.size());

    double tentative_gScore;

    int num_iter = 0;
    // Failure-only diagnostics. Counts do not alter search, endpoint adjustment,
    // collision predicates, sampling, queue ordering or its time budget.
    std::size_t rejected_pool=0,rejected_closed=0,rejected_neighbor=0;
    std::size_t rejected_edge_start=0,rejected_edge_inside=0,rejected_edge_end=0;
    std::size_t admitted_edges=0,start_admitted_edges=0;
    std::size_t start_occupied_edges=0,start_unknown_edges=0,start_outside_edges=0;
    Eigen::Vector3d first_blocked_point=Eigen::Vector3d::Zero();
    Eigen::Vector3d first_blocked_from=Eigen::Vector3d::Zero(),first_blocked_to=Eigen::Vector3d::Zero();
    double first_blocked_yaw=0.;int first_blocked_state=0;
    int first_blocked_sample=-1,first_blocked_samples=0;
    const auto record_blocked=[&](const Vector3d &from,const Vector3d &to,const SearchEdgeCheck &edge,double yaw) {
        if (first_blocked_state==0) {
            first_blocked_point=edge.point;first_blocked_yaw=yaw;first_blocked_state=edge.occupancy;
            first_blocked_from=from;first_blocked_to=to;
            first_blocked_sample=edge.sample;first_blocked_samples=edge.samples;
        }
    };
    const auto report_exhaustion=[&](const char *reason) {
        RCLCPP_WARN(rclcpp::get_logger("path_searching"),
            "A-star %s: lattice=(%d %d %d) capacity=%zu bytes=%zu peak_open=%zu expanded=%zu queries=%zu "
            "adjust_endpoints=%s exact_reference_anchors=%s attempt=%d iter=%d adjusted_start=(%.6f %.6f %.6f) "
            "adjusted_end=(%.6f %.6f %.6f) admitted=%zu start_admitted=%zu "
            "reject(pool=%zu closed=%zu neighbor=%zu edge_start=%zu edge_inside=%zu edge_end=%zu) "
            "first_blocked=(%.6f %.6f %.6f) yaw=%.6f state=%d "
            "edge_from=(%.6f %.6f %.6f) edge_to=(%.6f %.6f %.6f) sample=%d/%d",
            reason,POOL_SIZE_.x(),POOL_SIZE_.y(),POOL_SIZE_.z(),nodeCapacity(),storageBytes(),peak_open_nodes_,expanded_nodes_,planning_query_count_,
            adjust_endpoints?"true":"false",exact_reference?"true":"false",attempt,num_iter,
            search_start.x(),search_start.y(),search_start.z(),
            search_end.x(),search_end.y(),search_end.z(),admitted_edges,start_admitted_edges,
            rejected_pool,rejected_closed,rejected_neighbor,rejected_edge_start,
            rejected_edge_inside,rejected_edge_end,first_blocked_point.x(),
            first_blocked_point.y(),first_blocked_point.z(),first_blocked_yaw,first_blocked_state,
            first_blocked_from.x(),first_blocked_from.y(),first_blocked_from.z(),
            first_blocked_to.x(),first_blocked_to.y(),first_blocked_to.z(),first_blocked_sample,first_blocked_samples);
    };
    while (!openSet_.empty())
    {
        if (!allowed()) {
            report_exhaustion("time_limit");
            return search_failure();
        }
        num_iter++;
        current=openSet_.pop();
        if(!current) return search_failure();
        ++expanded_nodes_;

        // if ( num_iter < 10000 )
        //     cout << "current=" << current->index.transpose() << endl;

        if (current->index(0) == endPtr->index(0) && current->index(1) == endPtr->index(1) && current->index(2) == endPtr->index(2))
        {
            // ros::Time time_2 = ros::Time::now();
            // printf("\033[34mA star iter:%d, time:%.3f\033[0m\n",num_iter, (time_2 - time_1).toSec()*1000);
            // if((time_2 - time_1).toSec() > 0.1)
            //     ROS_WARN("Time consume in A star path finding is %f", (time_2 - time_1).toSec() );
            gridPath_ = retrievePath(current);
            if (exact_reference) {
                for (auto it=gridPath_.rbegin();it!=gridPath_.rend();++it) {
                    const Vector3d point=node_coordinate((*it)->index);
                    if (connected_reference_path_.empty() || (point-connected_reference_path_.back()).norm()>1e-9)
                        connected_reference_path_.push_back(point);
                }
            }
            if(!allowed()) {gridPath_.clear();connected_reference_path_.clear();return search_failure();}
            last_search_reason_="success";return ASTAR_RET::SUCCESS;
        }
        current->state = GridNode::CLOSEDSET; //move current node from open set to closed set.

        for (int dx = -1; dx <= 1; dx++)
            for (int dy = -1; dy <= 1; dy++)
            {
                if (dx == 0 && dy == 0)
                    continue;

                Vector3i neighborIdx;
                neighborIdx(0) = (current->index)(0) + dx;
                neighborIdx(1) = (current->index)(1) + dy;
                neighborIdx(2) = plane.zIndex(neighborIdx(0),neighborIdx(1));
                if(exact_reference && neighborIdx.head<2>()==end_idx.head<2>())
                    neighborIdx.z()=end_idx.z();

                if (neighborIdx(0) < 1 || neighborIdx(0) >= POOL_SIZE_(0) - 1 || neighborIdx(1) < 1 || neighborIdx(1) >= POOL_SIZE_(1) - 1 || neighborIdx(2) < 1 || neighborIdx(2) >= POOL_SIZE_(2) - 1)
                {
                    ++rejected_pool;
                    continue;
                }

                neighborPtr=nodeAt(neighborIdx);
                neighborPtr->index = neighborIdx;

                bool flag_explored = neighborPtr->rounds == rounds_;

                if (flag_explored && neighborPtr->state == GridNode::CLOSEDSET)
                {
                    ++rejected_closed;
                    continue; //in closed set.
                }

                const Vector3d edge_from=node_coordinate(current->index),edge_to=node_coordinate(neighborIdx);
                const Vector3d edge_delta=edge_to-edge_from;
                const bool exact_anchor_edge=exact_reference &&
                    (current->index==start_idx || neighborIdx==end_idx);
                const double neighbor_yaw = frozen_body_yaw_.value_or(exact_anchor_edge?
                    std::atan2(edge_delta.y(),edge_delta.x()):
                    std::atan2(static_cast<double>(dy),static_cast<double>(dx)));
                const auto edge=checkSearchEdge(*grid_map_,edge_from,
                                                edge_to,neighbor_yaw,step_size_,allowed,planning_query_count_,frozen_body_yaw_);
                if(edge.interrupted) return search_failure();
                if (edge.occupancy!=0) {
                    if(current==startPtr) {
                        if(edge.occupancy==1)++start_occupied_edges;
                        else if(edge.occupancy==2)++start_unknown_edges;
                        else if(edge.occupancy==-1)++start_outside_edges;
                    }
                    if (edge.sample<0) ++rejected_neighbor;
                    else if (edge.sample==0) ++rejected_edge_start;
                    else if (edge.sample==edge.samples) ++rejected_edge_end;
                    else ++rejected_edge_inside;
                    record_blocked(edge_from,edge_to,edge,neighbor_yaw);
                    continue;
                }
                ++admitted_edges;
                if (current==startPtr) ++start_admitted_edges;
                neighborPtr->rounds = rounds_;

                const int dz=neighborIdx.z()-current->index.z();
                const double static_cost = exact_anchor_edge ? edge_delta.norm()*inv_step_size_:
                    std::sqrt(dx*dx+dy*dy+dz*dz);
                tentative_gScore = current->gScore + static_cost;

                if (!flag_explored)
                {
                    //discover a new node
                    neighborPtr->state = GridNode::OPENSET;
                    neighborPtr->cameFrom = current;
                    neighborPtr->gScore = tentative_gScore;
                    neighborPtr->fScore = tentative_gScore + getHeu(neighborPtr, endPtr);
                    if(!openSet_.pushOrDecrease(neighborPtr)) {
                        last_search_resource_limit_=true;last_search_reason_="open_heap_resource_limit";
                        return search_failure();
                    }
                }
                else if (tentative_gScore < neighborPtr->gScore)
                { //in open set and need update
                    neighborPtr->cameFrom = current;
                    neighborPtr->gScore = tentative_gScore;
                    neighborPtr->fScore = tentative_gScore + getHeu(neighborPtr, endPtr);
                    // Restore heap order immediately; one reserved heap slot
                    // per node, regardless of the number of score decreases.
                    if(!openSet_.pushOrDecrease(neighborPtr)) {
                        last_search_resource_limit_=true;last_search_reason_="open_heap_resource_limit";
                        return search_failure();
                    }
                }
                peak_open_nodes_=std::max(peak_open_nodes_,openSet_.size());
            }
        const auto time_2 = search_now_();
        if (!scan_planner::solveAllowed(solve_budget_) || time_2>=deadline)
        {
            report_exhaustion("time_limit");
            RCLCPP_WARN(rclcpp::get_logger("path_searching"),
                        "Failed in A-star path search: original absolute deadline exceeded");
            return search_failure();
        }
    }

    const auto time_2 = search_now_();

    const double elapsed = std::chrono::duration<double>(time_2 - time_1).count();
    if (elapsed > 0.1)
        RCLCPP_WARN(rclcpp::get_logger("path_searching"),
                    "A-star path search took %.3fs, iter=%d", elapsed, num_iter);

    report_exhaustion("open_set_exhausted");
    if (exact_reference && start_admitted_edges==0) {
        last_search_reason_="exact_reference_start_no_collision_free_edge";
        if(start_occupied_edges==0 && start_unknown_edges>0)return ASTAR_RET::INIT_UNOBSERVED;
        if(start_occupied_edges==0 && start_outside_edges>0)return ASTAR_RET::INIT_OUTSIDE_MAP;
        return ASTAR_RET::INIT_CONNECTOR_COLLISION;
    }
    if (!adjust_endpoints || remaining_shifts<2 ||
        search_now_()>=deadline) return search_failure();
    // A locally viable edge can still lead only into a small isolated pocket.
    // Retry internal helper anchors farther along the ORIGINAL line. Keep the
    // lattice origin, collision checks, total wall budget and shift budget.
    start_pt-=adjustment_direction*step_size_;
    end_pt+=adjustment_direction*step_size_;
    remaining_shifts-=2;
    }
}

vector<Vector3d> AStar::getPath()
{
    if (!connected_reference_path_.empty()) return connected_reference_path_;
    vector<Vector3d> path;

    for (auto ptr : gridPath_)
        path.push_back(Index2Coord(ptr->index));

    reverse(path.begin(), path.end());
    return path;
}
