#include "path_searching/dyn_a_star.h"
#include <algorithm>
#include <chrono>
#include <cmath>

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
    SearchPlane(const Vector3d &a,const Vector3d &b,const Vector3d &origin,
                const Vector3i &origin_index,double resolution,int z_index)
        : start(a),end(b),center(origin),center_index(origin_index),
          xy_delta(b.head<2>()-a.head<2>()),step(resolution),inv_step(1./resolution),
          xy_length2(xy_delta.squaredNorm()),start_z(z_index) {}
    int zIndex(int x,int y) const {
        if (xy_length2<1e-8) return start_z;
        const Vector2d xy((x-center_index.x())*step+center.x(),
                          (y-center_index.y())*step+center.y());
        const double ratio=std::clamp((xy-start.head<2>()).dot(xy_delta)/xy_length2,0.,1.);
        const double z=start.z()+ratio*(end.z()-start.z());
        return static_cast<int>(std::round((z-center.z())*inv_step))+center_index.z();
    }
};

struct SearchEdgeCheck {
    int occupancy=0,sample=-1,samples=0;
    Vector3d point=Vector3d::Zero();
};

// Preserve the existing destination-first query and complete edge sampling.
// sample=-1 identifies a rejected destination; >=0 identifies an edge sample.
SearchEdgeCheck checkSearchEdge(GridMap &map,const Vector3d &from,const Vector3d &to,
                               double yaw,double step) {
    SearchEdgeCheck result;
    result.point=to;result.occupancy=map.getInflateOccupancy(to,yaw);
    if (result.occupancy!=0) return result;
    result.samples=std::max(1,static_cast<int>(std::ceil((to-from).norm()/(step*.5))));
    for (result.sample=0;result.sample<=result.samples;++result.sample) {
        result.point=from+(to-from)*(static_cast<double>(result.sample)/result.samples);
        result.occupancy=map.getInflateOccupancy(result.point,yaw);
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
                         const std::chrono::steady_clock::time_point &deadline) {
    if (!outgoing && !insideSearchInterior(anchor,size)) return false;
    for (int dx=-1;dx<=1;++dx) for (int dy=-1;dy<=1;++dy) {
        if (std::chrono::steady_clock::now()>=deadline) return false;
        if (dx==0 && dy==0) continue;
        Vector3i neighbor(anchor.x()+dx,anchor.y()+dy,0);
        neighbor.z()=plane.zIndex(neighbor.x(),neighbor.y());
        if (!insideSearchInterior(neighbor,size)) continue;
        if (!outgoing && plane.zIndex(anchor.x(),anchor.y())!=anchor.z()) continue;
        const double yaw=std::atan2(static_cast<double>(outgoing?dy:-dy),
                                    static_cast<double>(outgoing?dx:-dx));
        const Vector3d a=searchCoordinate(anchor,plane),b=searchCoordinate(neighbor,plane);
        const auto edge=outgoing ? checkSearchEdge(map,a,b,yaw,plane.step) :
                                   checkSearchEdge(map,b,a,yaw,plane.step);
        if (edge.occupancy==0) return true;
    }
    return false;
}
} // namespace

AStar::~AStar()
{
    if (!GridNodeMap_) return;
    for (int i = 0; i < POOL_SIZE_(0); i++)
    {
        for (int j = 0; j < POOL_SIZE_(1); j++)
        {
            for (int k = 0; k < POOL_SIZE_(2); k++)
                delete GridNodeMap_[i][j][k];
            delete[] GridNodeMap_[i][j];
        }
        delete[] GridNodeMap_[i];
    }
    delete[] GridNodeMap_;
}

void AStar::initGridMap(GridMap::Ptr occ_map, const Eigen::Vector3i pool_size)
{
    if (GridNodeMap_) throw std::logic_error("AStar node pool is initialized once; use setEnvironment");
    POOL_SIZE_ = pool_size;
    CENTER_IDX_ = pool_size / 2;

    GridNodeMap_ = new GridNodePtr **[POOL_SIZE_(0)];
    for (int i = 0; i < POOL_SIZE_(0); i++)
    {
        GridNodeMap_[i] = new GridNodePtr *[POOL_SIZE_(1)];
        for (int j = 0; j < POOL_SIZE_(1); j++)
        {
            GridNodeMap_[i][j] = new GridNodePtr[POOL_SIZE_(2)];
            for (int k = 0; k < POOL_SIZE_(2); k++)
            {
                GridNodeMap_[i][j][k] = new GridNode;
            }
        }
    }

    grid_map_ = occ_map;
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
    if (!Coord2Index(start_pt, start_idx) || !Coord2Index(end_pt, end_idx))
        return false;

    const double path_yaw = std::atan2(start_to_end(1), start_to_end(0));

    // Internal rebound anchors may be free at the continuous path heading but
    // trapped for all eight lattice directions. Extend only these adjustable
    // helper anchors, not an external trajectory start or reference endpoint.
    // All anchor shifts and search retries share the caller's budgets.
    const auto shift_start=[&]() {
        if (remaining_shifts<=0) return false;
        --remaining_shifts;
        start_pt-=start_to_end*step_size_;
        return Coord2Index(start_pt,start_idx);
    };
    const auto shift_end=[&]() {
        if (remaining_shifts<=0) return false;
        --remaining_shifts;
        end_pt+=start_to_end*step_size_;
        return Coord2Index(end_pt,end_idx);
    };
    for (;;) {
        if (!scan_planner::solveAllowed(solve_budget_) || std::chrono::steady_clock::now()>=deadline) return false;
        const int start_occ=checkOccupancy(Index2Coord(start_idx),path_yaw);
        const int end_occ=checkOccupancy(Index2Coord(end_idx),path_yaw);
        if (start_occ==-1 || end_occ==-1) return false;
        if (start_occ!=0) {if (!shift_start()) return false;continue;}
        if (end_occ!=0) {if (!shift_end()) return false;continue;}

        const SearchPlane plane(Index2Coord(start_idx),Index2Coord(end_idx),
                                center_,CENTER_IDX_,step_size_,start_idx.z());
        if (!hasSearchAnchorEdge(*grid_map_,start_idx,true,plane,POOL_SIZE_,deadline)) {
            if (!shift_start()) return false;
            continue;
        }
        if (!hasSearchAnchorEdge(*grid_map_,end_idx,false,plane,POOL_SIZE_,deadline)) {
            if (!shift_end()) return false;
            continue;
        }
        return true;
    }
}

ASTAR_RET AStar::AstarSearch(const double step_size, Vector3d start_pt, Vector3d end_pt,
                           bool adjust_endpoints,bool recover_reference_lattice)
{
    const auto time_1 = std::chrono::steady_clock::now();
    const auto deadline=solve_budget_ ? std::min(solve_budget_->deadline(),time_1+std::chrono::milliseconds(200)) :
        time_1+std::chrono::milliseconds(200);
    gridPath_.clear();
    connected_reference_path_.clear();
    if (!scan_planner::solveAllowed(solve_budget_)) return ASTAR_RET::SEARCH_ERR;
    if (!std::isfinite(step_size) || step_size <= 0. ||
        !start_pt.allFinite() || !end_pt.allFinite() ||
        (adjust_endpoints && recover_reference_lattice)) return ASTAR_RET::INIT_ERR;
    step_size_ = step_size;
    inv_step_size_ = 1 / step_size;
    center_ = (start_pt + end_pt) / 2;

    const Vector3d original_delta=end_pt-start_pt;
    if (adjust_endpoints && original_delta.norm()<1e-6) return ASTAR_RET::INIT_ERR;
    const Vector3d adjustment_direction=original_delta.norm()>0. ?
        Vector3d(original_delta.normalized()):Vector3d::Zero();
    // A straight line crosses at most sum(pool dimensions) cells per endpoint.
    // The same budget covers initial adjustment and every exhausted-search retry.
    int remaining_shifts=2*POOL_SIZE_.sum()+4;
    bool searched=false;
    bool reference_recovery=false;
    std::vector<std::pair<Vector3i,Vector3i>> reference_pairs;
    for (int attempt=0;;++attempt) {
    if (!scan_planner::solveAllowed(solve_budget_) || std::chrono::steady_clock::now()>=deadline) return ASTAR_RET::SEARCH_ERR;
    Vector3i start_idx, end_idx;
    const double endpoint_yaw=std::atan2(end_pt.y()-start_pt.y(), end_pt.x()-start_pt.x());
    bool endpoints_valid=false;
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
        endpoints_valid=exact_start==0 && exact_end==0 && grid_start==0 && grid_end==0;
        // Only free EXACT reference endpoints may opt into this recovery when
        // the nearest cell OR its directed exact connector is blocked. A free
        // cell alone is insufficient for the heading-dependent double cylinder.
        // Never move/skip an occupied,
        // unknown-strict or out-of-map exact endpoint. The original endpoints,
        // lattice origin and the one 200 ms wall budget remain fixed.
        const auto connector_clear=[&](const Vector3d &exact,const Vector3i &index,bool outgoing) {
            const Vector3d point=Index2Coord(index);
            const Vector3d from=outgoing?exact:point,to=outgoing?point:exact;
            const double yaw=(to-from).head<2>().norm()>1e-9 ?
                std::atan2(to.y()-from.y(),to.x()-from.x()):endpoint_yaw;
            return checkSearchEdge(*grid_map_,from,to,yaw,step_size_).occupancy==0;
        };
        bool recover_start=false,recover_end=false;
        // An unobserved nearest cell beside an observed-free exact endpoint is
        // a snapping artifact, like an occupied one: a stopped body may never
        // observe that one extra voxel. Candidates below must themselves be
        // observed free with observed-free connectors; unknown never becomes free.
        if (recover_reference_lattice && exact_start==0 && exact_end==0 &&
            grid_start>=0 && grid_start<=2 && grid_end>=0 && grid_end<=2) {
            recover_start=grid_start!=0 || !connector_clear(start_pt,start_idx,true);
            recover_end=grid_end!=0 || !connector_clear(end_pt,end_idx,false);
        }
        if (recover_start || recover_end) {
            endpoints_valid=false;
            if (!reference_recovery) {
                reference_recovery=true;
                const SearchPlane original_plane(Index2Coord(start_idx),Index2Coord(end_idx),
                    center_,CENTER_IDX_,step_size_,start_idx.z());
                const auto candidates=[&](const Vector3d &exact,const Vector3i &nearest,
                                           bool needs_recovery,bool outgoing) {
                    std::vector<Vector3i> result;
                    const int extent=needs_recovery ? 1:0; // at most one XY lattice step
                    for (int dx=-extent;dx<=extent;++dx) for (int dy=-extent;dy<=extent;++dy) {
                        if (std::chrono::steady_clock::now()>=deadline) return result;
                        Vector3i index(nearest.x()+dx,nearest.y()+dy,0);
                        index.z()=original_plane.zIndex(index.x(),index.y());
                        if (!insideSearchInterior(index,POOL_SIZE_)) continue;
                        const Vector3d point=Index2Coord(index);
                        if (checkOccupancy(point,endpoint_yaw)!=0) continue;
                        const Vector3d from=outgoing?exact:point,to=outgoing?point:exact;
                        const double yaw=(to-from).head<2>().norm()>1e-9 ?
                            std::atan2(to.y()-from.y(),to.x()-from.x()):endpoint_yaw;
                        // A clear nearby cell does not justify crossing an
                        // obstacle, nor rotating an exact endpoint into one.
                        if (checkSearchEdge(*grid_map_,from,to,yaw,step_size_).occupancy==0)
                            result.push_back(index);
                    }
                    std::stable_sort(result.begin(),result.end(),[&](const auto &a,const auto &b) {
                        return (Index2Coord(a)-exact).squaredNorm()<(Index2Coord(b)-exact).squaredNorm();
                    });
                    return result;
                };
                const auto starts=candidates(start_pt,start_idx,recover_start,true);
                const auto ends=candidates(end_pt,end_idx,recover_end,false);
                for (const auto &a:starts) for (const auto &b:ends) {
                    if (std::chrono::steady_clock::now()>=deadline) break;
                    const SearchPlane plane(Index2Coord(a),Index2Coord(b),center_,CENTER_IDX_,step_size_,a.z());
                    if (hasSearchAnchorEdge(*grid_map_,a,true,plane,POOL_SIZE_,deadline) &&
                        hasSearchAnchorEdge(*grid_map_,b,false,plane,POOL_SIZE_,deadline))
                        reference_pairs.emplace_back(a,b);
                }
                std::stable_sort(reference_pairs.begin(),reference_pairs.end(),[&](const auto &a,const auto &b) {
                    const auto cost=[&](const auto &pair) {
                        return (Index2Coord(pair.first)-start_pt).squaredNorm()+
                               (Index2Coord(pair.second)-end_pt).squaredNorm();
                    };
                    return cost(a)<cost(b);
                });
            }
            if (static_cast<std::size_t>(attempt)<reference_pairs.size()) {
                start_idx=reference_pairs[attempt].first;end_idx=reference_pairs[attempt].second;
                endpoints_valid=true;
            }
        }
    }
    if (!endpoints_valid)
    {
        if (searched || std::chrono::steady_clock::now()>=deadline) {
            RCLCPP_WARN(rclcpp::get_logger("path_searching"),
                "A-star internal retry stopped: attempt=%d remaining_shifts=%d total_seconds=%.6f",
                attempt,remaining_shifts,
                std::chrono::duration<double>(std::chrono::steady_clock::now()-time_1).count());
            return ASTAR_RET::SEARCH_ERR;
        }
        if (!adjust_endpoints && exact_start!=3 && grid_map_->requiresObservedFree()) {
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
        const auto now=std::chrono::steady_clock::now();
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
        if (exact_start==1) return ASTAR_RET::INIT_START_OCCUPIED;
        if (exact_end==1) return ASTAR_RET::INIT_TARGET_OCCUPIED;
        if (grid_start==1 || grid_end==1) return ASTAR_RET::INIT_LATTICE_OCCUPIED;
        // With an unobserved lattice cell, no clear pair may still be waiting
        // for observation; keep that distinct from a proven connector collision.
        if (reference_recovery && reference_pairs.empty() && grid_start!=2 && grid_end!=2)
            return ASTAR_RET::INIT_CONNECTOR_COLLISION;
        if (exact_start==-1 || exact_end==-1 || grid_start==-1 || grid_end==-1)
            return ASTAR_RET::INIT_OUTSIDE_MAP;
        if (exact_start==2 || exact_end==2 || grid_start==2 || grid_end==2)
            return ASTAR_RET::INIT_UNOBSERVED;
        return ASTAR_RET::INIT_ERR;
    }

    ++rounds_;
    searched=true;
    const Eigen::Vector3d search_start = Index2Coord(start_idx);
    const Eigen::Vector3d search_end = Index2Coord(end_idx);
    const SearchPlane plane(search_start,search_end,center_,CENTER_IDX_,step_size_,start_idx.z());

    // if ( start_pt(0) > -1 && start_pt(0) < 0 )
    //     cout << "start_pt=" << start_pt.transpose() << " end_pt=" << end_pt.transpose() << endl;

    GridNodePtr startPtr = GridNodeMap_[start_idx(0)][start_idx(1)][start_idx(2)];
    GridNodePtr endPtr = GridNodeMap_[end_idx(0)][end_idx(1)][end_idx(2)];

    std::priority_queue<OpenNodeEntry, std::vector<OpenNodeEntry>, NodeComparator> empty;
    openSet_.swap(empty);

    GridNodePtr neighborPtr = NULL;
    GridNodePtr current = NULL;

    endPtr->index = end_idx;

    startPtr->index = start_idx;
    startPtr->rounds = rounds_;
    startPtr->gScore = 0;
    startPtr->fScore = getHeu(startPtr, endPtr);
    startPtr->state = GridNode::OPENSET; //put start node in open set
    startPtr->cameFrom = NULL;
    openSet_.push({startPtr, startPtr->fScore, startPtr->gScore});

    double tentative_gScore;

    int num_iter = 0;
    // Failure-only diagnostics. Counts do not alter search, endpoint adjustment,
    // collision predicates, sampling, queue ordering or its time budget.
    std::size_t rejected_pool=0,rejected_closed=0,rejected_neighbor=0;
    std::size_t rejected_edge_start=0,rejected_edge_inside=0,rejected_edge_end=0;
    std::size_t admitted_edges=0,start_admitted_edges=0;
    Eigen::Vector3d first_blocked_point=Eigen::Vector3d::Zero();
    double first_blocked_yaw=0.;int first_blocked_state=0;
    const auto record_blocked=[&](const Eigen::Vector3d &point,double yaw,int state) {
        if (first_blocked_state==0) {
            first_blocked_point=point;first_blocked_yaw=yaw;first_blocked_state=state;
        }
    };
    const auto report_exhaustion=[&](const char *reason) {
        RCLCPP_WARN(rclcpp::get_logger("path_searching"),
            "A-star %s: adjust_endpoints=%s reference_lattice_recovery=%s candidates=%zu attempt=%d iter=%d adjusted_start=(%.6f %.6f %.6f) "
            "adjusted_end=(%.6f %.6f %.6f) admitted=%zu start_admitted=%zu "
            "reject(pool=%zu closed=%zu neighbor=%zu edge_start=%zu edge_inside=%zu edge_end=%zu) "
            "first_blocked=(%.6f %.6f %.6f) yaw=%.6f state=%d",
            reason,adjust_endpoints?"true":"false",reference_recovery?"true":"false",reference_pairs.size(),attempt,num_iter,
            search_start.x(),search_start.y(),search_start.z(),
            search_end.x(),search_end.y(),search_end.z(),admitted_edges,start_admitted_edges,
            rejected_pool,rejected_closed,rejected_neighbor,rejected_edge_start,
            rejected_edge_inside,rejected_edge_end,first_blocked_point.x(),
            first_blocked_point.y(),first_blocked_point.z(),first_blocked_yaw,first_blocked_state);
    };
    while (!openSet_.empty())
    {
        if (std::chrono::steady_clock::now()>=deadline) {
            report_exhaustion("time_limit");
            return ASTAR_RET::SEARCH_ERR;
        }
        num_iter++;
        const auto entry=openSet_.top();
        current = entry.node;
        openSet_.pop();
        if (current->state==GridNode::CLOSEDSET || entry.g_score!=current->gScore) continue;

        // if ( num_iter < 10000 )
        //     cout << "current=" << current->index.transpose() << endl;

        if (current->index(0) == endPtr->index(0) && current->index(1) == endPtr->index(1) && current->index(2) == endPtr->index(2))
        {
            // ros::Time time_2 = ros::Time::now();
            // printf("\033[34mA star iter:%d, time:%.3f\033[0m\n",num_iter, (time_2 - time_1).toSec()*1000);
            // if((time_2 - time_1).toSec() > 0.1)
            //     ROS_WARN("Time consume in A star path finding is %f", (time_2 - time_1).toSec() );
            gridPath_ = retrievePath(current);
            if (reference_recovery) {
                connected_reference_path_.push_back(start_pt);
                for (auto it=gridPath_.rbegin();it!=gridPath_.rend();++it) {
                    const Vector3d point=Index2Coord((*it)->index);
                    if ((point-connected_reference_path_.back()).norm()>1e-9)
                        connected_reference_path_.push_back(point);
                }
                if ((end_pt-connected_reference_path_.back()).norm()>1e-9)
                    connected_reference_path_.push_back(end_pt);
                RCLCPP_INFO(rclcpp::get_logger("path_searching"),
                    "A-star reference lattice recovered: attempt=%d candidates=%zu exact_start=(%.6f %.6f %.6f) "
                    "lattice_start=(%.6f %.6f %.6f) exact_end=(%.6f %.6f %.6f) lattice_end=(%.6f %.6f %.6f)",
                    attempt,reference_pairs.size(),start_pt.x(),start_pt.y(),start_pt.z(),
                    search_start.x(),search_start.y(),search_start.z(),end_pt.x(),end_pt.y(),end_pt.z(),
                    search_end.x(),search_end.y(),search_end.z());
            }
            return ASTAR_RET::SUCCESS;
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

                if (neighborIdx(0) < 1 || neighborIdx(0) >= POOL_SIZE_(0) - 1 || neighborIdx(1) < 1 || neighborIdx(1) >= POOL_SIZE_(1) - 1 || neighborIdx(2) < 1 || neighborIdx(2) >= POOL_SIZE_(2) - 1)
                {
                    ++rejected_pool;
                    continue;
                }

                neighborPtr = GridNodeMap_[neighborIdx(0)][neighborIdx(1)][neighborIdx(2)];
                neighborPtr->index = neighborIdx;

                bool flag_explored = neighborPtr->rounds == rounds_;

                if (flag_explored && neighborPtr->state == GridNode::CLOSEDSET)
                {
                    ++rejected_closed;
                    continue; //in closed set.
                }

                const double neighbor_yaw = std::atan2(static_cast<double>(dy), static_cast<double>(dx));
                const auto edge=checkSearchEdge(*grid_map_,Index2Coord(current->index),
                                                Index2Coord(neighborIdx),neighbor_yaw,step_size_);
                if (edge.occupancy!=0) {
                    if (edge.sample<0) ++rejected_neighbor;
                    else if (edge.sample==0) ++rejected_edge_start;
                    else if (edge.sample==edge.samples) ++rejected_edge_end;
                    else ++rejected_edge_inside;
                    record_blocked(edge.point,neighbor_yaw,edge.occupancy);
                    continue;
                }
                ++admitted_edges;
                if (current==startPtr) ++start_admitted_edges;
                neighborPtr->rounds = rounds_;

                const int dz = neighborIdx(2) - current->index(2);
                double static_cost = sqrt(dx * dx + dy * dy + dz * dz);
                tentative_gScore = current->gScore + static_cost;

                if (!flag_explored)
                {
                    //discover a new node
                    neighborPtr->state = GridNode::OPENSET;
                    neighborPtr->cameFrom = current;
                    neighborPtr->gScore = tentative_gScore;
                    neighborPtr->fScore = tentative_gScore + getHeu(neighborPtr, endPtr);
                    openSet_.push({neighborPtr, neighborPtr->fScore, neighborPtr->gScore});
                }
                else if (tentative_gScore < neighborPtr->gScore)
                { //in open set and need update
                    neighborPtr->cameFrom = current;
                    neighborPtr->gScore = tentative_gScore;
                    neighborPtr->fScore = tentative_gScore + getHeu(neighborPtr, endPtr);
                    // Immutable queue entries implement decrease-key by
                    // reinsertion; mutating a node does not restore heap order.
                    openSet_.push({neighborPtr, neighborPtr->fScore, neighborPtr->gScore});
                }
            }
        const auto time_2 = std::chrono::steady_clock::now();
        if (!scan_planner::solveAllowed(solve_budget_) || time_2>=deadline)
        {
            report_exhaustion("time_limit");
            RCLCPP_WARN(rclcpp::get_logger("path_searching"),
                        "Failed in A-star path search: 0.2 second time limit exceeded");
            return ASTAR_RET::SEARCH_ERR;
        }
    }

    const auto time_2 = std::chrono::steady_clock::now();

    const double elapsed = std::chrono::duration<double>(time_2 - time_1).count();
    if (elapsed > 0.1)
        RCLCPP_WARN(rclcpp::get_logger("path_searching"),
                    "A-star path search took %.3fs, iter=%d", elapsed, num_iter);

    report_exhaustion("open_set_exhausted");
    if (reference_recovery) {
        if (static_cast<std::size_t>(attempt+1)<reference_pairs.size() &&
            std::chrono::steady_clock::now()<deadline) continue;
        return ASTAR_RET::SEARCH_ERR;
    }
    if (!adjust_endpoints || remaining_shifts<2 ||
        std::chrono::steady_clock::now()>=deadline) return ASTAR_RET::SEARCH_ERR;
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
