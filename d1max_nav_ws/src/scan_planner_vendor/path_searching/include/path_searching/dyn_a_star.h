#ifndef _DYN_A_STAR_H_
#define _DYN_A_STAR_H_

#include <iostream>
#include <rclcpp/rclcpp.hpp>
#include <Eigen/Eigen>
#include <plan_env/grid_map.h>
#include <queue>
#include <path_searching/search_lattice.hpp>

constexpr double inf = 1 >> 20;
struct GridNode;
typedef GridNode *GridNodePtr;

enum ASTAR_RET
{
	SUCCESS,
	INIT_ERR,
	SEARCH_ERR,
    INIT_START_OCCUPIED,
    INIT_TARGET_OCCUPIED,
    INIT_LATTICE_OCCUPIED,
    INIT_UNOBSERVED,
    INIT_OUTSIDE_MAP
};

struct GridNode
{
	enum enum_state
	{
		OPENSET = 1,
		CLOSEDSET = 2,
		UNDEFINED = 3
	};

	int rounds{0}; // Distinguish every call
	enum enum_state state
	{
		UNDEFINED
	};
	Eigen::Vector3i index;

	double gScore{inf}, fScore{inf};
	GridNodePtr cameFrom{NULL};
};

struct OpenNodeEntry
{
  GridNodePtr node;
  double f_score;
  double g_score;
};

class NodeComparator
{
public:
	bool operator()(const OpenNodeEntry &node1, const OpenNodeEntry &node2) const
	{
		return node1.f_score > node2.f_score;
	}
};

class AStar
{
private:
	GridMap::Ptr grid_map_;

	inline void coord2gridIndexFast(const double x, const double y, const double z, int &id_x, int &id_y, int &id_z);

	double getDiagHeu(GridNodePtr node1, GridNodePtr node2);
	double getManhHeu(GridNodePtr node1, GridNodePtr node2);
	double getEuclHeu(GridNodePtr node1, GridNodePtr node2);
	inline double getHeu(GridNodePtr node1, GridNodePtr node2);

	bool ConvertToIndexAndAdjustStartEndPoints(const Eigen::Vector3d start_pt, const Eigen::Vector3d end_pt, Eigen::Vector3i &start_idx, Eigen::Vector3i &end_idx);

	inline Eigen::Vector3d Index2Coord(const Eigen::Vector3i &index) const;
	inline bool Coord2Index(const Eigen::Vector3d &pt, Eigen::Vector3i &idx) const;

	//bool (*checkOccupancyPtr)( const Eigen::Vector3d &pos );

	inline int checkOccupancy(const Eigen::Vector3d &pos, const double yaw) { return grid_map_->getInflateOccupancy(pos, yaw); }

	std::vector<GridNodePtr> retrievePath(GridNodePtr current);

	double step_size_, inv_step_size_;
	Eigen::Vector3d center_;
	Eigen::Vector3i CENTER_IDX_, POOL_SIZE_;
	const double tie_breaker_ = 1.0 + 1.0 / 10000;

	std::vector<GridNodePtr> gridPath_;

	GridNodePtr ***GridNodeMap_;
	std::priority_queue<OpenNodeEntry, std::vector<OpenNodeEntry>, NodeComparator> openSet_;

	int rounds_{0};

public:
	typedef std::shared_ptr<AStar> Ptr;

	AStar(){};
	~AStar();

	void initGridMap(GridMap::Ptr occ_map, const Eigen::Vector3i pool_size);

	ASTAR_RET AstarSearch(const double step_size, Eigen::Vector3d start_pt, Eigen::Vector3d end_pt,
                         bool adjust_endpoints = true);

	std::vector<Eigen::Vector3d> getPath();
};

inline double AStar::getHeu(GridNodePtr node1, GridNodePtr node2)
{
	return tie_breaker_ * getDiagHeu(node1, node2);
}

inline Eigen::Vector3d AStar::Index2Coord(const Eigen::Vector3i &index) const
{
	return ((index - CENTER_IDX_).cast<double>() * step_size_) + center_;
};

inline bool AStar::Coord2Index(const Eigen::Vector3d &pt, Eigen::Vector3i &idx) const
{
	if (!scan_planner::nearestSearchIndex(pt, center_, step_size_, CENTER_IDX_, POOL_SIZE_, idx))
	{
		RCLCPP_ERROR(rclcpp::get_logger("path_searching"),
		             "Search endpoint outside bounded lattice: point=(%.4f %.4f %.4f)", pt.x(), pt.y(), pt.z());
		return false;
	}

	return true;
};

#endif
