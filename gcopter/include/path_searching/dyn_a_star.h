#ifndef _DYN_A_STAR_H_
#define _DYN_A_STAR_H_

#include <iostream>
#include <ros/ros.h>
#include <ros/console.h>
#include <Eigen/Eigen>
#include <queue>

#include "gcopter/voxel_map.hpp"

constexpr double inf = 1 >> 20;
struct GridNode;
typedef GridNode *GridNodePtr;

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

class NodeComparator
{
public:
	bool operator()(GridNodePtr node1, GridNodePtr node2)
	{
		return node1->fScore > node2->fScore;
	}
};

class AStar
{
private:
	voxel_map::VoxelMap::Ptr grid_map_;

	inline void coord2gridIndexFast(const double x, const double y, const double z, int &id_x, int &id_y, int &id_z);

	double getDiagHeu(GridNodePtr node1, GridNodePtr node2);
	double getManhHeu(GridNodePtr node1, GridNodePtr node2);
	double getEuclHeu(GridNodePtr node1, GridNodePtr node2);
	inline double getHeu(GridNodePtr node1, GridNodePtr node2);

	bool ConvertToIndexAndAdjustStartEndPoints(const Eigen::Vector3d start_pt, const Eigen::Vector3d end_pt, Eigen::Vector3i &start_idx, Eigen::Vector3i &end_idx);

	inline Eigen::Vector3d Index2Coord(const Eigen::Vector3i &index) const;
	inline bool Coord2Index(const Eigen::Vector3d &pt, Eigen::Vector3i &idx) const;

	//bool (*checkOccupancyPtr)( const Eigen::Vector3d &pos );

	inline bool checkOccupancy(const Eigen::Vector3d &pos) { return (bool)grid_map_->query(pos); }

	std::vector<GridNodePtr> retrievePath(GridNodePtr current);

	double step_size_, inv_step_size_;
	Eigen::Vector3d center_;
	Eigen::Vector3i CENTER_IDX_, POOL_SIZE_;
	const double tie_breaker_ = 1.0 + 1.0 / 10000;

	std::vector<GridNodePtr> gridPath_;

	GridNodePtr ***GridNodeMap_;
	std::priority_queue<GridNodePtr, std::vector<GridNodePtr>, NodeComparator> openSet_;

	int rounds_{0};

	bool use_search_bound_ = false;
	Eigen::MatrixX4d PolyBound_;

public:
	typedef std::shared_ptr<AStar> Ptr;

	AStar(){};
	~AStar();

	void initGridMap(voxel_map::VoxelMap::Ptr occ_map, const Eigen::Vector3i pool_size);

	bool AstarSearch(const double step_size, Eigen::Vector3d start_pt, Eigen::Vector3d end_pt, const double timeout = 0.2);

	std::vector<Eigen::Vector3d> getPath();

	inline void setSearchBound(const Eigen::MatrixX4d &Poly);
	inline void disableSearchBound() { use_search_bound_ = false; };
	inline bool checkSearchBound(const Eigen::Vector3d &pos);
	inline int getOutboundIdx(const Eigen::Vector3d &pos);
	inline Eigen::MatrixX4d getSearchBound() { return PolyBound_; };
	inline Eigen::Vector4d getSearchBound(const int idx) { return PolyBound_.row(idx); };
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
	idx = ((pt - center_) * inv_step_size_ + Eigen::Vector3d(0.5, 0.5, 0.5)).cast<int>() + CENTER_IDX_;

	if (idx(0) < 0 || idx(0) >= POOL_SIZE_(0) || idx(1) < 0 || idx(1) >= POOL_SIZE_(1) || idx(2) < 0 || idx(2) >= POOL_SIZE_(2))
	{
		ROS_ERROR("Ran out of pool, index=%d %d %d", idx(0), idx(1), idx(2));
		return false;
	}

	return true;
};

inline void AStar::setSearchBound(const Eigen::MatrixX4d &Poly)
{
	use_search_bound_ = true;
	PolyBound_ = Poly;
}

inline bool AStar::checkSearchBound(const Eigen::Vector3d &pos)
{
	if (!use_search_bound_) return true;

	for (int i = 0; i < PolyBound_.rows(); i++)
	{
		double testVal = PolyBound_.leftCols<3>().row(i).dot(pos) + PolyBound_.row(i)(3);
		if (testVal > 0) return false;
	}

	return true;
}

inline int AStar::getOutboundIdx(const Eigen::Vector3d &pos)
{
	if (!use_search_bound_) return -2;

	for (int i = 0; i < PolyBound_.rows(); i++)
	{
		double testVal = PolyBound_.leftCols<3>().row(i).dot(pos) + PolyBound_.row(i)(3);
		if (testVal > 0) return i;
	}

	return -1;
}

#endif
