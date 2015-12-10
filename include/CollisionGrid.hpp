#ifndef COLLISION_GRID_HPP
#define COLLISION_GRID_HPP

#ifndef DEFAULT_MAP_SIZE
#define DEFAULT_MAP_SIZE 10000
#endif

#ifndef MAX_MAP_SIZE
#define MAX_MAP_SIZE UINT_MAX
#endif

#ifndef INDEX_TYPE
typedef unsigned int idx_type;
#endif 

#include "Eigen/Dense" 
#include "Chain.hpp"
#include <map>

class CollisionGrid{
	public:
		CollisionGrid(double bs);
		virtual ~CollisionGrid();

		void set_up(Chain* c);
		// Takes the collision geometry and preforms broad phase collisoin detection
		// Returns a vector of all possible intersections.
		std::vector< std::shared_ptr<CollisionGeometry> > get_collision_bodies(CollisionGeometry& g);

	private:
		Chain* _c;
		double box_size;	
		int max_idx;
		std::map<idx_type, std::vector<int> > grid;

		int grid_map(int link, Eigen::Vector3d v);

		int get_max_axis(Chain* c);
		void push_key_to_map(idx_type key, int val);

		//std::vector<idx_type> get_intersection_keys(Chain::link l);
		std::vector<idx_type> get_intersection_keys(CollisionGeometry& g);
};

#endif
