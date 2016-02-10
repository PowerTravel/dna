#ifndef COLLISION_GRID_HPP
#define COLLISION_GRID_HPP

#ifndef DEFAULT_MAP_SIZE
#define DEFAULT_MAP_SIZE 810
#endif

#ifndef MAX_MAP_SIZE
#define MAX_MAP_SIZE UINT_MAX
#endif

#ifndef INDEX_TYPE
typedef unsigned int idx_type;
#endif 

#include "EigenLibs.hpp" 
#include "Chain.hpp"
#include <map>

class CollisionGrid{


	struct geom_struct{
		// pointer to a collision geometry
		std::shared_ptr<CollisionGeometry> cg;
		// Key to <<std::endl;all the boxes where it appears
		std::vector<idx_type> key;
		// the lower left corners of the boxes
		std::vector<Eigen::Vector3d> idx;
		// the length of the box sides
		double s;
		// The max idx
		double m_idx;
	};
	public:
	
		CollisionGrid();
		CollisionGrid(double bs);
		virtual ~CollisionGrid();

		void set_up(std::vector<cg_ptr> v, idx_type mid = DEFAULT_MAP_SIZE);
		// Takes the collision geometry and preforms broad phase collisoin detection
		// Returns a vector of all possible intersections.
		std::vector< std::shared_ptr<CollisionGeometry> > get_collision_bodies(std::shared_ptr<CollisionGeometry> g);


		// debug functions
		void print_box_corners(std::string path);
		std::vector<geom_struct> geoms;
		void print_intersecting_box_corners(cg_ptr g);
		geom_struct active_geom;
		bool active;

		std::string print_active_geom();
		
	private:
		double box_size;	
		int max_idx;
		std::map<idx_type, std::vector<int> > grid;
		bool ok;

		std::vector<cg_ptr> geom_vec;

		void set_max_axis(idx_type i);
		int get_max(Eigen::Array3d v);
		void push_key_to_map(idx_type key, int val);

		std::vector<idx_type> get_intersection_keys(std::shared_ptr<CollisionGeometry> g);

		idx_type map_key(int i, int j, int k);

};

#endif
