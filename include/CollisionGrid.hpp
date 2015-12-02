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
		CollisionGrid();
		virtual ~CollisionGrid();

		void set_chain(Chain* c);

	private:

		double box_size;	
		std::map<std::vector<int>, idx_type > grid;

		Eigen::ArrayXXd r;
		
		int grid_map(int link, Eigen::Vector3d v);
};

#endif
