#ifndef COLLISION_GRID_HPP
#define COLLISION_GRID_HPP

#include "EigenLibs.hpp" 
#include "CollisionGeometry.hpp"

class CollisionGrid
{

	public:
		void set_up(std::vector<cg_ptr> v);
		std::vector< cg_ptr > get_collision_bodies(std::shared_ptr<CollisionGeometry> g);

	private:
		
		std::vector<cg_ptr> _collision_bodies;
		std::map<unsigned int, std::vector<int> > _grid;
		double _grid_box_size;
		Vec3d _T // Translation vector which can translate the geometries to the
				// positive qadrant.
		Vec3d _S // The cumulative size of all the translated bodies in our grid.
		unsigned int _max_axis_length;


		std::vector<unsigned int> get_keys(VecXd span)
		{
		
		}
	
}

#endif // COLLISION_GRID_HPP
