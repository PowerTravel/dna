#ifndef COLLISION_GEOMETRY_HPP
#define COLLISION_GEOMETRY_HPP

#include <Eigen/Dense>

class CollisionGeometry
{

	public:
		CollisionGeometry();
		virtual ~CollisionGeometry();

		virtual bool intersects(Eigen::Array3d p) = 0;
	protected:
		
};

#endif // COLLISION_GEOMETRY_HPP
