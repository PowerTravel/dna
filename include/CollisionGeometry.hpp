#ifndef COLLISION_GEOMETRY_HPP
#define COLLISION_GEOMETRY_HPP

#include <Eigen/Dense>
#include <vector>
#ifndef COLLISION_GEOMETRY_TYPEDEF
#define COLLISION_GEOMETRY_TYPEDEF
class CollisionGeometry;
typedef CollisionGeometry collGeom;
#endif  // COLLGEOM_TYPEDEF
class Sphere;

class CollisionGeometry
{
	public:
		CollisionGeometry();
		virtual ~CollisionGeometry();

		virtual bool intersects(Sphere* s) = 0;
		virtual Eigen::ArrayXd get_span() = 0;
	protected:
		
};

#endif // COLLISION_GEOMETRY_HPP
