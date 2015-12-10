#ifndef COLLISION_GEOMETRY_HPP
#define COLLISION_GEOMETRY_HPP

#include <Eigen/Dense>
#include <vector>

class Sphere;
class Plane;

class CollisionGeometry
{
	public:
		virtual bool intersects(Sphere* s) = 0;
		virtual bool intersects(Plane* p) = 0;
		virtual double line_intersection_point(Eigen::ArrayXd x, Eigen::ArrayXd v) = 0;
		virtual Eigen::ArrayXd get_span() = 0;

	protected:
		
};

#endif // COLLISION_GEOMETRY_HPP
