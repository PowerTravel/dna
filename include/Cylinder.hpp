#ifndef CYLINDER_HPP
#define CYLINDER_HPP

#include "CollisionGeometry.hpp"

class Cylinder: public CollisionGeometry
{
	public:
		bool intersects(Cylinder* s, coll_struct& cs);
		bool intersects(Sphere* s, coll_struct& cs);
		bool intersects(Plane* p, coll_struct& cs);
		double line_intersection_point(Eigen::ArrayXd x, Eigen::ArrayXd v);
		Eigen::ArrayXd get_span();
};

#endif
