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
	private:
		double _r;
		double _h;
		Eigen::Vector3d _x;
		Eigen::Vector3d _d;
};

#endif
