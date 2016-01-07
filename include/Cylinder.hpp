#ifndef CYLINDER_HPP
#define CYLINDER_HPP

#include "CollisionGeometry.hpp"

class Cylinder: public CollisionGeometry
{

	friend class Sphere;

	public:
		Cylinder(double r, Eigen::Vector3d P, Eigen::Vector3d Q);
		virtual ~Cylinder();

		bool intersects(Sphere* s, coll_struct& cs);

		double line_intersection_point(Eigen::ArrayXd x, Eigen::ArrayXd v);
		Eigen::ArrayXd get_span();
	
	private:
		double _r;			// Radius
		double _h;			// Height
		Eigen::Vector3d _d;	// 'direction'
		Eigen::Vector3d _P;	// Bottom
		Eigen::Vector3d _Q; // Top
};

#endif
