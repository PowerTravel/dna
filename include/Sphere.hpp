#ifndef SPHERE_HPP
#define SPHERE_HPP

#include "CollisionGeometry.hpp"

class Sphere: public CollisionGeometry
{
	public:
		Sphere();
		Sphere(Eigen::Array3d xp, double rad);
		virtual ~Sphere();

		bool intersects(Cylinder* s, coll_struct& cs);
		bool intersects(Sphere* s, coll_struct& cs);
		bool intersects(Plane* p, coll_struct& cs);
		double line_intersection_point(Eigen::ArrayXd x, Eigen::ArrayXd v);
		Eigen::ArrayXd get_span();
	
		double getRadius();
//		Eigen::Array3d getCenter();
	private:

		double _r;				// Radious
		Eigen::Vector3d _x;		// Position
};

#endif // SPHERE_HPP
