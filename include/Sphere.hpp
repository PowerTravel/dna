#ifndef SPHERE_HPP
#define SPHERE_HPP

#include "CollisionGeometry.hpp"

class Sphere: public CollisionGeometry
{
	public:
		Sphere();
		Sphere(Eigen::Array3d xp, double rad);
		virtual ~Sphere();

		bool intersects(Cylinder* c, coll_struct& cs);
		bool intersects(Sphere* s, coll_struct& cs);
		bool intersects(Plane* p, coll_struct& cs);
		double line_intersection_point(Vec3d x, Vec3d v);
		Eigen::ArrayXd get_span();
		Vec3d get_center();
		double get_radius();
		std::string text_type();

		int nr_collisions=0;
	private:

		double _r;				// Radius
		Eigen::Vector3d _x;		// Position
};

#endif // SPHERE_HPP
