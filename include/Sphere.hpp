#ifndef SPHERE_HPP
#define SPHERE_HPP

#include "CollisionGeometry.hpp"

class Sphere: public CollisionGeometry
{
	public:
		Sphere();
		Sphere(Eigen::Array3d xp);
		virtual ~Sphere();

		bool intersects(Sphere* s);	
		Eigen::ArrayXd get_span();
//		axis_aligned get_axis_aligned();
	
		double getRadius();
		Eigen::Array3d getCenter();
	private:

		double r;				// Radious
		Eigen::Array3d x;		// Position
};

#endif // SPHERE_HPP
