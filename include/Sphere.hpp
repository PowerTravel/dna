#ifndef SPHERE_HPP
#define SPHERE_HPP

#include "CollisionGeometry.hpp"

class Sphere: public CollisionGeometry
{
	public:
		Sphere();
		Sphere(Eigen::Array3d xp);
		virtual ~Sphere();

		bool intersects(Eigen::Array3d p);	
	
		double getRadius();
	private:
		
		double r;				// Radious
		Eigen::Array3d x;		// Position
};

#endif // SPHERE_HPP
