#ifndef SPHERE_HPP
#define SPHERE_HPP

#include "Dynamics.hpp"

class Sphere : public Dynamics
{
	public:
	
		Sphere();
		Sphere(Eigen::Vector3d x, Eigen::Vector3d v, double r);
		virtual ~Sphere();

		void update();
	
		double getRadius();
	private:
		
		double r;				// Radious
};

#endif // SPHERE_HPP
