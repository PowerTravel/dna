#ifndef SPHERE_HPP
#define SPHERE_HPP

#include <Eigen/Dense>

class Sphere
{
	public:
		Sphere();
		Sphere(Eigen::Vector3d x, Eigen::Vector3d v, double r);
		virtual ~Sphere();

		Eigen::Vector3d getPos();
		Eigen::Vector3d getVel();

		void move(Eigen::Vector3d dx);
	private:
	
		Eigen::Vector3d _x; 	// Position
		Eigen::Vector3d _v;		// Velocity
		double _r;				// Radious
	
};

#endif // SPHERE_HPP
