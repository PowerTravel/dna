#include "Sphere.hpp"

Sphere::Sphere()
{
	_x = Eigen::Vector3d(0,0,0); 
	_v = Eigen::Vector3d(0,0,0);;
	_r = 1.0;
}

Sphere::Sphere(Eigen::Vector3d x, Eigen::Vector3d v, double r)
{
	_x = x; 
	_v = v;
	_r = r;
}
Sphere::~Sphere()
{

}

Eigen::Vector3d Sphere::getPos()
{
	return _x;
}
Eigen::Vector3d Sphere::getVel()
{
	return _v;
}
