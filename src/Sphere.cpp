#include "Sphere.hpp"

Sphere::Sphere()
{
	r = 0.5;
	x = Eigen::Vector3d::Zero();
}

Sphere::Sphere(Eigen::Array3d xp)
{
	r = 0.5;
	x = xp;
}

Sphere::~Sphere()
{

}


bool Sphere::intersects(Eigen::Array3d p)
{
	return false;
}

double Sphere::getRadius()
{
	return r;
}
