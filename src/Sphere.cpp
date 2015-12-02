#include "Sphere.hpp"

Sphere::Sphere()
{
	r = 1.0;
}

Sphere::Sphere(Eigen::Vector3d xp, Eigen::Vector3d vp, double rp):
Dynamics(xp,vp)
{
	r = rp;
}

Sphere::~Sphere()
{

}

void Sphere::update()
{

}

double Sphere::getRadius()
{
	return r;
}
