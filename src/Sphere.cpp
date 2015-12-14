#include "Sphere.hpp"
#include "Plane.hpp"
#include <iostream>

Sphere::Sphere()
{
	r = 0.5;
	x = Eigen::Vector3d::Zero();
}

Sphere::Sphere(Eigen::Array3d xp, double rad)
{
	r = rad;
	x = xp.matrix();
}

Sphere::~Sphere()
{

}


bool Sphere::intersects(Sphere* s, coll_struct& cs)
{
	Eigen::Vector3d R =  s->x.matrix() - this-> x.matrix();
	if( R.norm() < (s->r+this->r) )
	{
		return true;
	}else{
		return false;
	}
}
bool Sphere::intersects(Plane* p, coll_struct& cs)
{
	Eigen::Vector3d pc = p->getPoint();
	Eigen::Vector3d pn = p->getPlaneNormal();

	// Find the distance from the plane to the sphere center
	double plane_sphere_distance =std::abs( ( x - pc ).transpose() * pn);
	if( plane_sphere_distance < r )
	{
		cs.p = r - plane_sphere_distance; // Penetration depth
		cs.n = pn;						  // Collision plane normal
		return true;
	}

	return false;
}

double Sphere::line_intersection_point(Eigen::ArrayXd x, Eigen::ArrayXd v)
{
	return 0;
}
// Returns min max values along the axis 
Eigen::ArrayXd Sphere::get_span()
{
	Eigen::ArrayXd ret = Eigen::ArrayXd::Zero(6);
	ret(0) = x(0)-r; // x_min
	ret(1) = x(0)+r; // x_max
	ret(2) = x(1)-r; // y_min
	ret(3) = x(1)+r; // y_max
	ret(4) = x(2)-r; // z_min
	ret(5) = x(2)+r; // z_max
	return ret;
}
double Sphere::getRadius()
{
	return r;
}
/*
Eigen::Array3d Sphere::getCenter()
{
	return x;
}
*/
