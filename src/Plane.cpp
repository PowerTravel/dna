#include "Plane.hpp"
#include "Sphere.hpp"
#include <iostream>
Plane::Plane(Eigen::Vector3d xp, Eigen::Vector3d np)
{
	x = xp;
	n = np.normalized();
}

Plane::~Plane()
{

}

bool Plane::intersects(Sphere* s, coll_struct& cs)
{
	return s->intersects(this, cs);	
}

double Plane::line_intersection_point(Eigen::ArrayXd r, Eigen::ArrayXd v)
{
	Eigen::Vector3d nv = n;
	Eigen::Vector3d xv = x;
	Eigen::Vector3d rv = r.matrix();
	Eigen::Vector3d vv = v.matrix();

	double num =(xv - rv).transpose() * nv; 
	double denom = vv.transpose() * nv;

	return num / denom;
}

Eigen::ArrayXd Plane::get_span()
{
	return Eigen::ArrayXd::Zero(6);
}
