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

std::string Plane::text_type()
{
	return std::string("Plane");
}

bool Plane::intersects(Sphere* s, coll_struct& cs)
{
	return s->intersects(this, cs);	
}

double Plane::line_intersection_point(Vec3d r, Vec3d v)
{
	Eigen::Vector3d nv = n;
	Eigen::Vector3d xv = x;
	Eigen::Vector3d rv = r;
	Eigen::Vector3d vv = v;

	double num =(xv - rv).transpose() * nv; 
	double denom = vv.transpose() * nv;

	return num / denom;
}

Vec3d Plane::get_center()
{
	return x;
}

Eigen::ArrayXd Plane::get_span()
{
	return Eigen::ArrayXd::Zero(6);
}
