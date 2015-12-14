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

bool Plane::intersects(Plane* p, coll_struct& cs)
{
	// Any plane that is not parallell intersects at some point
	Eigen::Vector3d pn1 = p->getPlaneNormal();
	Eigen::Vector3d pn2 = n;

	double tol = 0.0000001;

	if( (std::abs(pn1.transpose()*pn2) - 1.0 )< tol  )
	{
		return true;
	}

	return false;
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

Eigen::Vector3d Plane::getPlaneNormal()
{
	return n;
}
Eigen::Vector3d Plane::getPoint()
{
	return x;
}
