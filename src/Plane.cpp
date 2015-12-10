#include "Plane.hpp"
#include "Sphere.hpp"
Plane::Plane(Eigen::Array3d xp, Eigen::Array3d np)
{
	x = xp;
	n = np;
}

Plane::~Plane()
{

}

bool Plane::intersects(Sphere* s)
{
	return s->intersects(this);	
}

bool Plane::intersects(Plane* p)
{
	// Any plane that is not parallell intersects at some point
	Eigen::Vector3d pn1 = p->getPlaneNormal().matrix();
	Eigen::Vector3d pn2 = n.matrix();

	double tol = 0.0000001;

	if( (std::abs(pn1.transpose()*pn2) - 1.0 )< tol  )
	{
		return true;
	}

	return false;
}

Eigen::ArrayXd Plane::get_span()
{
	return Eigen::ArrayXd::Zero(6);
}

Eigen::ArrayXd Plane::getPlaneNormal()
{
	return n;
}
Eigen::ArrayXd Plane::getPoint()
{
	return x;
}
