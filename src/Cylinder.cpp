#include "Cylinder.hpp"


bool Cylinder::intersects(Cylinder* s, coll_struct& cs)
{
	std::cerr << "intersects Cylinder -> Cylinder not implemented " << std::endl;
	return false;
}
bool Cylinder::intersects(Sphere* s, coll_struct& cs)
{
	std::cerr << "intersects Cylinder -> Sphere not implemented " << std::endl;
	return false;
}

bool Cylinder::intersects(Plane* p, coll_struct& cs)
{
	std::cerr << "intersects Cylinder -> Plane not implemented " << std::endl;
	return false;
}

double Cylinder::line_intersection_point(Eigen::ArrayXd x, Eigen::ArrayXd v)
{
	return 0;
}

Eigen::ArrayXd Cylinder::get_span()
{
	return Eigen::ArrayXd::Zero(6);
}
