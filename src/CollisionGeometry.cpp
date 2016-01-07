#include "CollisionGeometry.hpp"

bool CollisionGeometry::intersects(Cylinder* s, coll_struct& cs)
{
	std::cerr << "ERROR: Intersection not implemented." << std::endl;
	return false;
}
bool CollisionGeometry::intersects(Sphere* s, coll_struct& cs)
{
	std::cerr << "ERROR: Intersection not implemented." << std::endl;
	return false;
}
bool CollisionGeometry::intersects(Plane* p, coll_struct& cs)
{
	std::cerr << "ERROR: Intersection not implemented." << std::endl;
	return false;
}
double CollisionGeometry::line_intersection_point(Eigen::ArrayXd x, Eigen::ArrayXd v)
{
	std::cerr << "ERROR: Intersection not implemented." << std::endl;
	return false;
}
