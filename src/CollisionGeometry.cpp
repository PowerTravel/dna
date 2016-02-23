#include "CollisionGeometry.hpp"

int CollisionGeometry::c_id = 0;

CollisionGeometry::CollisionGeometry()
{
	id= c_id;
	c_id++;
}
CollisionGeometry::~CollisionGeometry()
{

}

int CollisionGeometry::get_id()
{
	return id;
}

bool CollisionGeometry::intersects(Cylinder* s, coll_struct& cs)
{
	std::cerr << "CollisionGeometry::intersects" << std::endl;
	std::cerr << "\tERROR: Intersection not implemented." << std::endl;
	return false;
}
bool CollisionGeometry::intersects(Sphere* s, coll_struct& cs)
{
	std::cerr << "CollisionGeometry::intersects" << std::endl;
	std::cerr << "\tERROR: Intersection not implemented." << std::endl;
	return false;
}
bool CollisionGeometry::intersects(Plane* p, coll_struct& cs)
{
	std::cerr << "CollisionGeometry::intersects" << std::endl;
	std::cerr << "\tERROR: Intersection not implemented." << std::endl;
	return false;
}
double CollisionGeometry::line_intersection_point(Vec3d x, Vec3d v)
{
	std::cerr << "CollisionGeometry::line_intersection" << std::endl;
	std::cerr << "\tERROR: Intersection not implemented." << std::endl;
	return 0;
}


std::vector<cg_ptr> CollisionGeometry::mirror(VecXd span)
{
	std::cerr << "CollisionGeometry::mirror" << std::endl;
	std::cerr << "\tERROR: mirror not implemented." << std::endl;
	return std::vector<cg_ptr>();
}
