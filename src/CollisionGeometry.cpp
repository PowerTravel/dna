#include "CollisionGeometry.hpp"

CollisionGeometry::CollisionGeometry()
{

}
CollisionGeometry::~CollisionGeometry()
{

}
/*
std::vector<Eigen::Array3d> CollisionGeometry::axis_aligned::get_corners()
{

	// X is front back,
	// Y is left right
	// Z is up down.
	std::vector< Eigen::Array3d > corners=std::vector< Eigen::Array3d >();
	Eigen::Array3d p;

	// back left bottom corner
	// - x - y - z
	p(0) = cm(0)-x;
	p(1) = cm(1)-y;
	p(2) = cm(2)-z;
	corners.push_back(p);

	// front left bottom corner
	// + x - y - z
	p(0) = cm(0)+x;
	p(1) = cm(1)-y;
	p(2) = cm(2)-z;
	corners.push_back(p);

	// back right bottom corner
	// - x + y - z
	p(0) = cm(0)-x;
	p(1) = cm(1)+y;
	p(2) = cm(2)-z;
	corners.push_back(p);
	
	// front right bottom corner
	// + x + y - z
	p(0) = cm(0)+x;
	p(1) = cm(1)+y;
	p(2) = cm(2)-z;
	corners.push_back(p);

	// back left top corner
	// - x - y + z
	p(0) = cm(0)-x;
	p(1) = cm(1)-y;
	p(2) = cm(2)+z;
	corners.push_back(p);

	// front left top corner
	// + x - y + z
	p(0) = cm(0)+x;
	p(1) = cm(1)-y;
	p(2) = cm(2)+z;
	corners.push_back(p);

	// back right top corner
	// - x + y + z
	p(0) = cm(0)-x;
	p(1) = cm(1)+y;
	p(2) = cm(2)+z;
	corners.push_back(p);

	// front right top corner
	// + x + y + z
	p(0) = cm(0)+x;
	p(1) = cm(1)+y;
	p(2) = cm(2)+z;
	corners.push_back(p);

	return corners;
}
*/
