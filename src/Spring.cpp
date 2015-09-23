#include "Spring.hpp"

double Spring::_a  = 0;
double Spring::_k  = 0;

Spring::Spring(double a, double k)
{
	_a  = a;
	_k  = k;
}


Spring::~Spring()
{

}

double Spring::getForce( double dx )
{
	return -_k*( dx -_a);
}

// The force of p1, and p3 on p2     p1 ---- p2 -- p3
Eigen::Vector3d Spring::getForce( Eigen::Vector3d& p1, Spring& s1,Eigen::Vector3d& p2, Spring& s2, Eigen::Vector3d& p3)
{
	Eigen::Vector3d dir1 = p2 - p1;
	double len1 = dir1.norm();
	dir1 = dir1/len1;

	Eigen::Vector3d dir2 = p2 - p1;
	double len2 = dir2.norm();
	dir1 = dir2/len2;

	return s1.getForce(len1)*dir1 - s2.getForce(len2)*dir2 ;
}

