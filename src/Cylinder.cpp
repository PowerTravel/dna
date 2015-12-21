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
	Eigen::Vector3d xv = x.matrix();		
	Eigen::Vector3d vv = v.matrix();

	Eigen::Vector3d alpha = vv.cross(_d);
	Eigen::Vector3d beta = (xv - _x);
	beta = beta.cross(_d);

	double aa = alpha.transpose() * alpha;
	double bb = beta.transpose()*beta;
	double ab = alpha.transpose() * beta;

	if(aa == 0)
	{
		return 0;
	}

	double p = 2 * ab / aa;
	double q = (bb - _r*_r ) / aa;

	double t_1 = -p/2 - std::sqrt( std::pow((p/2),2) - q );
	double t_2 = -p/2 + std::sqrt( std::pow((p/2),2) - q );

	Eigen::Vector3d v1 =  xv + t_1 * vv;
	Eigen::Vector3d v2 =  xv + t_2 * vv;
	// Check if V1 and V2 hits the lower edge of the cylinder
	// If it hits the lower edge we can create a sphere with the same r
	// located at the edge and and return intersection with that instead
	// Note: lower edge means the base of the cylinder.
	// This wont't be a pure cylinder but a cylinder with a sphere
	// attached at the base. Each cylinder will be attached to another
	// cylinder such that there are no holes.


	if( vv.transpose() *  (v1 - xv ) >= 0)
	{
		return t_1;
	}else{
		return t_2;
	}
}
// Not as tight a fit as it can be but good enough I think
Eigen::ArrayXd Cylinder::get_span()
{
	Eigen::Vector3d ex = Eigen::Vector3d(1,0,0);
	Eigen::Vector3d ey = Eigen::Vector3d(0,1,0);
	Eigen::Vector3d ez = Eigen::Vector3d(0,0,1);
	Eigen::ArrayXd ret = Eigen::ArrayXd::Zero(6);

	if(_d.dot(ex) >= 0)
	{
		ret(0) = _x(0) -  _r;				// x_min
		ret(1) = _x(0) + _h * _d(0) + _r;	// x_max
	}else{
		ret(0) = _x(0) + _h * _d(0) - _r;	// x_min
		ret(1) = _x(0) + _r;				// x_max
	}

	if(_d.dot(ey) >= 0)
	{
		ret(2) = _x(1) -  _r;				// y_min
		ret(3) = _x(1) + _h * _d(1) + _r;	// y_max
	}else{
		ret(2) = _x(1) + _h * _d(1) - _r;	// y_min
		ret(3) = _x(1) + _r;				// y_max
	}
	
	if(_d.dot(ez) >= 0)
	{
		ret(4) = _x(2) -  _r;				// z_min
		ret(5) = _x(2) + _h * _d(2) + _r;	// z_max
	}else{
		ret(4) = _x(2) + _h * _d(2) - _r;	// z_min
		ret(5) = _x(2) + _r;				// z_max
	}
	return ret;
}
