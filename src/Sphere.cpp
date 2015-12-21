#include "Sphere.hpp"
#include "Plane.hpp"
#include <iostream>

Sphere::Sphere()
{
	_r = 0.5;
	_x = Eigen::Vector3d::Zero();
}

Sphere::Sphere(Eigen::Array3d xp, double rad)
{
	_r = rad;
	_x = xp.matrix();
}

Sphere::~Sphere()
{

}


bool Sphere::intersects(Cylinder* s, coll_struct& cs)
{
	std::cerr << "intersects Cylinder -> Cylinder not implemented " << std::endl;
	return false;
}

bool Sphere::intersects(Sphere* s, coll_struct& cs)
{
	Eigen::Vector3d R =  s->_x - this-> _x;
	double separation = R.norm();
	double contact_distance = s->_r+this->_r;
	
	if( separation  < contact_distance )
	{
		cs.n = R.normalized();
		cs.p = contact_distance - separation;	
		return true;

	}else{
		return false;
	}
}

bool Sphere::intersects(Plane* p, coll_struct& cs)
{
	Eigen::Vector3d pc = p->getPoint();
	Eigen::Vector3d pn = p->getPlaneNormal();

	// Find the distance from the plane to the sphere center
	double plane_sphere_distance =std::abs( ( _x - pc ).transpose() * pn);
	if( plane_sphere_distance < _r )
	{
		cs.p = _r - plane_sphere_distance; // Penetration depth
		cs.n = pn;						  // Collision plane normal
		return true;
	}

	return false;
}

double Sphere::line_intersection_point(Eigen::ArrayXd x, Eigen::ArrayXd v)
{
	Eigen::Vector3d xv = x.matrix();
	Eigen::Vector3d vv = v.matrix();
	Eigen::Vector3d separation = _x-xv;
	double v_dot_v = vv.transpose() * vv;
	double p =   (vv.transpose() * separation);
	p =2.0 * p / v_dot_v;
	double q = (separation.transpose() * separation - _r*_r) / v_dot_v;

	double p2 = std::pow(p/2.0,2);
	if(p2 < q)
	{
		std::cerr << "Immaginary solution to sphere - line intersection" << std::endl;
		return 0;
	}
	
	double ret_1 = - p/2.0 - std::sqrt( p2 - q );
	double ret_2 = - p/2.0 + std::sqrt( p2 - q );

	// May be overkill but always works
	Eigen::Vector3d v1 =  xv + ret_1 * vv;
	Eigen::Vector3d v2 =  xv + ret_2 * vv;
	if( vv.transpose() *  (v1 - xv ) >= 0)
	{
		return ret_1;
	}else{
		return ret_2;
	}

/*
// this assumes that the penetration depth is lower than the radius of the sphere.
	if(ret_1 <= ret_2)
	{
		return ret_1;
	}else{
		return ret_2;
	}
*/	
}
// Returns min max values along the axis 
Eigen::ArrayXd Sphere::get_span()
{
	Eigen::ArrayXd ret = Eigen::ArrayXd::Zero(6);
	ret(0) = _x(0)-_r; // x_min
	ret(1) = _x(0)+_r; // x_max
	ret(2) = _x(1)-_r; // y_min
	ret(3) = _x(1)+_r; // y_max
	ret(4) = _x(2)-_r; // z_min
	ret(5) = _x(2)+_r; // z_max
	return ret;
}
double Sphere::getRadius()
{
	return _r;
}
/*
Eigen::Array3d Sphere::getCenter()
{
	return x;
}
*/
