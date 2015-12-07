#include "Sphere.hpp"

Sphere::Sphere()
{
	r = 0.5;
	x = Eigen::Vector3d::Zero();
}

Sphere::Sphere(Eigen::Array3d xp)
{
	r = 0.5;
	x = xp;
}

Sphere::~Sphere()
{

}


bool Sphere::intersects(Sphere* s)
{
	Eigen::Vector3d R =  s->x.matrix() - this-> x.matrix();
	if( R.norm() < (s->r+this->r) )
	{
		return true;
	}else{
		return false;
	}
}

// Returns an axis aligned box
/*
Sphere::axis_aligned Sphere::get_axis_aligned()
{
	axis_aligned ret;
	ret.cm = x;
	ret.x = r;
	ret.y = r;
	ret.z = r;
	return ret;
}
*/
// Returns min max values along the axis 
Eigen::ArrayXd Sphere::get_span()
{
	Eigen::ArrayXd ret = Eigen::ArrayXd::Zero(6);
	ret(0) = x(0)-r; // x_min
	ret(1) = x(0)+r; // x_max
	ret(2) = x(1)-r; // y_min
	ret(3) = x(1)+r; // y_max
	ret(4) = x(2)-r; // z_min
	ret(5) = x(2)+r; // z_max
	return ret;
}

double Sphere::getRadius()
{
	return r;
}
Eigen::Array3d Sphere::getCenter()
{
	return x;
}
