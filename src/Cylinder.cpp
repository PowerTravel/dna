#include "Cylinder.hpp"
#include "Sphere.hpp"
#include "Plane.hpp"

Cylinder::Cylinder(double r, Eigen::Vector3d P, Eigen::Vector3d Q)
{
	_P = P;
	_Q = Q;
	_r = r;

	Eigen::Vector3d pq = _Q-_P;
	_h = pq.norm();
	_d = pq/_h;
}
Cylinder::~Cylinder()
{

}
Vec3d Cylinder::get_center()
{
	Vec3d ret = _P + (0.5*_h*_d);
	return ret;
}
std::string Cylinder::text_type()
{
	return std::string("Cylinder");
}
bool Cylinder::intersects(Sphere* s, coll_struct& cs)
{
	return s->intersects(this, cs);
}
double Cylinder::line_intersection_point(Vec3d x, Vec3d v)
{
	Eigen::Vector3d xv = x;		
	Eigen::Vector3d vv = v;

	Eigen::Vector3d alpha = vv.cross(_d);
	Eigen::Vector3d beta = (xv - _P);
	beta = beta.cross(_d);

	double aa = alpha.transpose() * alpha;
	double bb = beta.transpose()*beta;
	double ab = alpha.transpose() * beta;

	if(std::abs(aa) < 0.00000001 )
	{
		return 0;
	}

	double p = 2 * ab / aa;
	double q = (bb - _r*_r ) / aa;

	double t_1 = -p/2 - std::sqrt( std::pow((p/2),2) - q );
	double t_2 = -p/2 + std::sqrt( std::pow((p/2),2) - q );

	Eigen::Vector3d vec =  xv + t_1 * vv;

	double t_ret = 0;
	if( vv.transpose() *  (vec - xv ) >= 0)
	{
		t_ret = t_1;
	}else{
		vec =  xv + t_2 * vv;
		t_ret = t_2;
	}

	// Check if 'vec' falls below '_P' or above '_Q'
	double PQPV = (_Q-_P).dot(vec-_P);
	// On Top
	double h2 = _h*_h;
	if(PQPV > h2)
	{
		Plane p = Plane(_Q, _d);
		t_ret = p.line_intersection_point(x,v);
		vec =  xv + t_ret * vv;
		PQPV = (_Q-_P).dot(vec-_P);
		Eigen::Vector3d B = (PQPV/h2) * (_Q-_P);

		if( (B-vec).norm() > _r )
		{
			return 0;
		}
	// Below
	}else if(PQPV < 0){
		Plane p = Plane(_P, -_d);
		t_ret = p.line_intersection_point(x,v);
		vec =  xv + t_ret * vv;
		PQPV = (_Q-_P).dot(vec-_P);
		Eigen::Vector3d B = (PQPV/h2) * (_Q-_P);

		if( (B-vec).norm() > _r )
		{
			return 0;
		}
	}

	return t_ret;
}


// Not a tight fit but good enogh.
Eigen::ArrayXd Cylinder::get_span()
{
	Eigen::ArrayXd ret = Eigen::ArrayXd::Zero(6);

//	std::cout << _P.transpose() << std::endl;
//	std::cout << _Q.transpose() << std::endl;
//	std::cout << _r << std::endl;


	if( _P(0) < _Q(0) )
	{
		ret(0) = _P(0) - _r;				// x_min
		ret(1) = _Q(0) + _r;				// x_max
	}else{
		ret(0) = _Q(0) - _r;				// x_min
		ret(1) = _P(0) + _r;				// x_max
	}

	if( _P(1) < _Q(1) )
	{
		ret(2) = _P(1) - _r;				// y_min
		ret(3) = _Q(1) + _r;				// y_max
	}else{
		ret(2) = _Q(1) - _r;				// y_min
		ret(3) = _P(1) + _r;				// y_max
	}
	
	if( _P(2) < _Q(2))
	{
		ret(4) = _P(2) - _r;				// z_min
		ret(5) = _Q(2) + _r;				// z_max
	}else{
		ret(4) = _Q(2) - _r;				// z_min
		ret(5) = _P(2) + _r;				// z_max
	}
	
	return ret;
}
