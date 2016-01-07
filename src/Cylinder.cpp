#include "Cylinder.hpp"
#include "Sphere.hpp"

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

bool Cylinder::intersects(Sphere* s, coll_struct& cs)
{
	return s->intersects(this, cs);
}
double Cylinder::line_intersection_point(Eigen::ArrayXd x, Eigen::ArrayXd v)
{
	Eigen::Vector3d xv = x.matrix();		
	Eigen::Vector3d vv = v.matrix();

	Eigen::Vector3d alpha = vv.cross(_d);
	Eigen::Vector3d beta = (xv - _P);
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


	// RIGHT NOW THE CYLINDER IS TREATED AS BEING INFINITELY LONG


	// Check if V1 and V2 hits the lower edge of the cylinder
	// If it hits the lower edge we can create a sphere with the same r
	// located at the edge and and return intersection with that instead
	// Note: lower edge means the base of the cylinder.
	// This wont't be a pure cylinder but a cylinder with a sphere
	// attached at the base. Each cylinder will be attached to another
	// cylinder such that there are no holes.


	//double h2 = _h*_h;
	if( vv.transpose() *  (v1 - xv ) >= 0)
	{
/*
		// We check if the 'intersection point' lies above the cylinder
		Eigen::Vector3d PQ = Q-P;
		Eigen::Vector3d PV = v1-P;
		double PQPV = PQ.dot(PV);
		if(PQPV > h2)
		{
			Plane p = Plane(Q, _d);
			return p.line_intersection_point(x,v);
		}
*/
		return t_1;
	}else{
		return t_2;
	}
}

Eigen::ArrayXd Cylinder::get_span()
{
	Eigen::Vector3d ex = Eigen::Vector3d(1,0,0);
	Eigen::Vector3d ey = Eigen::Vector3d(0,1,0);
	Eigen::Vector3d ez = Eigen::Vector3d(0,0,1);

	Eigen::Vector3d rd = _r*_d;
	double r_x = ex.dot(rd);
	double r_y = ey.dot(rd);
	double r_z = ez.dot(rd);


	Eigen::Vector3d cz = Eigen::Vector3d(1,0,0);

	


	Eigen::ArrayXd ret = Eigen::ArrayXd::Zero(6);

	
	if( _P(0) < _Q(0) )
	{
		ret(0) = _P(0) -  r_x;				// x_min
		ret(1) = _Q(0) +  r_x;				// x_max
	}else{
		ret(0) = _Q(0) - r_x;				// x_min
		ret(1) = _P(0) + r_x;				// x_max
	}

	if( _P(1) < _Q(1) )
	{
		ret(2) = _P(1) - r_y;				// y_min
		ret(3) = _Q(1) + r_y;				// y_max
	}else{
		ret(2) = _Q(1) - r_y;				// y_min
		ret(3) = _P(1) + r_y;				// y_max
	}
	
	if( _P(2) < _Q(2))
	{
		ret(4) = _P(2) - r_z;				// z_min
		ret(5) = _Q(2) + r_z;				// z_max
	}else{
		ret(4) = _Q(2) - r_z;				// z_min
		ret(5) = _P(2) + r_z;				// z_max
	}
	
	return ret;
}
