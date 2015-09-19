#include "Spring.hpp"

Spring::Spring(double a, double k, double xi)
{
	_a  = a;
	_k  = k;
	_xi = xi;
}

Spring::~Spring()
{

}
	
double Spring::getForce( double dx, double dv )
{
	return _k*(dx-_a) - _xi*dv;
}

double Spring::getSpeed( double dx )
{
	return (_k/_xi)*(dx-_a);
}
