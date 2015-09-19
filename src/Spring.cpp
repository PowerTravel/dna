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
	return 0;
}
