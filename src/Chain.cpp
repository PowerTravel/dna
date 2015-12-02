#include "Chain.hpp"
#include <iostream>
#include <cmath>
#include <iomanip>
#include <ctime>
#include <limits>

std::default_random_engine Chain::_generator = std::default_random_engine(time(NULL));

Chain::Chain()
{	
	_ok = false;
}

Chain::~Chain()
{

}

Eigen::ArrayXXd Chain::as_array(int start, int size)
{
	return _chain.block(0,start,3,size);
}
Eigen::ArrayXXd Chain::as_array()
{
	return _chain;
}

Eigen::Array3d Chain::span()
{
	Eigen::ArrayXd sp = Eigen::ArrayXd::Zero(6);
	for(int i = 0; i<len(); i++)
	{
		Eigen::Array3d x = _chain.block(0,i,3,1);

		if( x(0) > sp(0) ){
			sp(0) = x(0);
		}else if(  x(0) < sp(1) ){
			sp(1) = x(0);
		}
		
		if( x(1) > sp(2) ){
			sp(2) = x(1);
		}else if(  x(1) < sp(3) ){
			sp(3) = x(1);
		}

		if( x(2) > sp(4) ){
			sp(4) = x(2);
		}else if(  x(2) < sp(5) ){
			sp(5) = x(2);
		}
	}
	Eigen::Array3d ret(sp(0)-sp(1), sp(2)-sp(3), sp(4)-sp(5)  );
	return ret;

}

std::ostream& operator<<(std::ostream& os, const Chain& c)
{
	os << c._chain.transpose() << std::endl;
	return os;
}

double Chain::Rg()
{
	return Rg(0,len());
}

double Chain::Rg(int start, int size)
{
	Eigen::Array3d mean = cm(start, size);
	double var = 0;
	for(int i = start; i<start+size; i++)
	{
		Eigen::Vector3d v = (_chain.block(0,i,3,1) - mean).matrix();
		var += v.transpose() * v;
	}
	return sqrt(var/(double(size)));
}

Eigen::Array3d Chain::cm(int start, int size)
{
	return _chain.block(0,start,3,size).rowwise().mean();
}

int Chain::len()
{
	return _chain.outerSize();
}

Eigen::ArrayXd Chain::weights()
{
	return _w;
}

Eigen::Array3d Chain::int_to_coord(int i)
{
	Eigen::Vector3d idx(0,0,0);
	switch(i)
	{
		// POSITIVE X (RIGHT)
		case 0:
			idx << 1,0,0;
			break;
		// NEGATIVE X (LEFT)
		case 1:
			idx << -1,0,0;
			break;
		// POSITIVE Y(FORWARD)
		case 2:
			idx << 0,1,0;
			break;
		// NEGATIVE Y (BACKWARD)
		case 3:
			idx << 0,-1,0;
			break;
		// POSITIVE Z (UP)
		case 4: 
			idx << 0,0,1;
			break;
		// NEGATIVE Z (DOWN)
		case 5:
			idx << 0,0,-1;
			break;
		// STAY
		default:
			idx << 0,0,0;
			break;
	}
	return idx;
}

bool Chain::ok()
{
	return _ok;
}
