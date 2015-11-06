#include "Chain.hpp"
#include <iostream>
#include <cmath>
#include <iomanip>
#include <ctime>
#include <limits>

std::default_random_engine Chain::_generator = std::default_random_engine(time(NULL));

Chain::Chain()
{
}

Chain::~Chain()
{

}

Eigen::ArrayXXd Chain::as_array(int start, int size)
{
	return _chain.block(0,start,3,size);
}
std::ostream& operator<<(std::ostream& os, const Chain& c)
{
	os << c._chain.transpose() << std::endl;
	return os;
}

double Chain::Rg(int start, int size)
{
	Eigen::Array3d mean = cm(start, size);
	double var = 0;
	for(int i = start; i<start+size; i++)
	{
		//Eigen::Vector3d v = (_chain.block(0,i,3,1) - mean).matrix();
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

double Chain::weight()
{
	return _weight;
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

