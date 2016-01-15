#include "Chain.hpp"
#include <iostream>
#include <cmath>
#include <iomanip>
#include <ctime>
#include <limits>

#include "Cylinder.hpp"
#include "Sphere.hpp"

std::default_random_engine Chain::_generator = std::default_random_engine(time(NULL));

Chain::Chain()
{	
	_ok = false;
	_selfint = false;
	_use_weights = true;
	_rad = 0.2;
	_link_len = 1.0;
}

Chain::~Chain()
{

}

void Chain::allow_selfintersection( bool as )
{
	_selfint = as;
}

void Chain::use_weights( bool uw )
{
	_use_weights = uw;
}

Eigen::ArrayXXd Chain::as_array(int start, int size)
{
	return _chain.block(0,start,3,size);
}
Eigen::ArrayXXd Chain::as_array()
{
	return _chain;
}

Eigen::Array3d Chain::axis_length()
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

void Chain::set_radius(double r)
{
	_rad = std::abs(r);
}

void Chain::set_link_length(double l)
{
	_link_len = std::abs(l);
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
	

std::vector< std::shared_ptr<CollisionGeometry> > Chain::get_collision_vec()
{
	double l = _link_len;
	double r = _rad;
	if(!_ok)
	{
		return std::vector< std::shared_ptr<CollisionGeometry> >();
	}

	// one for each site + each link
	//int nr_geoms = 2*len() - 1;
	std::vector< std::shared_ptr<CollisionGeometry> > cv = std::vector< std::shared_ptr<CollisionGeometry> >( );

	cv.push_back( std::shared_ptr<CollisionGeometry>( 
				new Sphere(_chain.block(0,0,3,1).matrix(), r) ));
	for( int i = 1; i < len(); i++ )
	{
		Eigen::Vector3d P = _chain.block(0,i-1,3,1).matrix() * l;
		Eigen::Vector3d Q = _chain.block(0,i,3,1).matrix() * l;
		cv.push_back(std::shared_ptr<CollisionGeometry>( new Cylinder( r, P , Q)));
		cv.push_back(std::shared_ptr<CollisionGeometry>(new Sphere( Q , r) ));
	}

	return cv;

}
