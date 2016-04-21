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

ArrXd Chain::span()
{
	Eigen::ArrayXd sp = Eigen::ArrayXd::Zero(6);
	for(int i = 0; i<len(); i++)
	{
		Eigen::Array3d x = _chain.block(0,i,3,1);

		if( x(0) < sp(0) ){
			sp(0) = x(0);
		}else if(  x(0) > sp(1) ){
			sp(1) = x(0);
		}
		
		if( x(1) < sp(2) ){
			sp(2) = x(1);
		}else if(  x(1) > sp(3) ){
			sp(3) = x(1);
		}

		if( x(2) < sp(4) ){
			sp(4) = x(2);
		}else if(  x(2)> sp(5) ){
			sp(5) = x(2);
		}
	}
	return sp;
}

Eigen::Array3d Chain::axis_length()
{
	ArrXd sp = span();
	Arr3d ret(sp(1)-sp(0), sp(2)-sp(3), sp(5)-sp(4)  );
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

int Chain::get_3d_array_index(int i, int j, int k, int I, int J, int K)
{
	int Result = k+K*j+J*K*i;
	return k+K*j+J*K*i;
}
#include<cmath>

int Chain::GetSideIdx(int side, int offset, int width, int height, int I,int J, int K)
{
	int idx = 0;
	switch(side)
	{
		// offset = -X, Width = Y, Height = Z
		case 0:
		{
			idx = get_3d_array_index( offset, width, height, I, J, K);
		}break;
		
		// offset = X, Width = Y, Height = Z
		case 1:
		{
			idx = get_3d_array_index( offset, width, height, I, J, K);
		}break;
		
		// offset = -Y, Width = X, Height = Z
		case 2:
		{
			idx = get_3d_array_index( width, offset, height, I, J, K);
		}break;
		
		// offset = X, Width = Y, Height = Z
		case 3:
		{
			idx = get_3d_array_index( width, offset, height, I, J, K);
		}break;
		
		// offset = -Z, Width = Y, Height = X
		case 4:
		{
			idx = get_3d_array_index( height, width, offset, I, J, K);
		}break;
		
		// offset = X, Width = Y, Height = Z
		case 5:
		{
			idx = get_3d_array_index( height, width, offset, I, J, K);		
		}break;
		default:{
			
		}break;
	}
	return idx;
}
void Chain::CalcSideDensity(int* box, int I, int J, int K, side_density* s, int CalcSide)
{

	int s1Min = *s[CalcSide].wMin;
	int s1Max = *s[CalcSide].wMax;
	int s2Min = *s[CalcSide].hMin;
	int s2Max = *s[CalcSide].hMax;
	int offset = *s[CalcSide].offset;
	
	// Corner Occupancy;
	s[CalcSide].CO = 0;
	s[CalcSide].EO = 0;
	s[CalcSide].BO = 0;
	s[CalcSide].Vol = 0;
	
	
	// interior
	for(int width = s1Min; width < s1Max+1; ++width )
	{
		for(int height = s2Min; height < s2Max+1; ++height )
		{
			int idx = GetSideIdx(CalcSide,offset,width,height,I,J,K);
			int occupied = box[idx];
			// Corners
			if( ( (width == s1Min) && (height == s2Min) ) ||
				( (width == s1Min) && (height == s2Max) ) ||
				( (width == s1Max) && (height == s2Min) ) ||
				( (width == s1Max) && (height == s2Max) ))
			{
				s[CalcSide].CO += occupied;

			// Edges
			}else if( ( (width > s1Min) && (width < s1Max ) && (height == s2Min)   ) ||
					  ( (width > s1Min) && (width < s1Max ) && (height == s2Max)   ) ||
					  ( (height > s2Min) && (height < s2Max ) && (width == s1Min)  ) ||
					  ( (height > s2Min) && (height < s2Max ) && (width == s1Max) )){	
			
				s[CalcSide].EO += occupied;
				
			// Body	
			}else{	
				s[CalcSide].BO += occupied;	
				
			}

			s[CalcSide].Vol++; 
		}
	}
	s[CalcSide].Density = double(s[CalcSide].CO+s[CalcSide].EO+s[CalcSide].BO)/double(s[CalcSide].Vol);
	
}

VecXd Chain::get_density_boundary(double density)
{	
	double l = _link_len;
	double r = _rad;
	if(!_ok)
	{
		return VecXd::Zero(6);
	}
	
	ArrXd axis = span();
	int d = axis(1) - axis(0);
	int w = axis(3) - axis(2);
	int h = axis(5) - axis(4);
	int d0 =0;
	int w0 = 0;
	int h0 = 0;
	int I = std::ceil( d )+1;
	int J = std::ceil( w )+1;
	int K = std::ceil( h )+1;
	
	side_density sides[6] = {};
	
	// Xnegative, width = Y, Height = Z
	side_density xn = {};
	xn.wMin = &w0;
	xn.wMax = &w;
	xn.hMin = &h0;
	xn.hMax = &h;
	xn.offset = &d0;
	
	sides[0] = xn;
	
	// Xpositive, width = Y, Height = Z;
	side_density xp = xn;
	xp.offset = &d;
	
	sides[1] = xp;

	// Ynegative, width = Y, Height = Z
	side_density yn = {};
	yn.wMin = &d0;
	yn.wMax = &d;
	yn.hMin = &h0;
	yn.hMax = &h;
	yn.offset = &w0;
	
	sides[2] = yn;
	
	// Ypositive, width = X, Height = Z;
	side_density yp = yn;
	yp.offset = &w;
	
	sides[3] = yp;

	// Znegative, width = Y, Height = X
	side_density zn = {};
	zn.wMin = &w0;
	zn.wMax = &w;
	zn.hMin = &d0;
	zn.hMax = &d;
	zn.offset = &h0;
	sides[4] = zn;
	
	// Zpositive, width = Y, Height = X;
	side_density zp = zn;
	zp.offset = &h;
	sides[5] = zp;


	// we are gonna use the particles position as an index in our int array
	double Xoffset = axis(0);
	double Yoffset = axis(2);
	double Zoffset = axis(4);
	int* box = new int[I*J*K]();
	Arr3d CM = Arr3d();

	int ones = len();
	for(int i = 0; i<len(); i++)
	{
		int px = std::round(_chain(0,i) - Xoffset);
		int py = std::round(_chain(1,i) - Yoffset);
		int pz = std::round(_chain(2,i) - Zoffset);
		int idx = get_3d_array_index(px,py,pz,I,J,K);
		box[idx] = 1;
	}

	int bound[6] = {(int)axis(0),(int)axis(1),(int)axis(2),(int)axis(3),(int)axis(4),(int)axis(5)};
	double vol =I*J*K;
	double dens = double(ones)/double(vol);
	while(dens < density)
	{
		for(int CalcSide = 0; CalcSide<6; CalcSide++)
		{
			CalcSideDensity(box, I, J, K, sides, CalcSide);
			
		}
		
		int lowestDensityIdx = 0;
		double LowestDensity = sides[0].Density; 
	
		for(int CalcSide = 1; CalcSide<6; CalcSide++)
		{
			if(sides[CalcSide].Density < LowestDensity)
			{
				LowestDensity = sides[CalcSide].Density;
				lowestDensityIdx = CalcSide;
			}
			
		}
		
		int onesOnSide = (sides[lowestDensityIdx].CO + sides[lowestDensityIdx].EO+sides[lowestDensityIdx].BO);
		ones -= onesOnSide;
		vol -= sides[lowestDensityIdx].Vol;
		dens = double(ones)/double(vol);
		
		int addNr = ( ( ( lowestDensityIdx % 2 ) == 0 ) ? 1:-1 );
		*sides[lowestDensityIdx].offset += addNr;
	}

	double Boundary[6] = {(double) *sides[0].offset,(double) *sides[1].offset, (double) *sides[2].offset,
						  (double) *sides[3].offset,(double) *sides[4].offset, (double) *sides[5].offset };
	
	Boundary[0] += Xoffset - 0.5;
	Boundary[1] += Xoffset + 0.5;
	Boundary[2] += Yoffset - 0.5;
	Boundary[3] += Yoffset + 0.5;
	Boundary[4] += Zoffset - 0.5;
	Boundary[5] += Zoffset + 0.5;
	delete [] box;	
	
	VecXd b = VecXd::Zero(6);
	b << Boundary[0] , Boundary[1] , Boundary[2] , Boundary[3] , Boundary[4] , Boundary[5];
	
	return(b);
}


std::vector< cg_ptr > Chain::get_collision_vec()
{
	double l = _link_len;
	double r = _rad;
	if(!_ok)
	{
		return std::vector< std::shared_ptr<CollisionGeometry> >();
	}
	std::vector< std::shared_ptr<CollisionGeometry> > cv = std::vector< std::shared_ptr<CollisionGeometry> >( );

	cv.push_back( std::shared_ptr<CollisionGeometry>( 
				new Sphere(_chain.block(0,0,3,1).matrix(), r) ));
	for( int i = 1; i < len(); i++ )
	{
		Vec3d P = _chain.block(0,i-1,3,1).matrix() * l;
		Vec3d Q = _chain.block(0,i,3,1).matrix() * l;
		cv.push_back(std::shared_ptr<CollisionGeometry>( new Cylinder( r, P , Q)));
		cv.push_back(std::shared_ptr<CollisionGeometry>(new Sphere( Q , r) ));
	}
	
	return cv;
}

std::vector< cg_ptr > Chain::get_collision_vec(VecXd boundary)
{
	double l = _link_len;
	double r = _rad;
	if(!_ok)
	{
		return std::vector< std::shared_ptr<CollisionGeometry> >();
	}
	std::vector< std::shared_ptr<CollisionGeometry> > cv = std::vector< std::shared_ptr<CollisionGeometry> >( );

	// Only add geometries that completely fit the boundary
	Vec3d P = _chain.block(0,0,3,1).matrix();
	Vec3d Q = _chain.block(0,1,3,1).matrix() * l;
	bool use_P = false;
	bool use_Q = false;
	
	if( (P(0)-r >= boundary(0)) &&
		(P(0)+r <= boundary(1)) &&
		(P(1)-r >= boundary(2)) &&
		(P(1)+r <= boundary(3)) &&
		(P(2)-r >= boundary(4)) &&
		(P(2)+r <= boundary(5)))
	{
		use_P = true;
		cv.push_back( std::shared_ptr<CollisionGeometry>( new Sphere(P, r) ));
	}

	for( int i = 1; i < len(); i++ )
	{ 
		P = _chain.block(0,i-1,3,1).matrix() * l;
		Q = _chain.block(0,i,3,1).matrix() * l;
		
		if( (Q(0)-r >= boundary(0)) &&
			(Q(0)+r <= boundary(1)) &&
			(Q(1)-r >= boundary(2)) &&
			(Q(1)+r <= boundary(3)) &&
			(Q(2)-r >= boundary(4)) &&
			(Q(2)+r <= boundary(5)))
		{
			use_Q = true;
		}else{
			use_Q = false;
		}
	
		if(use_Q)
		{
			cv.push_back(std::shared_ptr<CollisionGeometry>(new Sphere( Q , r) ));
		}

		if(use_P && use_Q)
		{
			cv.push_back(std::shared_ptr<CollisionGeometry>( new Cylinder( r, P , Q)));
		}
		use_P = use_Q;
	}

	return cv;

}

void Chain::center_chain()
{
	if(!_ok)
	{
		return;
	}

	Eigen::Array3d cm = this->cm(0, this->len());
	cm(0) = std::round(cm(0));
	cm(1) = std::round(cm(1));
	cm(2) = std::round(cm(2));
	for(int i = 0; i<this->len(); i++)
	{
		_chain.block(0,i,3,1) = _chain.block(0,i,3,1) - cm;
	}
}

void Chain::set_chain_manually(Eigen::ArrayXXd c, bool rad)
{
	_chain = c;
	_link_len = 1;
	_rad = rad;
}
