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
VecXd Chain::get_density_boundary(double density)
{
	double l = _link_len;
	double r = _rad;
	if(!_ok)
	{
		return VecXd::Zero(6);
	}
	
	ArrXd axis = span();
	int I = std::ceil( axis(1)-axis(0) )+1;
	int J = std::ceil( axis(3)-axis(2) )+1;
	int K = std::ceil( axis(5)-axis(4) )+1;
	
	// we are gonna use the particles position as an index in our int array
	double Xoffset = axis(0);
	double Yoffset = axis(2);
	double Zoffset = axis(4);
	int Xmiddle = I/2.f;
	int Ymiddle = J/2.f;
	int Zmiddle = K/2.f;
	int* box = new int[I*J*K]();
	Arr3d CM = Arr3d();
	int idx = 0;
	
	for(int i = 0; i<len(); i++)
	{
		int px = std::round(_chain(0,i) - Xoffset);
		int py = std::round(_chain(1,i) - Yoffset);
		int pz = std::round(_chain(2,i) - Zoffset);
//		std::cerr << Arr3d(px,py,pz).transpose()<<std::endl;
//		CM = CM + Arr3d(px,py,pz);
		int idx = get_3d_array_index(px,py,pz,I,J,K);
		box[idx] = 1;
	}
//	CM = CM/len();
//	std::cerr << CM.transpose()<<std::endl;
//	exit(1);

	/*
		Strategy:
	
			We have a box with ones repressenting sites where we have particles.
			We want to find a rectangular span that has the biggest volume we can get
			
			add outer layers for each of the 6 directions in succession
			Add all the sides whos density to volume ration is 1. 
			Keep adding layers while the density-volume ration for all sides greater than the given density value. 
	
			Should all the layers of the added region have a density below the given density value, 
			expand in all directions untill the total density is below the given value
			
		
			We will make use of a 
			Boundary vector[6] 
				- The boundary which will be used to select the collision chain
			Sites_To_Add[6] vector
				- Each index keeps track of how many particles to add for the layer racing that direction
			Sites_To_Advance[6] vector
				- Each index keeps track of how many sites the layer has
			Density_ratio_vector[6]
				- Density ratio of each sides

	*/

	Vec3d cmVec = this->cm(0,len());
	int added = 0;
	int advanced = 0;
	double Boundary[6] ={};
	Boundary[0] = Xmiddle;
	Boundary[1] = Xmiddle;
	Boundary[2] = Ymiddle;
	Boundary[3] = Ymiddle;
	Boundary[4] = Zmiddle;
	Boundary[5] = Zmiddle;
		
	idx = 0;
	double to_add = 0;
	double to_advance = 0;
	double density_ratio = 0;
	
	int sides_to_stall = 0;
	double added_total = 0;
	double advanced_total = 0;
	double density_ratio_total = 0;
	
	double Stop = false;
	bool halt[6] = {};
	while(!Stop)
	{
		for(int CalcSide = 0; CalcSide<6; CalcSide++)
		{
			to_add = 0;
			to_advance = 0;
			density_ratio = 0;
			bool skipped = true;
			
			int XMin = Boundary[0];
			int XMax = Boundary[1];
			
			int YMin = Boundary[2];
			int YMax = Boundary[3];
	
			int ZMin = Boundary[4];
			int ZMax = Boundary[5];
			
			if((sides_to_stall >= 6) || (halt[CalcSide]==false) )
			{		
				switch(CalcSide)
				{
					// Negative X
					case 0:
					{
						// We add alues only if this side still contributes more than the desired value or
						// all other sides contribution is less than desired value
						// ( optimization stops )
							XMin -= 1;		
							for(int y = YMin; y<YMax+1; y++ )
							{
								for( int z = ZMin; z<ZMax+1; z++ )
								{
									int idx = get_3d_array_index(XMin,y,z,I,J,K);
									if( box[idx]==1)
									{
										to_add += 1;
									}
									to_advance += 1;
								}
							}
								
						
					}break;
					// Positive X
					case 1:
					{
						XMax += 1;
						for(int y = YMin; y<YMax+1; y++ )
						{
							for( int z = ZMin; z<ZMax+1; z++ )
							{
								int idx = get_3d_array_index(XMax,y,z,I,J,K);
								if( box[idx]==1)
								{
									to_add+=1;
								}
								to_advance+=1;
							}
						}	
					
					}break;
					// Negative Y
					case 2:
					{
						YMin -= 1;
						for( int x = XMin; x<XMax+1; x++ )
						{
							for( int z = ZMin; z<ZMax+1; z++ )
							{
								int idx =  get_3d_array_index(x,YMin,z,I,J,K);
								if( box[idx]==1)
								{
									to_add+=1;
								}
								to_advance+=1;
							}
						}
					}break;
					// Positive Y
					case 3:
					{
						YMax += 1;
						for(int x = XMin; x<XMax+1; x++ )
						{
							for( int z = ZMin; z<ZMax+1; z++ )
							{
								int idx = get_3d_array_index(x,YMax,z,I,J,K);
								if( box[idx]==1)
								{
									to_add+=1;
								}
								to_advance+=1;
							}
						}
					}break;
					// Negative Z
					case 4:
					{
						ZMin -= 1;
						for(int x = XMin; x<XMax+1; x++ )
						{
							for( int y = YMin; y<YMax+1; y++ )
							{
								int idx = get_3d_array_index(x,y,ZMin,I,J,K);
								if( box[idx]==1)
								{
									to_add+=1;
								}
								to_advance+=1;
							}
						}	
					}break;
					// Positive Z			
					case 5:
					{
						ZMax += 1;
						for(int x = XMin; x<XMax+1; x++ )
						{
							for( int y = YMin; y<YMax+1; y++ )
							{
								int idx = get_3d_array_index(x,y,ZMax,I,J,K);
								if( box[idx]==1)
								{
									to_add+=1;
								}
								to_advance+=1;
							}
						}
        		
					}break;
				}
			}
			
			// If all the sides adds less than the desired density ratio we keep adding sides untill total
			// denisty is just above the desired value;
			if( sides_to_stall < 6 )
			{
				if(to_advance > 0)
				{
					density_ratio = to_add/to_advance;
				}else{
					density_ratio = 0;
				}
				
				if(density_ratio >= density)
				{
					int add_val = ((CalcSide+1) % 2 ? -1:1 );
					Boundary[CalcSide] += add_val;
					advanced_total += to_advance;
					added_total += to_add;
				}else{
					halt[CalcSide] = true;
					sides_to_stall++;
				}
			}else{
				double total_density_ratio_tmp = (added_total + to_add)/( advanced_total+ to_advance );
				if(total_density_ratio_tmp >= density)
				{
					int add_val = ((CalcSide+1) % 2 ? -1:1 );
					Boundary[CalcSide] += add_val;
					
					added_total += to_add;
					advanced_total += to_advance;
				}else{
					Stop = true;
				}
			}
			
			if( (Boundary[0]<0) || (Boundary[1]<0) || (Boundary[2]<0) ||
				(Boundary[3]<0) || (Boundary[4]<0) || (Boundary[5]<0))
			{
				std::cerr<< "void Chain::get_collision_vec(double density): " <<std::endl;
				exit(1);
			}
			
		}
	}

	std::cout<< Boundary[0] << ", "<< Boundary[1] << ", "<< Boundary[2] << ", "<< Boundary[3] << ", "<< Boundary[4] << ", "<< Boundary[5] << ", " << std::endl;
	Boundary[0] += Xoffset - 0.5;
	Boundary[1] += Xoffset + 0.5;
	Boundary[2] += Yoffset - 0.5;
	Boundary[3] += Yoffset + 0.5;
	Boundary[4] += Zoffset - 0.5;
	Boundary[5] += Zoffset + 0.5;
	std::cout<< Boundary[0] << ", "<< Boundary[1] << ", "<< Boundary[2] << ", "<< Boundary[3] << ", "<< Boundary[4] << ", "<< Boundary[5] << ", " << std::endl;
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
	//std::cerr << "Chain.cpp:get_collision_vec" <<std::endl;
	//std::cerr << "\tNOTE: Not adding cylinders untill spheres work"<<std::endl;
	// one for each site + each link
	//int nr_geoms = 2*len() - 1;
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
	//	std::cerr << "Chain.cpp:get_collision_vec" <<std::endl;
	//	std::cerr << "\tNOTE: Not adding cylinders untill spheres work"<<std::endl;
	// one for each site + each link
	//int nr_geoms = 2*len() - 1;
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

		// Skip cylinders for now
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
	cm(0) = std::round(cm(0))+0.5;
	cm(1) = std::round(cm(1))+0.5;
	cm(2) = std::round(cm(2))+0.5;
	for(int i = 0; i<this->len(); i++)
	{
		_chain.block(0,i,3,1) = _chain.block(0,i,3,1) - cm;
	}
}
