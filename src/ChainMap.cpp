#include "ChainMap.hpp"
#include <climits>
#include <iostream>

ChainMap::ChainMap(int size)
{
	// The max size of our grid is the average walk distance of our SAW
	//mod_size = pow(size,0.588);
	mod_size = floor(size/2.f); // arbitrary so we get collisions for testing

	x_prime = X_PRIME;
	y_prime = Y_PRIME;
	z_prime = Z_PRIME;

	if(mod_size >= pow(  MAX_MAP_SIZE, 1/3.f) )
	{
		// If average size greater than third root of idx_type max we truncate it
		mod_size =floor( pow( MAX_MAP_SIZE, 1/3.f)-1 ) ;
	}
	
	_map = -1 * Eigen::ArrayXi::Ones( pow(mod_size,3) );
	_neighbours = Eigen::ArrayXi::Zero( pow(mod_size,3) );
	_chain = std::vector<link>(size);
	_i=0;
}

ChainMap::~ChainMap()
{
	
}

void ChainMap::print()
{
	for(int i = 0; i<mod_size ; i++)
	{
		for(int j = 0; j<mod_size ; j++)
		{
			for(int k = 0; k<mod_size ; k++)
			{
				std::cout << _neighbours(i*mod_size*mod_size+j*mod_size+k);
			}
			std::cout << std::endl;
		}
		std::cout << std::endl;
	}
}

void ChainMap::push(Eigen::Array3d pos)
{
	// Hash position
	Eigen::Array3d hp = hash_function(pos);

	// Get hashed index
	idx_type idx = get_idx(hp);

	// Check if the index is occupied
	int oufh = 0;

	std::cout <<"trying "<< idx << " " << hp.transpose()<< std::endl;
	while( _map(idx) != -1  && oufh<5){
		hp = hash_function(hp);
		idx = get_idx(hp);
		std::cout <<"koll"<< idx << " " << hp.transpose()<< std::endl;
		oufh++;
	}

	std::cout <<"pos = " << pos.transpose() <<" hp = " <<  hp.transpose()<< "  idx = " << idx << std::endl;

	add_position(pos,idx);
}

void ChainMap::add_position(Eigen::Array3d pos, idx_type idx)
{
	increment_neighbours(pos);
	_chain[_i] = link(pos);
	_map(idx) = _i;
	_i++;
}

int ChainMap::get_nr_neigbours(Eigen::Array3d pos)
{
	int n = 0;
	for(int i = 0; i<6; i++){
		Eigen::Array3d p = pos + int_to_coord(i);
		Eigen::Array3d hp = hash_function(p);
		idx_type idx = get_idx(hp);
	
		while(  ( _map(idx) != -1 ) && 
				( !compare_position( _chain[_map(idx)].pos ,  p) ) ){
			hp = hash_function(hp);
			idx = get_idx(hp);
		}
		n += _neighbours(idx);
	}
	return n;
}


void ChainMap::increment_neighbours(Eigen::Array3d pos)
{
	for(int i = 0; i<6; i++){
		Eigen::Array3d p = pos + int_to_coord(i);
		Eigen::Array3d hp = hash_function(p);
		idx_type idx = get_idx(hp);
	
		while(  ( _map(idx) != -1 ) && 
				( !compare_position( _chain[_map(idx)].pos ,  p) ) ){
			hp = hash_function(hp);
			idx = get_idx(hp);
		}
		_neighbours(idx)++;
	}
}


bool ChainMap::compare_position(Eigen::Array3d pos1,Eigen::Array3d pos2)
{
	if( ( pos1(0) == pos2(0) ) &&
		( pos1(1) == pos2(1) ) &&
		( pos1(2) == pos2(2) ) )
		{
			return true;
		}
	return false;
}

Eigen::Array3d ChainMap::int_to_coord(int i)
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
Eigen::ArrayXXd ChainMap::as_array()
{
	Eigen::ArrayXXd ret = Eigen::ArrayXXd::Zero(3, _chain.size() );
	for(int i=0; i < _chain.size(); i++)
	{
		ret.block(0,i,3,1) = _chain[i].pos;
	}
	return ret;
}

// This hashes the position down to a position that fits into the hash_map
Eigen::Array3d ChainMap::hash_function(Eigen::Array3d pos)
{
	
	//int shiftwidth = mod_size / 2; 
	// Jag skiter i att shifta negativa positioner.
	// Om en kedja når mod_size så äre otroligt att de även skulle nå - mod size
	int x = fmod( (pos(0) +1) * x_prime,mod_size );
	int y = fmod( (pos(1) +1) * y_prime,mod_size );
	int z = fmod( (pos(2) +1) * z_prime,mod_size );

	return Eigen::Array3d(x,y,z);
}

// This converts the hashed position into an index which can be accesed in the hash_map
idx_type ChainMap::get_idx(Eigen::Array3d hashed_pos)
{
	return hashed_pos(0) + mod_size*hashed_pos(1) + pow(mod_size,2) * hashed_pos(2);
}
