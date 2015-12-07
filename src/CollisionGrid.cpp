#include "CollisionGrid.hpp"
#include <iostream>

CollisionGrid::CollisionGrid(double bs)
{
	box_size = bs;
}

CollisionGrid::~CollisionGrid()
{

}

void CollisionGrid::set_up(Chain* c)
{
	std::cerr << "Creating grid " << std::endl;
	if(!c->ok())
	{
		std::cerr << "ERROR: Chain not OK." << std::endl;
		std::cerr << "Error posted from CollisionGrid::set_chain." << std::endl;
		exit(1);
	}

	// Get the distance of hte longest axis of the chain
	max_idx = get_max_axis(c);

	for(int i = 0; i < c->len(); i++)
	{
		std::vector<idx_type> v = get_intersection_keys(c->get_link(i));

		for(auto key = v.begin(); key != v.end(); key++)
		{
			push_key_to_map(*key,i);
		}
	}
	std::cout << grid.size() <<  std::endl;
}

void CollisionGrid::push_key_to_map(idx_type key, int val)
{
	std::vector<int> vec;
	if( grid.find(key) == grid.end() )
	{
		vec = std::vector<int>();
	}else{
		vec = grid[key];
	}
	vec.push_back(val);
	grid[key] = vec;
}

int CollisionGrid::get_max_axis(Chain* c)
{
	Eigen::ArrayXd span  = c->axis_length();
	double max_axis = span(0);
	if( max_axis < span(1) )
	{
		max_axis = span(1);	
	}
	if( max_axis < span(2) )
	{
		max_axis = span(2);	
	}

	if(std::pow(2*max_axis,3) >= MAX_MAP_SIZE)
	{
		std::cerr << "ERROR: Chain not too large to work with." << std::endl;
		std::cerr << "Error posted from CollisionGrid::set_chain." << std::endl;
		exit(1);
	}

	return std::floor(max_axis/box_size) + 1;
}

std::vector<idx_type> CollisionGrid::get_intersection_keys(Chain::link l)
{
	Eigen::ArrayXd span = l.geom->get_span();
	Eigen::ArrayXd idx = span/box_size;
	idx = idx+max_idx; // Shift all the indices to be positive
	std::vector<idx_type> ret = std::vector<idx_type>();
//	std::cout << max_idx << std::endl;
//	std::cerr << idx.transpose() << std::endl;
	for(int i = idx(0); i<idx(1); i++)
	{
		for(int j = idx(2); j<idx(3); j++)
		{
			for(int k = idx(4); k<idx(5); k++)
			{
				ret.push_back( i + j*max_idx + k * max_idx * max_idx );
//				if(grid.find(ret.back()) == grid.end())
//				{
//					std::cout << i << ", " << j << ", " << k << std::endl;
//				}
			}
		}
	}
	return ret;
}

int CollisionGrid::grid_map(int link, Eigen::Vector3d v)
{
	 return 0;	
}
