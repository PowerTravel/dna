#include "CollisionGrid.hpp"
#include <iostream>

CollisionGrid::CollisionGrid()
{
	box_size = 10.0;
}

CollisionGrid::~CollisionGrid()
{

}

void CollisionGrid::set_chain(Chain* c)
{
	if(!c->ok())
	{
		std::cerr << "ERROR: Chain not OK." << std::endl;
		std::cerr << "Error posted from CollisionGrid::set_chain." << std::endl;
		exit(1);
	}
			
	Eigen::ArrayXd span  = c->span();
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

/*	
	Eigen::ArrayXXd tmp = c->as_array();
	r = Eigen::ArrayXXd::Zero( 4 , c->len() );
	r.block(1,0,3,c->len());

	for(int i = 0; i<c->len(); i++)
	{
		r(0, i) = i;
		r.block(1,i,3,1) = tmp.block(0,i,3,1);
	}
*/

	for(int i = 0; i < c->len(); i++)
	{
		// x, y and z are the indices of the box they will be placed in
		Eigen::Vector3d r = c->as_array(i,1);
		// Shift all indices to the positive quadrant
		double x = floor((r(0)+max_axis)/box_size);
		double y = floor((r(1)+max_axis)/box_size);
		double z = floor((r(2)+max_axis)/box_size);

		// set the map indexes with x+y*max_size+z*max_size*max_size
		idx_type key = x+y*box_size + z*box_size*box_size;
	}
}


int CollisionGrid::grid_map(int link, Eigen::Vector3d v)
{
	 return 0;	
}

