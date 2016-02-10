#include "CollisionGrid.hpp"
bool set_up(std::vector<cg_ptr> v)
{
	_collision_bodies = v;
	int N = _collision_bodies.size();

	VecXd span;
	if(N>0)
	{
		span = v[0].get_span();
	}else{
		return false;
	}

	for(int i = 1; i<N; i++)
	{
		VecXd tmp_span = _collision_bodies[i].get_span();

		if(span(0) < tmp_span(0))
		{
			span(0) = tmp_span(0);	
		}
		if(span(1) > tmp_span(1))
		{
			span(1) = tmp_span(1);	
		}
		if(span(2) < tmp_span(2))
		{
			span(2) = tmp_span(2);	
		}
		if(span(3) > tmp_span(3))
		{
			span(3) = tmp_span(3);	
		}
		if(span(4) < tmp_span(4))
		{
			span(4) = tmp_span(4);	
		}
		if(span(5) > tmp_span(5))
		{
			span(5) = tmp_span(5);	
		}
	}
	_T <<  span(0), span(2), span(4);
	_S <<  span(1)-span(0), span(3) - span(2), span(5) - span(4);
	_max_axis_length = _S(0);
	if(_max_axis_length < _S(1))
	{
		_max_axis_length = _S(1);
	}
	if(_max_axis_length < _S(2))
	{
		_max_axis_length = _S(2);
	}
	
	// Check that our grid_keys wont overflow
	unsigned int U = (unsigned int) _max_axis_length * _max_axis_length *_max_axis_length + _max_axis_length *_max_axis_length +_max_axis_length;
	if(U<0)
	{
		// The grid structure span is too large
		return false;
	}

	
	for(int collision_idx=0; collision_idx<N; collision_idx++)
	{
		VecXd span = _collision_bodies[collision_idx].get_span();
		std::vector<unsigned int> key_chain = get_keys(span);
	
		for(int i = 0; i<key_chain.size(); i++)
		{
			unsigned int key = key_chain[i];
			std::vector<int>& collision_idx_vector = _grid.at(key);
			collision_idx_vector.push_back(collision_idx);
		}
	}

	return true;
}

std::vector< cg_ptr > get_collision_bodies(std::shared_ptr<CollisionGeometry> g)
{
	VecXd span = g->get_span();	
	std::vector<unsigned int> key_chain = get_keys(span);
	std::vector< cg_ptr > ret_vec;

	for(int i = 0; i<key_chain.size(); i++)
	{
		unsigned int key = key_chain[i];
		if(_grid.find(key) != _grid.end() )
		{
			std::vector<int> collision_idx_vector = _grid.at(key);	
			for(int j = 0; j<collision_idx_vector.size(); j++)
			{
				int coll_geom_idx = collision_idx_vector[j]; 
				cg_ptr collision_geometry = _collision_bodies[coll_geom_idx];
				ret_vec.push_back(collision_geometry);
			}
		}
	}
	return ret_vec;
}


std::vector<unsigned int> CollisionGrid::get_keys(VecXd span)
{
	unsigned int x_min =(int) std::floor((span(0)-_T(0))/_grid_box_size);
	unsigned int x_max =(int) std::floor((span(1)-_T(0))/_grid_box_size)+1;
	unsigned int y_min =(int) std::floor((span(2)-_T(1))/_grid_box_size);
	unsigned int y_max =(int) std::floor((span(3)-_T(1))/_grid_box_size)+1;
	unsigned int z_min =(int) std::floor((span(4)-_T(2))/_grid_box_size);
	unsigned int z_max =(int) std::floor((span(5)-_T(2))/_grid_box_size)+1;
	
	std::vector<unsigned int> key_chain;
	for(unsigned int i = x_min; i<x_max; i++)
	{
		for(unsigned int j = y_min; j<y_max; j++)
		{
			for(unsigned int k = z_min; k<z_max; k++)
			{
				unsigned int key =i+j*_max_axis_length + k*_max_axis_length*_max_axis_length;
				key_chain.push_back(key);
			}
		}
	}

	return key_chain;
}
