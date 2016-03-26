#include "CollisionGrid2.hpp"

CollisionGrid::CollisionGrid()
{
	_grid_box_size  = 1;

}

CollisionGrid::CollisionGrid(double box_size)
{
	_grid_box_size = box_size;
}

CollisionGrid::~CollisionGrid()
{

}
bool CollisionGrid::set_up(std::vector<cg_ptr> v)
{
	_collision_bodies = v;
	int N = _collision_bodies.size();

	VecXd span;
	if(N>0)
	{
		span = v[0]->get_span();
	}else{
		return false;
	}

	for(int i = 1; i<N; i++)
	{
		VecXd tmp_span = _collision_bodies[i]->get_span();

		if(span(0) > tmp_span(0))
		{
			span(0) = tmp_span(0);	
		}
		if(span(1) < tmp_span(1))
		{
			span(1) = tmp_span(1);	
		}
		if(span(2) > tmp_span(2))
		{
			span(2) = tmp_span(2);	
		}
		if(span(3) < tmp_span(3))
		{
			span(3) = tmp_span(3);	
		}
		if(span(4) > tmp_span(4))
		{
			span(4) = tmp_span(4);	
		}
		if(span(5) < tmp_span(5))
		{
			span(5) = tmp_span(5);	
		}
	}
	_T <<  span(0), span(2), span(4);
	_S <<  span(1)-span(0), span(3) - span(2), span(5) - span(4);
	double _max_axis_length = _S(0);	
	if(_max_axis_length < _S(1))
	{
		_max_axis_length = _S(1);
	}
	if(_max_axis_length < _S(2))
	{
		_max_axis_length = _S(2);
	}
	_max_box_idx = std::floor(_max_axis_length/_grid_box_size )+1;
	
	// TODO Check so that 
	//		_max_axis_lengt + 
	//		_max_axis_lengt*_max_box_idx + 
	//		_max_axis_lengt*_max_box_idx * _max_axis_lengt
	// does not overflow
	
	
	for(int collision_idx=0; collision_idx<N; collision_idx++)
	{
		VecXd span = _collision_bodies[collision_idx]->get_span();
		std::vector<unsigned int> key_chain = get_keys(span);
	
		for(int i = 0; i<key_chain.size(); i++)
		{
			unsigned int key = key_chain[i];
			std::vector<int>& collision_idx_vector = _grid[key];
			collision_idx_vector.push_back(collision_idx);
		}
	}

	return true;
}

std::vector< cg_ptr > CollisionGrid::get_collision_bodies(std::shared_ptr<CollisionGeometry> g)
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
	std::vector<Vec3i> idx_vec = get_idx(span);
	
	std::vector<unsigned int> key_chain;
	for(int i = 0; i<idx_vec.size(); i++ )
	{
		Vec3i idx = idx_vec[i];
		unsigned int key =idx(0)+idx(1)*_max_box_idx + idx(2)*_max_box_idx*_max_box_idx;
		key_chain.push_back(key);
	}

	return key_chain;
}

std::vector<Vec3i> CollisionGrid::get_idx(VecXd span)
{
	Vec3d min_span, max_span, low_idx, high_idx;
	std::vector<Vec3i> idx_vec;

	min_span << span(0), span(2), span(4);
	max_span << span(1), span(3), span(5);
	min_span << clamp(min_span);
	max_span << clamp(max_span);

	low_idx = (min_span - _T)/_grid_box_size;
	high_idx = (max_span - _T)/_grid_box_size;
	
	for(int i = low_idx(0); i<high_idx(0); i++)
	{
		for(int j = low_idx(1); j<high_idx(1); j++)
		{
			for(int k = low_idx(2); k<high_idx(2); k++)
			{
				Vec3i idx = Vec3i(i,j,k);
				idx_vec.push_back(idx);
			}
		}
	}

	return idx_vec;
}

Vec3d CollisionGrid::clamp(Vec3d v)
{
	Vec3d ret = v;
	Vec3d bottom = _T;
	Vec3d top = _T + _S; 

	for(int i = 0; i<3; i++)
	{
		if(ret(i)<bottom(i))
		{
			ret(i)=bottom(i);
		}else if( ret(i)>top(i) ){
			ret(i)=top(i);
		}
	}
	return ret;
}


bool CollisionGrid::run_tests()
{
	std::cerr << "CollisionGrid:run_tests()" << std::endl;

	std::cerr << "\tcollision_grid_test_one_sphere_A() ";
	if(!collision_grid_test_one_sphere_A())
	{
		std::cerr << "\tfailed" << std::endl;
		return false;
	}else{
		std::cerr << "\tSucceeded" << std::endl;
	}
	return true;
}

#include "Sphere.hpp"
bool CollisionGrid::collision_grid_test_one_sphere_A()
{
	CollisionGrid g = CollisionGrid(2);
	std::vector<cg_ptr> v;
	v.push_back(cg_ptr(new Sphere(Vec3d(-1,-1,-1), 1 )));
	g.set_up(v);

	cg_ptr S = cg_ptr(new Sphere(Vec3d(1,1,1), 1 ));
	std::vector<cg_ptr> iv = g.get_collision_bodies(S);

	if(iv.size()!= 0)
	{
		return false;
	}


	S = cg_ptr(new Sphere(Vec3d(-1.2,-1.2,-1.2), 1.2 ));
	iv = g.get_collision_bodies(S);
	if(iv.size()!= 1)
	{
		return false;
	}

	return true;

}


#include <fstream>
void CollisionGrid::print_box_corners(std::string path)
{
	if(_collision_bodies.empty())
	{
		return;
	}

	std::ofstream file;
	file.open(path, std::fstream::out | std::fstream::trunc);
	if(file.is_open()){
		file << _grid_box_size <<" 0 0 0 0 0 0" << std::endl;
		for(int i = 0; i < _collision_bodies.size(); i++)
		{
			cg_ptr cgp =  _collision_bodies[i];
			if(cgp->text_type().compare("Sphere")==0)
			{
				file <<"1 ";
			}
			if(cgp->text_type().compare("Cylinder")==0)
			{
				file  <<"2 ";
			}

			VecXd span = cgp->get_span();
			VecXd span_translated = VecXd::Zero(6);
			span_translated(0) = span(0);// - _T(0);
			span_translated(1) = span(1);// - _T(0);
			span_translated(2) = span(2);// - _T(1);
			span_translated(3) = span(3);// - _T(1);
			span_translated(4) = span(4);// - _T(2);
			span_translated(5) = span(5);// - _T(2);
			file << span_translated.transpose() << std::endl;

			std::vector<Vec3i> idx_vec = get_idx(span);

			for( int j = 0; j < idx_vec.size(); j++)
			{
				Vec3i idx = idx_vec[j];
				Vec3d corner = Vec3d(idx(0),idx(1),idx(2));
				corner = corner* _grid_box_size + _T;
				file << "0 " << corner.transpose() << " 0 0 0"  << std::endl;
			}
		}
	}else{
		std::cerr << "Failed to open " << path << std::endl;
	}

	file.close();
}

