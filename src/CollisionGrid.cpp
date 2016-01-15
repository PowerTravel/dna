#include "CollisionGrid.hpp"
#include <iostream>

CollisionGrid::CollisionGrid(double bs)
{
	box_size = bs;
	_c = NULL;
	ok = false;
	active = false;
}

CollisionGrid::~CollisionGrid()
{

}

void CollisionGrid::set_up(Chain* c)
{
	if(!c->ok())
	{
		std::cerr << "ERROR: Chain not OK." << std::endl;
		std::cerr << "Error posted from CollisionGrid::set_chain." << std::endl;
		exit(1);
	}else{
		_c = c;
	}

	
	grid = std::map<idx_type, std::vector<int> >();
	geoms = std::vector< geom_struct >();

	// Get the distance of hte longest axis of the chain
	max_idx = get_max_axis(c);

	Chain::link l =  c->get_link(0);
	std::vector<idx_type> v = get_intersection_keys( l.sphere );
	for(auto key = v.begin(); key != v.end(); key++)
	{
			push_key_to_map(*key,0);
	}

	for(int i = 1; i < c->len(); i++)
	{
		l = c->get_link(i);
		
		v = get_intersection_keys( l.cyl1 );
		
		for(auto key = v.begin(); key != v.end(); key++)
		{
			push_key_to_map(*key,i);
		}

		v = get_intersection_keys( l.sphere );
		for(auto key = v.begin(); key != v.end(); key++)
		{
			push_key_to_map(*key,i);
		}
	}

	ok = true;

}


void CollisionGrid::push_key_to_map(idx_type key, int val)
{
	std::vector<int> vec = std::vector<int>();
	
	if( grid.find(key) != grid.end() )
	{
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

std::vector<idx_type> CollisionGrid::get_intersection_keys(std::shared_ptr<CollisionGeometry> g)
{
	Eigen::ArrayXd span = g->get_span(); // Get an AABB for the collision geometry
	Eigen::ArrayXd idx = span/box_size; // Divide by the box size to get the box indexes 
	idx = idx+max_idx +1/2.0; // Shift all the indices to be positive
	//std::cout << idx.transpose() << std::endl;
	std::vector<idx_type> ret = std::vector<idx_type>();


	std::vector<idx_type> keyv = std::vector<idx_type>();
	std::vector<Eigen::Vector3d> idxv = std::vector<Eigen::Vector3d>();

	for(int i = std::floor(idx(0)); i<idx(1); i++)
	{
		for(int j =std::floor(idx(2)); j<idx(3); j++)
		{
			for(int k = std::floor(idx(4)); k<idx(5); k++)
			{
				//std::cout << i << ", "<< j<<",  "<< k << std::endl;

				keyv.push_back(map_key(i,j,k));
				idxv.push_back(Eigen::Vector3d(i,j,k));

				ret.push_back( map_key(i,j,k) );
			}
		}
	}

	if(!ok)
	{
		geom_struct gs;
		gs.cg = g;
		gs.key = keyv;
		gs.idx = idxv;
		gs.s = box_size;
		gs.m_idx = max_idx;
		geoms.push_back(gs);
	}
	
	if(active)
	{
		geom_struct ags;
		ags.cg = g;
		ags.key = keyv;
		ags.idx = idxv;
		ags.s = box_size;
		ags.m_idx = max_idx;
		active_geom = ags;
	}
	return ret;
}

idx_type CollisionGrid::map_key(int i, int j, int k)
{
	return i + j*max_idx + k * max_idx * max_idx;
}

/*
int CollisionGrid::grid_map(int link, Eigen::Vector3d v)
{
	 return 0;	
}
*/
std::vector< std::shared_ptr<CollisionGeometry> > CollisionGrid::get_collision_bodies(std::shared_ptr<CollisionGeometry> g)
{	
	std::vector< std::shared_ptr<CollisionGeometry> > ret;
	if(_c == NULL || !_c->ok())
	{
		std::cerr << "Error: grid not set up. Error thrown from CollisionGrid::get_collision_bodies()" << std::endl;
		return ret;
	}

	std::vector<idx_type> v1 = get_intersection_keys(g);
	for(auto key = v1.begin(); key != v1.end(); key++)
	{
		if( grid.find(*key) != grid.end() )
		{
			// Push all the possible intersection bodies to the return vector
			std::vector<int> l = grid[*key];
			for(auto link_idx = l.begin(); link_idx != l.end(); link_idx++ ) 
			{
				ret.push_back( _c->get_link(*link_idx).sphere );
				if(*link_idx != 0)
				{
					ret.push_back( _c->get_link(*link_idx).cyl1 );
				}
				if(*link_idx != _c->len()-1 ){
					ret.push_back( _c->get_link(*link_idx).cyl2 );
				}
			}
		}
	}

	return ret;
}

void CollisionGrid::print_intersecting_box_corners(cg_ptr g)
{
//	std::ostream os;
	// get_intersection_keys updates the active_geoms vector with the 
	// debug data 

	// Active är en variabel som bestämmer om vi ska uppdatera active_geom när vi
	// kallar get_intersection_keys
	active = true;
	get_intersection_keys(g);
	acive = false;
	//std::vector< cg_ptr > CollisionGrid::get_collision_bodies(std::shared_ptr<CollisionGeometry> g)
	if(active_geom.cg->text_type().compare("Sphere")==0)
	{
		std::cout << "1 ";
	}
	if(active_geom.cg->text_type().compare("Cylinder")==0)
	{
		std::cout << "2 ";
	}
	std::cout << active_geom.cg->get_span().transpose();
	int len = active_geom.idx.size();
	for(int i = 0; i < len; i++)
	{
			Eigen::Array3d ax = active_geom.idx[i];
			// transpose the indices to actucal positions
			ax = (ax - active_geom.m_idx-0.5)*active_geom.s;
			// a box
			std::cout << "0 "<< ax.transpose() << " 0 0 0 " <<std::endl;





			//os << *key << std::endl;
	}
//	return os;
}

void CollisionGrid::print_box_corners(std::string path)
{
	if(_c == NULL)
	{
		return;
	}

	std::ofstream file;
	file.open(path, std::fstream::out | std::fstream::trunc);
	if(file.is_open()){
		for(auto A = geoms.begin(); A != geoms.end(); A++)
		{
			cg_ptr cgp = A->cg;
			if(cgp->text_type().compare("Sphere")==0)
			{
				//std::cout << "Sphere"<< std::endl;
				file <<"1 ";
			}
			if(cgp->text_type().compare("Cylinder")==0)
			{
				//std::cout << "Cylinder"<< std::endl;
				file  <<"2 ";
			}
			//file << cgp->text_type() << " " ;//<<std::endl;
			file << cgp->get_span().transpose() << std::endl;
			for(auto B = A->idx.begin(); B!=A->idx.end(); B++)
			{
				Eigen::Array3d ax = *B;
				ax = (ax - A->m_idx-0.5)*A->s;
				file << "0 " << ax.transpose() << " 0 0 0"  << std::endl;
			}
		}
	}else{
		std::cerr << "Failed to open " << path << std::endl;
	}

	file.close();
}
