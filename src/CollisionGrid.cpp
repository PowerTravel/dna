#include "CollisionGrid.hpp"
#include <iostream>

CollisionGrid::CollisionGrid(double bs)
{
	box_size = bs;
	ok = false;
	active = false;
}

CollisionGrid::~CollisionGrid()
{

}

void CollisionGrid::set_up(std::vector<cg_ptr> v, idx_type m_axis)
{
	geom_vec = v;
	set_max_axis(m_axis);

	grid = std::map<idx_type, std::vector<int> >();
	geoms = std::vector< geom_struct >();

	int c_len = geom_vec.size();
	for(int i = 0; i<c_len; i++)
	{
		cg_ptr cptr = geom_vec[i];
		std::vector<idx_type> v = get_intersection_keys( cptr );
		for(auto key = v.begin(); key != v.end(); key++)
		{
			push_key_to_map(*key, i);
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

void CollisionGrid::set_max_axis( idx_type max_axis )
{
	if( std::pow(2*max_axis,3) >= MAX_MAP_SIZE )
	{
		idx_type m = std::pow( MAX_MAP_SIZE , 1.0/3.0)/2.0;
		std::cerr << "ERROR: Chain too large to work with." << std::endl;
		std::cerr << "input size is: " << max_axis << std::endl;
		std::cerr << "max allowed size is: " << m << std::endl;
		std::cerr << "Error posted from CollisionGrid::set_max_axis." << std::endl;
		std::cerr << "exiting." << std::endl;
		exit(1);
	}

	max_idx = std::floor(max_axis/box_size) + 1;
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
	if(!ok)
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
				ret.push_back(geom_vec[*link_idx]);
			}
		}
	}

	return ret;
}

int CollisionGrid::get_max(Eigen::Array3d v)
{
	double max = v(0);
	if( max < v(1) )
	{
		max = v(1);	
	}
	if( max < v(2) )
	{
		max = v(2);	
	}
	return max;
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
	active = false;
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
	if(!ok)
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
