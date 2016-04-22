#include "Particle.hpp"
#include <iostream>
#include <algorithm>
#include <fstream>

std::default_random_engine Particle::_generator = std::default_random_engine(time(NULL));

Particle::Particle(double dt, double rad, Arr3d pos,Arr3d vel, CollisionGrid* gr)
{
	grid = gr;

	_x = pos.matrix();
	_r = rad;
	_v = vel.matrix();
	_dt = dt;
	first_step = true;
	use_brownian = true;
	particle_is_stuck = false;

	use_periodic_boundary= false;
	boundary = VecXd::Zero(6);
	boundary_leaps =Vec3d(0,0,0);
	boundary_span = Vec3d(0,0,0);

	traj = std::vector<Eigen::VectorXd>();
}

Particle::~Particle()
{

}

void Particle::update()
{
	if(	_x(0) < boundary(0) &&
		_x(0) > boundary(1) &&
		_x(1) < boundary(2) &&
		_x(1) < boundary(3) &&
		_x(2) < boundary(4) &&
		_x(2) < boundary(5) )
	{
		std::cerr << "Particl::update" << std::endl;
		std::cerr << "	Particle outside of boundary! Exiting" << std::endl;
		exit(1);
	}

	
	Vec3d new_pos = _x;
	if(!particle_is_stuck)
	{	
		double Diff_constant = 1;
		Vec3d brownian  = get_random_vector();
		Vec3d new_vel = std::sqrt(2*Diff_constant*_dt)*brownian;
		new_pos = _x + new_vel;
		
		int stuck_count = 1000;
		int coll_count = 0;
		collision coll =  get_earliest_collision(new_pos);
		while(coll.t != 0)
		{
			brownian = get_random_vector(); 
			new_vel = std::sqrt(2*Diff_constant*_dt)*brownian;
			new_pos = _x + new_vel;
			coll =  get_earliest_collision(new_pos);
			if(coll_count > stuck_count )
			{
				std::cerr << "Particle with radius "<< _r <<" got stuck" << std::endl;
				particle_is_stuck = true;
				break;
			}
			coll_count ++;
		}
	}
	
	_x = new_pos;
 
	// Periodic boundary
	if(use_periodic_boundary)
	{
		if(_x(0) < boundary(0))
		{
			_x(0) = _x(0)+ boundary_span(0);
			boundary_leaps(0) = boundary_leaps(0)-1;
		}else if(_x(0) > boundary(1)){
			_x(0) = _x(0)-boundary_span(0);
			boundary_leaps(0) = boundary_leaps(0)+1;
		}
		
		if(_x(1) < boundary(2))
		{
			_x(1) = _x(1)+boundary_span(1);
			boundary_leaps(1) = boundary_leaps(1)-1;
		}else if(_x(1) > boundary(3)){
			_x(1) = _x(1)-boundary_span(1);
			boundary_leaps(1) = boundary_leaps(1)+1;
		}
		
		if(_x(2) < boundary(4))
		{
			_x(2) = _x(2)+boundary_span(2);
			boundary_leaps(2) = boundary_leaps(2)-1;
		}else if(_x(2) > boundary(5)){
			_x(2) = _x(2)-boundary_span(2);
			boundary_leaps(2) = boundary_leaps(2)+1;
		}
	}

	Eigen::VectorXd log = Eigen::VectorXd::Zero(6);
	Eigen::Array3d skips = boundary_leaps.array() * boundary_span.array();
	log.segment(0,3) = _x+skips.matrix();
	log.segment(3,3) = _v;
	traj.push_back(log);
}

Arr3d Particle::get_position()
{
	return traj.back().segment(0,3);
}

Vec3d Particle::get_random_vector()
{
	std::normal_distribution<double> norm(0, 1);
	double x =norm(_generator);
	double y =norm(_generator);
	double z =norm(_generator);
	Vec3d dir = Vec3d(x,y,z);
	return dir;
}

Vec3d Particle::reflect_velocity(Vec3d v, Vec3d n)
{
	double len = v.transpose() * n;
	Vec3d v_normal_to_plane  = len * n;
	Vec3d v_paralell_to_plane  = v - v_normal_to_plane;
	Vec3d v_new = v_paralell_to_plane - v_normal_to_plane;

	return v_new;
}


void Particle::set_periodic_boundary(VecXd bound)
{
	use_periodic_boundary= true;
	boundary = bound;

	boundary_span(0) = bound(1)-bound(0);
	boundary_span(1) = bound(3)-bound(2);
	boundary_span(2) = bound(5)-bound(4);
}

Particle::collision Particle::get_earliest_collision(Vec3d particle_position)
{
	std::vector<cg_ptr > v;

	if( (grid != NULL) )
	{
		cg_ptr S = cg_ptr(new Sphere(particle_position,_r));

		std::vector<cg_ptr > mirrors;
		if(use_periodic_boundary)
		{
			mirrors = S->mirror(boundary);
		}
		mirrors.push_back(S);


		for(int i = 0; i<mirrors.size(); i++)
		{
			std::vector<cg_ptr > tmp = grid->get_collision_bodies(mirrors[i]);
			for(int j = 0; j<tmp.size(); j++)
			{
				v.push_back(tmp[j]);
			}
		}
	}else{
		v = test_coll_vec;
	}

	Sphere S = Sphere(particle_position, _r);
	collision ret = {};
	ret.n = Vec3d(0,0,0);

	for(int i = 0; i != v.size(); ++i)
	{	
		CollisionGeometry::coll_struct cs;
		std::shared_ptr<CollisionGeometry> c = v[i];
		
		if(c->intersects(&S, cs))
		{
			ret.t = 1;
			Vec3d collision_normal = cs.n;
			double penetration_depth = cs.p;

			// Aligning the normal to be away from coll geom
			Vec3d collision_geometry_center = c->get_center();
			Vec3d geom_to_sphere = particle_position-collision_geometry_center;
			if( (geom_to_sphere.transpose() * cs.n) < 0)
			{
				collision_normal = -cs.n;
			}

			ret.n = (ret.n + collision_normal).normalized();
		}
	}

	return ret;
}

void Particle::dump_info( debug_snapshot ds )
{
	std::ofstream particle_file;
	particle_file.open("../matlab/Distance/debug/particle_info", 
					std::fstream::out | std::fstream::trunc);
	if(particle_file.is_open()){
		particle_file << "particle: " << std::endl; 
		particle_file <<"\t" <<  ds.state.dt  << std::endl;
		particle_file <<"\t" <<  ds.radius  << std::endl;
		particle_file <<"\t" <<  ds.state.pos.transpose()  << std::endl;
		particle_file <<"\t" <<  ds.state.vel.transpose()  << std::endl;
			
		particle_file << "Collision_struct" << std::endl;
		particle_file << "\t" <<ds.cs.p << std::endl;
		particle_file << "\t" <<ds.cs.n.transpose() << std::endl;
		particle_file << "Derived values" << std::endl;
		particle_file << "\t"<< "Collision Normal" << std::endl;
		particle_file << "\t\t"<< ds.collision_normal.transpose() << std::endl;
		particle_file << "\t"<< "Contact Point" << std::endl;
		particle_file << "\t\t"<< ds.contact_point.transpose() << std::endl;
		particle_file << "\t"<< "Penetration Depth" << std::endl;
		particle_file << "\t\t"<< ds.penetration_depth << std::endl;
		particle_file << "\t"<< "Collision_time" << std::endl;
		particle_file << "\t\t"<< ds.collision_time << std::endl;

		particle_file << "Collision bodies" << std::endl;
		particle_file <<  "\t" <<ds.v.size()<< " "<< ds.i << std::endl;
		for(int j = 0; j<ds.v.size(); j++)
		{	
			cg_ptr c = ds.v[j];
			if(c->text_type().compare("Plane")==0)
			{
				particle_file << 0 << "\t";
			}else if(c->text_type().compare("Sphere")==0){
				particle_file << 1 << "\t";
			}else if(c->text_type().compare("Cylinder")==0){
				particle_file << 2 << "\t";
			}
			particle_file << c->get_span().transpose() << std::endl;
		}


		particle_file.close();
	}else{
		std::cerr << "Failed to open " << std::string("../matlab/Distance/debug/particle_info") << std::endl;
	}

	std::ofstream traj_file;
	traj_file.open("../matlab/Distance/debug/particle_traj_dump", 
					std::fstream::out | std::fstream::trunc);
	
	if(traj_file.is_open())
	{
		for(int i = 0; i<traj.size(); i++)
		{
			traj_file  << traj[i].transpose() << std::endl;
		}

		traj_file.close();
	}else{
		std::cerr << "Failed to open " << std::string("../matlab/Distance/debug/particle_traj_dump") << std::endl;
	}

}

std::ostream& operator<<(std::ostream& os, const Particle& p)
{
	for(auto it = p.traj.begin(); it != p.traj.end(); it++)
	{
		os << it->transpose() << std::endl;
	}
	return os;
}


void Particle::set_test_collision_vector(std::vector<cg_ptr> v )
{
	test_coll_vec = v;
}

