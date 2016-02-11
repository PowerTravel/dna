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
	use_periodic_boundary = false;
	particle_is_stuck = false;
	traj = std::vector<Eigen::VectorXd>();
}

Particle::~Particle()
{

}

Arr3d Particle::get_position()
{
	return _x;
}

Arr3d Particle::get_velocity()
{
	return _v;
}

void Particle::update()
{
	static int timestep = 0;
	if(!particle_is_stuck)
	{
		particle_state old_state = {};
		old_state.pos = _x;
		old_state.vel = _v;

		// Brownian Impulse
		double max_len = 2;
		double min_len = 0;

		Vec3d brownian  = Vec3d::Zero();
		if(use_brownian)
		{
			brownian = get_random_vector(min_len,max_len); 
		}

		particle_state new_state = {};
		new_state.dt = _dt;
		// Leapfrog Euler
		if(first_step)
		{
			new_state.vel = old_state.vel + (_dt*0.5)*brownian;
			first_step = false;
		}else{
			new_state.vel = old_state.vel + _dt*brownian;
		}
		new_state.pos = old_state.pos + _dt*new_state.vel;
		new_state = handle_collisions(new_state);

		if(new_state.dt != _dt)
		{
			std::cerr << "Particle::update" << std::endl;
			std::cerr << "	Particle is stuck" << std::endl;
			std::cerr << "	exiting" << std::endl;
			exit(0);
		}
	
		_x = new_state.pos;
		_v = new_state.vel;

		// Periodic boundary
		// This should be done in collision grid.
		Vec3d max_dim = Vec3d(7.5,7.5,7.5);
		if(use_periodic_boundary)
		{
			if(_x(0) > max_dim(0))
			{
				_x(0) = _x(0)-2*max_dim(0);
			}else if(_x(0) < -max_dim(0)){
				_x(0) = _x(0)+2*max_dim(0);
			}
			
			if(_x(1) > max_dim(1))
			{
				_x(1) = _x(1)-2*max_dim(1);
			}else if(_x(1) < -max_dim(1)){
				_x(1) = _x(1)+2*max_dim(1);
				
			}
			
			if(_x(2) > max_dim(2))
			{
				_x(2) = _x(2)-2*max_dim(2);
			}else if(_x(2) < -max_dim(2)){
				_x(2) = _x(2)+2*max_dim(2);
			}
		}
	}

	Eigen::VectorXd log = Eigen::VectorXd::Zero(6);
	log.segment(0,3) = _x;
	log.segment(3,3) = _v;
	traj.push_back(log);
	timestep++;
}


Vec3d Particle::get_random_vector(double min_len, double max_len)
{
	std::uniform_real_distribution<double> scale(min_len, max_len);
	double s= scale(_generator);
	std::uniform_real_distribution<double> direction(-1, 1);
	double x = direction(_generator);
	double y = direction(_generator);
	double z = direction(_generator);
	Vec3d dir = Vec3d(x,y,z);
	dir.normalize();
	return dir*s;
}

std::vector<cg_ptr > Particle::remove_cylinders(std::vector<cg_ptr > vec)
{
	std::vector<cg_ptr> ret_vec;
	for(int i=0; i<vec.size(); ++i )
	{

		if(vec[i]->text_type().compare("Cylinder")==0)
		{
			i++;
		}else{
			ret_vec.push_back(vec[i]);
		}
	}
	return ret_vec;
}

Particle::particle_state Particle::handle_collisions(particle_state state)
{

	collision coll =  get_earliest_collision(state);

	double time_left = _dt;
	double last_collision_time = 0;
	particle_state collision_state = {};
	particle_state post_collision_state = state;
	while(coll.t != 0){
		// Find collision state
		collision_state.dt = post_collision_state.dt + coll.t; 
		collision_state.pos = post_collision_state.pos + 
								coll.t * post_collision_state.vel;
		if( (last_collision_time  == 0) && (collision_state.dt <= _dt) )
		{
			// The case where we are dealing with the first collision we
			// require that the collision happened somewhere inside dt
			last_collision_time = collision_state.dt;

		//	std::cerr << "FIRST_COLLISION! " << collision_state.dt << std::endl;
		//	std::cerr << "	collision_time: " << collision_state.dt << std::endl;
		//	std::cerr << "	previous collision_time: " << last_collision_time << std::endl;
		}else if( (last_collision_time <= collision_state.dt+0.00000000000) && 
						(collision_state.dt < _dt)  )
		{
			// The case where we are dealing with consecutive collisions inside
			// the timestep we require that the collision happened in the time
			// between the last treated collision and the end of the timestep
			last_collision_time = collision_state.dt;

		//	std::cerr << "NEXT_COLLISION! " << collision_state.dt << std::endl;
		//	std::cerr << "	collision_time: " << collision_state.dt << std::endl;
		//	std::cerr << "	previous collision_time: " << last_collision_time << std::endl;
		//	std::cerr << "	Post_collision_State: " <<  std::endl;
		//	std::cerr << "	dt = "<< post_collision_state.dt << std::endl;
		//	std::cerr << "	pos: "<< post_collision_state.pos.transpose() << std::endl;
		//	std::cerr << "	vel: "<< post_collision_state.vel.transpose() << std::endl;
		}else{
			// if not we have some sort of error and should investigate what is 
			// going on.

			std::cerr << "Particle::handle_collisions" << std::endl;
			std::cerr << "	The collision happened at a weird time" << std::endl;
			std::cerr << "	collision_time: " << collision_state.dt << std::endl;
			std::cerr << "	previous collision_time: " << last_collision_time << std::endl;
			std::cerr << "	previos collision_time - collision_time " << last_collision_time-collision_state.dt << std::endl;
			
			std::cerr << "	Setting Particle to stuck at tiemstep " << timestep << "or at time: "<< ((double) timestep )* _dt << std::endl;
			particle_is_stuck=true;
			break;
		//	exit(1);

		}
		// Reflect Velocity
		double len = post_collision_state.vel.transpose() * coll.n;
		Vec3d v_normal_to_plane  = len * coll.n;
		Vec3d v_paralell_to_plane  = post_collision_state.vel - v_normal_to_plane;
		collision_state.vel = v_paralell_to_plane - v_normal_to_plane;

		double remaining_dt = -coll.t;
		post_collision_state.dt = _dt;
		post_collision_state.vel = collision_state.vel;
		post_collision_state.pos = collision_state.pos + remaining_dt*post_collision_state.vel;

		coll =  get_earliest_collision(post_collision_state);
	}

	return post_collision_state;
}

Particle::collision Particle::get_earliest_collision(particle_state particle)
{
	std::vector<cg_ptr > v;
	if( (grid != NULL) )
	{
		cg_ptr S = cg_ptr(new Sphere(particle.pos,_r));
		//grid->active = true;
		v = grid->get_collision_bodies(S);
		//grid->active = false;
	//	v = remove_cylinders(v);
	}else{
		v = test_coll_vec;
	}

	Sphere S = Sphere(particle.pos, _r);
	collision ret = {};

	for(int i = 0; i != v.size(); ++i)
	{	
		CollisionGeometry::coll_struct cs;
		std::shared_ptr<CollisionGeometry> c = v[i];
		double collision_time = 0;

		if(c->intersects(&S, cs))
		{
			Vec3d collision_normal = cs.n;
			double penetration_depth = cs.p;

			// Aligning the normal to be away from coll geom
			Vec3d collision_geometry_center = c->get_center();
			Vec3d geom_to_sphere = particle.pos-collision_geometry_center;
			if( (geom_to_sphere.transpose() * cs.n) < 0)
			{
				collision_normal = -cs.n;
			}
			Vec3d contact_point = particle.pos - _r * collision_normal;	
			collision_time = c->line_intersection_point(
									contact_point, particle.vel);
			double tol =  0.0000001;
			if(collision_time > tol )
			{
				std::cerr << "Particle::get_earliest_collision: " <<std::endl;
				std::cerr << "	Error: collision_time evaluated to " <<collision_time<< " which is larger than 0 " <<std::endl;
				std::cerr << "	This should never happen since collision_time is how long time since a collision has happened. IE negative " <<std::endl;
				std::cerr << "	exiting" << std::endl;
				debug_snapshot dbss = 
				{	
					i, v, cs,collision_normal , contact_point, penetration_depth, collision_time, 
					particle, _r
				};

				cg_ptr spp = cg_ptr(new Sphere(particle.pos,_r));
				// TODO implement this
				//grid->print_intersecting_box_corners(spp);
				dump_info( dbss  );
				exit(1);
			}else if( (collision_time+tol) < (-particle.dt)) {

				c->intersects(&S, cs);
				std::cerr << "Particle::get_earliest_collision: " <<std::endl;
				std::cerr << "	Error: collision_time evaluated to " <<collision_time<< "which is smaller than than dt="<< (-particle.dt ) <<std::endl;
				std::cerr << "	This should never happen since dt is the beginning of a timestep where a collision is assumed never to happen" <<std::endl;
				std::cerr << "	exiting"  <<std::endl;
				std::cerr << v.size() << std::endl;
				debug_snapshot dbss = 
				{	
					i, v, cs,collision_normal , contact_point, penetration_depth, collision_time, 
					particle, _r
				};


				dump_info( dbss  );

				exit(1);
			}
		

			double diff = collision_time - ret.t;
			if( diff <  (-tol) )
			{
//				std::cerr << "Overriding" << std::endl;
//				std::cerr << collision_time << std::endl;
//				std::cerr << ret.t << std::endl;

				ret.n = collision_normal;
				ret.t = collision_time;
			}else if(std::abs(diff) < tol){
//				std::cerr << "Adding" << std::endl;
//				std::cerr << collision_time << std::endl;
//				std::cerr << ret.t << std::endl;


				ret.n = ret.n + collision_normal;
				ret.n = (ret.n).normalized();
			}else{
				// Do nothing
//				std::cerr << "Ignoring" << std::endl;
//				std::cerr << collision_time << std::endl;
//				std::cerr << ret.t << std::endl;
			
			}
//			std::cout<< "N: " << ret.n.transpose() << std::endl;
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

