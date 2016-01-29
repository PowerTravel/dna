#include "Particle.hpp"

#include <iostream>
#include <algorithm>

std::default_random_engine Particle::_generator = std::default_random_engine(time(NULL));

Particle::Particle(double rad, Arr3d pos,Arr3d vel, CollisionGrid* gr)
{
	grid = gr;

	_x = pos.matrix();
	_r = rad;
	_v = vel.matrix();
	first_step = true;

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

void Particle::update(double dt)
{
	static int timestep = 0;

	particle_state old_state = {};
	old_state.dt = 0;
	old_state.pos = _x;
	old_state.vel = _v;

	// Brownian Impulse
	double max_len = 2;
	double min_len = 0;

	bool use_brownian = false;
	Vec3d brownian  = Vec3d::Zero();
	if(use_brownian)
	{
		brownian = get_random_vector(min_len,max_len); 
	}

	particle_state new_state = {};
	// Leapfrog Euler
	if(first_step)
	{
		new_state.vel = old_state.vel + (dt*0.5)*brownian;
		first_step = false;
	}else{
		new_state.vel = old_state.vel + dt*brownian;
	}
	new_state.pos = old_state.pos + dt*new_state.vel;
	new_state.dt = dt;
	new_state = handle_collisions(new_state);

	_x = new_state.pos;
	_v = new_state.vel;

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
	std::vector<cg_ptr > coll_geom_vec;
	if(test_coll_vec.size() == 0)
	{
		cg_ptr S = cg_ptr(new Sphere(state.pos,_r));
		coll_geom_vec = grid->get_collision_bodies(S);
		coll_geom_vec = remove_cylinders(coll_geom_vec);
	}else{
		coll_geom_vec = test_coll_vec;
	}

	collision coll =  get_earliest_collision(coll_geom_vec,state);
	

	if(coll.t != 0)
	{
		// Find collision state
		particle_state collision_state;
		collision_state.dt = state.dt + coll.t; 
		collision_state.pos = state.pos + coll.t * state.vel;
		double remaining_dt = -coll.t;

		// Reflect Velocity
		double len = state.vel.transpose() * coll.n;
		Vec3d v_normal_to_plane  = len * coll.n;
		Vec3d v_paralell_to_plane  = state.vel - v_normal_to_plane;
		collision_state.vel = v_paralell_to_plane - v_normal_to_plane;


		particle_state post_collision_state;
		post_collision_state.dt = state.dt;
		post_collision_state.vel = collision_state.vel;
		post_collision_state.pos = collision_state.pos +	 remaining_dt*post_collision_state.vel;
		state = post_collision_state;
	}

	return state;
}

Particle::collision Particle::get_earliest_collision(std::vector<cg_ptr > v, particle_state particle)
{
	Sphere S = Sphere(particle.pos, _r);
	collision ret = {};

	for(int i = 0; i != v.size(); ++i)
	{
		//Plane P = Plane(Vec3d(0,0,0), Vec3d(0,1,0));
		CollisionGeometry::coll_struct cs;
		std::shared_ptr<CollisionGeometry> c = v[i];
		double collision_time = 0;
		if(c->intersects(&S, cs))
		{
			Vec3d collision_normal = cs.n;

			// Aligning the normal to be away from coll geom
			Vec3d geom_to_sphere = particle.pos-c->get_center();
			if( (geom_to_sphere.transpose() * cs.n) < 0)
			{
				collision_normal = -cs.n;
			}

			Vec3d contact_point = particle.pos - _r * collision_normal;
			collision_time = c->line_intersection_point(
									contact_point, particle.vel);

			if(collision_time > 0 )
			{
				std::cerr << "Particle::get_earliest_collision: " <<std::endl;
				std::cerr << "	Error: collision_time evaluated to " <<collision_time<< "which is larger than 0 " <<std::endl;
				std::cerr << "	This should never happen since collision_time is how long time since a collision has happened. IE negative " <<std::endl;
				std::cerr << "	exiting" << std::endl;
				exit(1);
			}else if(collision_time < (-particle.dt)) {
				std::cerr << "Particle::get_earliest_collision: " <<std::endl;
				std::cerr << "	Error: collision_time evaluated to " <<collision_time<< "which is smaller than than dt="<< (-particle.dt ) <<std::endl;
				std::cerr << "	This should never happen since dt is the beginning of a timestep where a collision is assumed never to happen" <<std::endl;
				std::cerr << "	exiting"  <<std::endl;
				exit(1);

			}

			double tol =  0.0001;
			double diff = collision_time - ret.t;
			if( diff <  (-tol) )
			{
				ret.n = collision_normal;
				ret.t = collision_time;
			}else if(std::abs(diff) < tol)
			{
				ret.n = ret.n + collision_normal;

				ret.t = collision_time;
			}
		}
	}

	if(ret.t != 0)
	{

		ret.n = (ret.n).normalized();
		std::cout << ret.t << std::endl;
		std::cout << ret.n.transpose() << std::endl;
	}
	return ret;
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








