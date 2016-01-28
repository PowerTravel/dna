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

Particle::particle_state Particle::get_collision_state(intersections I, particle_state state)
{

	Vec3d contact_point = state.pos - _r * I.cs.n;

	double rewind_time_to_collision = I.geom->line_intersection_point(
					contact_point, state.vel);
	// rewind_time_to_collision should NEVER be zero and should ALWAYS be negative; This print will catch those situations if they occur
	if(rewind_time_to_collision >= 0)
	{
		std::cerr << "Particle::do_one_collision"<< std::endl;
		std::cerr << 	"	dt " << state.dt << std::endl;
		std::cerr << 	"	rewind_time " << rewind_time_to_collision << std::endl;
		std::cerr <<	"	Collision Normal " << I.cs.n.transpose() << std::endl;
		std::cerr <<	"	Contact Point" << contact_point.transpose() << std::endl;
		std::cerr <<	"	Particle Pos " << state.pos.transpose() << std::endl;
		std::cerr <<	"	Particle Vel  " << state.vel.transpose() << std::endl;
		std::cerr <<	"	Particle rad  " << _r << std::endl;
		std::cerr <<	"	Exiting  " << std::endl;
		exit(0);
	}

	particle_state collision_state;
	collision_state.dt = state.dt + rewind_time_to_collision; 
	collision_state.pos = state.pos +  rewind_time_to_collision * state.vel;

	// Reflect Velocity
	Vec3d collision_normal;
	collision_normal = I.cs.n;

	double len = state.vel.transpose() * collision_normal;
	Vec3d v_normal_to_plane  = len * collision_normal;
	Vec3d v_paralell  = state.vel - v_normal_to_plane;
	collision_state.vel = v_paralell - v_normal_to_plane;

	// TODO: handle the edgecase of truly simultaneous collisions by setting 
	//		 collision_normal to intersections.effective_n if 
	// 		 intersections.composite is true


	// the state of particle the instance after collision
	return collision_state;
}

Particle::particle_state Particle::do_one_collision(intersections I, particle_state state)
{
	particle_state collision_state = get_collision_state(I, state);

	double remaining_dt = state.dt-collision_state.dt;

	particle_state post_collision_state;
	post_collision_state.dt = state.dt;
	post_collision_state.vel = collision_state.vel;
	post_collision_state.pos = collision_state.pos + remaining_dt*post_collision_state.vel;

	return post_collision_state;
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

Particle::particle_state Particle::do_collisions(particle_state state)
{

	std::vector<cg_ptr > coll_geom_vec = build_plane_test();
	//std::vector<cg_ptr > coll_geom_vec = build_cylinder_test_A();
	
	//cg_ptr S = cg_ptr(new Sphere(state.pos,_r));
	//std::vector<cg_ptr > coll_geom_vec = grid->get_collision_bodies(S);
	//coll_geom_vec = remove_cylinders(coll_geom_vec);

	// Gets a vector of all actual collisions, sorted after penetration depth
	// and with normals aligned
	std::vector< intersections > collisions = 
							get_coll_vec(coll_geom_vec, state );


	// Treat the top most collisions one by one
	// Do more than 100 Collisions withut advancing timestep and we call the particle stuck.
	int max_collision =100;
	int coll_idx = 0;
#if 0

	while( (collisions.size() !=0) && (coll_idx < max_collision) ){

		if(collisions.size() > 1)
		{
			//	std::cerr << "Printed Froom Particle::do_collisions(), More than one collision in the same timestep detected. Should work fine but not thoroughly tested." << std::endl;

			for(int i = 0; i < collisions.size(); ++i)
			{
				std::cout << collisions[i].cs.p << ", ";
			}
			std::cout<<std::endl;

			int simultaneous_collisions = check_for_simultaneous_collisions(collisions, state);

			if(simultaneous_collisions != 0)
			{
				std::cerr << "Particle::do_collisions(): "<< simultaneous_collisions << " those are simultaneous collisions" << std::endl;
			}
		}

		intersections intersect = collisions[0];

		// TODO: Handle case where two penetration depths are equal
		state = do_one_collision(intersect, state);

	collisions = get_coll_vec(coll_geom_vec, state);
		coll_idx ++;
	}
	if(coll_idx >= max_collision)
	{
		std::cerr << "Particle::do_collisions(): Particle Stuck. Exiting" << std::endl; 
		exit(0);

	}
#endif 
	if( (collisions.size() !=0) ){
		state = do_one_collision(collisions[0], state);
	}

	return state;
}


#if 0
int Particle::check_for_simultaneous_collisions(std::vector< intersections > v, particle_state state)
{
		

	int simultaneous_collisions=0;
	double tol = 0.0001;
	if(v.size() != 0)
	{
		intersections I = v[0];

		Vec3d contact_point = state.pos - _r * I.cs.n;	
		double dt = I.geom->line_intersection_point(contact_point, state.vel);


		if(dt == 0)
		{
			std::cerr << "Particle::check_for_simultaneous_collisions::Collision Normal "<< I.cs.n.transpose() << std::endl;
			std::cerr << "Particle::check_for_simultaneous_collisions::Contact Point "<< contact_point.transpose() << std::endl;
		}

		for(int i = 1; i<v.size(); i++)
		{
			intersections I_compare = v[i];

			Vec3d contact_point_compare = state.pos - _r * I_compare.cs.n;	
			double dt_compare = I_compare.geom->line_intersection_point(contact_point, state.vel);

			if( std::abs(dt_compare-dt) < tol )
			{
				simultaneous_collisions++;

				// Handle by adding together all collision normals into an
				// "effective normal" and use that when evaluating collisions
				/*
				intersections tmp_i = collisions.front();
				tmp_i = align_normal(tmp_i, vp);
				tmp_i.effective_n = tmp_i.cs.n;
				tmp_i.effective_n = (tmp_i.effective_n + it->cs.n).normalized();
				
				tmp_i.composite = true;

				add effective_n to I and return it somehow 
				*/
			}
		}

	}
	return simultaneous_collisions;
}
#endif

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
	new_state = do_collisions(new_state);

	Vec3d diff = _x - new_state.pos;
	if(diff.norm()>1)
	{
		std::cerr << "Particle::Update" << std::endl;
		std::cerr << "	x: "<< _x.transpose() <<std::endl;
		std::cerr << "	n: "<< new_state.pos.transpose() <<std::endl;
		std::cerr << "	t: " << timestep << std::endl; 
	}

	_x = new_state.pos;
	_v = new_state.vel;

	Eigen::VectorXd log = Eigen::VectorXd::Zero(9);
	log.segment(0,3) = _x;
	log.segment(3,3) = _v;
	traj.push_back(log);
	timestep++;
}


std::vector<Particle::intersections> Particle::get_coll_vec(std::vector<cg_ptr > v, particle_state particle)
{
	Sphere S = Sphere(particle.pos, _r);
	std::vector<intersections> ret;
	for(int i = 0; i != v.size(); ++i)
	{
		//Plane P = Plane(Vec3d(0,0,0), Vec3d(0,1,0));
		CollisionGeometry::coll_struct cs;
		std::shared_ptr<CollisionGeometry> c = v[i];

		if(c->intersects(&S, cs))
		{
			intersections is;
			is.geom = c;

			// Aligning the normal to be away from coll geom
			Vec3d geom_to_sphere = particle.pos-c->get_center();
			if( (geom_to_sphere.transpose() * cs.n) < 0)
			{
				cs.n = -cs.n;
			}
			is.cs = cs;
			ret.push_back(is);
		}
	}

	if(ret.size() > 1)
	{
		std::sort(ret.begin(), ret.end(), sort_after_penetration_depth);
	}
	

	return ret;
}


bool Particle::sort_after_penetration_depth(const Particle::intersections& first, const intersections& second)
{
	double p1 = first.cs.p;
	double p2 = second.cs.p;
	return (p1>p2);
}

std::ostream& operator<<(std::ostream& os, const Particle& p)
{
	for(auto it = p.traj.begin(); it != p.traj.end(); it++)
	{
		os << it->transpose() << std::endl;
	}
	return os;
}

std::vector< cg_ptr > Particle::build_plane_test()
{
	double box_r = 5;
	std::vector< cg_ptr > coll_geom_vec;
	coll_geom_vec.push_back( std::shared_ptr<CollisionGeometry>( 
				new Plane(Vec3d(0,box_r,0), Vec3d(0,1,0)) ));
	coll_geom_vec.push_back( std::shared_ptr<CollisionGeometry>( 
				new Plane(Vec3d(0,-box_r,0), Vec3d(0,-1,0)) ));
			

	// Left and right wall
	coll_geom_vec.push_back( std::shared_ptr<CollisionGeometry>( 
				new Plane(Vec3d(box_r,0,0), Vec3d(1,0,0)) ));
	coll_geom_vec.push_back( std::shared_ptr<CollisionGeometry>( 
				new Plane(Vec3d(-box_r,0,0), Vec3d(-1,0,0)) ));

	// Front back wall
	coll_geom_vec.push_back( std::shared_ptr<CollisionGeometry>( 
				new Plane(Vec3d(0,0,box_r), Vec3d(0,0,1)) ));
	coll_geom_vec.push_back( std::shared_ptr<CollisionGeometry>( 
				new Plane(Vec3d(0,0, -box_r), Vec3d(0,0,-1)) ));

	return coll_geom_vec;
}
















