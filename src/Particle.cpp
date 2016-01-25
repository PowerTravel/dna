#include "Particle.hpp"

#include <iostream>
#include <algorithm>

std::default_random_engine Particle::_generator = std::default_random_engine(time(NULL));

Particle::Particle(double rad, Eigen::Array3d pos,Eigen::Array3d vel, CollisionGrid* gr)
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

Eigen::Array3d Particle::get_position()
{
	return _x;
}

Eigen::Array3d Particle::get_velocity()
{
	return _v;
}

Eigen::Vector3d Particle::get_random_vector(double min_len, double max_len)
{
	std::uniform_real_distribution<double> scale(min_len, max_len);
	double s= scale(_generator);
	std::uniform_real_distribution<double> direction(-1, 1);
	double x = direction(_generator);
	double y = direction(_generator);
	double z = direction(_generator);
	Eigen::Vector3d dir = Eigen::Vector3d(x,y,z);
	dir.normalize();
	return dir*s;
}

Particle::particle_state Particle::do_one_collision(intersections I, particle_state state)
{
	Eigen::Vector3d contact_point = state.pos - _r * I.cs.n;

	particle_state collision_state;
	collision_state.dt = I.geom->line_intersection_point(
					contact_point, state.vel);

	collision_state.pos = state.pos +  collision_state.dt * state.vel;
	

	// Reflect Velocity
	Eigen::Vector3d collision_normal = I.cs.n;
	double len = state.vel.transpose() * collision_normal;
	Eigen::Vector3d v_normal_to_plane  = len * collision_normal;
	Eigen::Vector3d v_paralell  = state.vel - v_normal_to_plane;
	collision_state.vel = v_paralell - v_normal_to_plane;

	particle_state final_state;
	state.vel = collision_state.vel;
	state.pos = collision_state.pos + (state.dt-collision_state.dt)*state.vel;

	return state;

}

Eigen::VectorXd Particle::do_one_collision(double dt_tot, Eigen::VectorXd X, Eigen::Vector3d a , intersections is )
{
	Eigen::Vector3d x,v;
	double dt = dt_tot - X(0);
	x = X.segment(1,3);
	v = X.segment(4,3);

	v = v + dt * a;

	CollisionGeometry::coll_struct cs = is.cs;

	// Den punkt på sfären som först träffar collisionsplanet
	Eigen::Vector3d contact_point = x - _r * cs.n;
	double dt_t = is.geom->line_intersection_point(contact_point, v);

	// Center på sfären när den kolliderar
	Eigen::Vector3d x_c = x +  dt_t * v;
	// Hastigheten på sfären när den kolliderar
	Eigen::Vector3d v_c = v +  dt_t * a;

	// Reflektera hastigheten runt kollisionsplanet
	Eigen::Vector3d n = cs.n;
	if(is.effective_n != Eigen::Vector3d::Zero())
	{
		n = is.effective_n;
	}

	double len = v_c.transpose() * n;
	Eigen::Vector3d v_norm  = len * n;
	Eigen::Vector3d v_paralell  = v_c - v_norm;
	v_c = v_paralell - v_norm;

	// positionen efter kolissionen
	Eigen::Vector3d vp = v_c +  (dt-dt_t) * a;
	Eigen::Vector3d xp = x_c +  (dt-dt_t) * vp;

	Eigen::VectorXd ret = Eigen::VectorXd::Zero(14);

	ret(0) = (dt_t);
	ret.segment(1,3)  = x_c;
	ret.segment(4,3)  = v_c;
	
	ret(7) = dt;
	ret.segment(8,3)  = xp;
	ret.segment(11,3)  = vp;
	
	return ret;
}

Particle::particle_state Particle::do_collisions(particle_state state)
{

	std::vector<cg_ptr > coll_geom_vec = build_plane_test();
	//std::vector<cg_ptr > coll_geom_vec = grid->get_collision_bodies(S);

	// Gets a vector of all actual collisions, sorted after penetration depth
	// and with normals aligned
	std::vector< intersections > collisions = 
							get_coll_vec(coll_geom_vec, state );

	if(collisions.size() > 1)
	{
		std::cerr << "bajs" << std::endl;
	}

	// Treat the top most collisions one by one
	while(collisions.size() > 0 ){

		intersections intersect = collisions[0];

		// TODO: Handle case where two penetration depths are equal
		state = do_one_collision(intersect, state);

		collisions = get_coll_vec(coll_geom_vec, state);
	}
	
	return state;
}


//	Sphere S = *( (Sphere*) sps.get() );

	// NEXT UP: FOR EACH CALL TO UPDATE(), PRINT OUT TO A FILE THE COLLISIONGOMETRIES IN COLL_GEOM_VEC SO WE CAN TRACK THE SPHERE AND THE COLLISIONGEOMETRIES IT TESTSINTERSECTION AGAINST AS WE MAKE IT MOVE
/*
	for(auto it = coll_geom_vec.begin(); it != coll_geom_vec.end(); it++)
	{
		std::shared_ptr< CollisionGeometry> cgptr = *it;
		if(cgptr->text_type().compare("Sphere") == 0)
		{
			std::cout << "1 ";	
		}else if(cgptr->text_type().compare("Cylinder") == 0){
			std::cout << "2 ";	
		}
		std::cout << (*it)->get_id() <<" ";
		std::cout << (*it)->get_span().transpose() << std::endl;
	}
*/

	// Remove all cylidnders for debugging purposes
/*
	std::vector<cg_ptr> ctmp;
	for(auto its = coll_geom_vec.begin(); its != coll_geom_vec.end(); its++)
	{
		if((*its)->text_type().compare("Sphere")==0)
		{
			ctmp.push_back(*its);
		}
	}
	coll_geom_vec = ctmp;
*/

/*
	collisions = get_coll_list(coll_geom_vec, S );

	while(collisions.size() > 0 ){
		// Move the collision with highest penetration-depth to the top to be
		// resolved first.
		// The assumption being that that was the first collision.
		
		collisions.sort(sort_after_penetration_depth);

		auto it = collisions.begin();

		intersections s = align_normal(*it, vp);
		collisions.pop_front();
		collisions.push_front(s);

		Eigen::Vector3d contact_point_1 = xp - _r * it->cs.n;
		double dt_t1 = it->geom->line_intersection_point(contact_point_1, vp);
		it++;
		while( it != collisions.end() )
		{
			Eigen::Vector3d contact_point_2 = xp - _r * it->cs.n;
			double dt_t2 = it->geom->line_intersection_point(contact_point_2, vp);
			if( std::abs(dt_t2 - dt_t1) < 0.0000000001 )
			{
				intersections tmp_i = collisions.front();
				tmp_i = align_normal(tmp_i, vp);
				tmp_i.effective_n = tmp_i.cs.n;

				tmp_i.effective_n = (tmp_i.effective_n + it->cs.n).normalized();

				collisions.pop_front();
				collisions.push_front(tmp_i);

				std::cerr << "Simultaneous collision with many collisionbodies, may contain errors. This is printed mainly because I don't bother making this work perfectly untill I know it will happen in real simulations. So if I see this later when diffusing particles in a fractal. Check it out in particle " << std::endl;
				std::cerr << "I just realized that this may indeed happen if we have a knot in the chain. Then two identical collision-geometries will be on the exact same place. This is different from the particle colliding with two different geometries at the same time and should be able to be handled as a special case. But lets see how it works in practice" << std::endl;
			}
			it++;
		}
		// Handle the first collision, update the position of the particle and do this all over again
		// Flip the normal of the collision - geometry such that v dot n < 0
		intersections c = align_normal(collisions.front(), _v);

		Eigen::VectorXd tmp = do_one_collision(dt, X,a, c);
		
		if( tmp(7) != tmp(7))
		{
			std::cerr << tmp.transpose() << std::endl;
			exit(1);
		}

		X = tmp.segment(0,7);
		X_P = tmp.segment(7,7);
	
		S = Sphere(X_P.segment(1,3), _r);
		collisions = get_coll_list(coll_geom_vec, S);
	}
	
	return X_P;
}
*/


void Particle::update(double dt)
{

	particle_state old_state = {};
	old_state.dt = 0;
	old_state.pos = _x;
	old_state.vel = _v;



	// Brownian Impulse
	double max_len = 2;
	double min_len = 0;
	Eigen::Vector3d brownian = get_random_vector(min_len,max_len); 


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

	// Checks for nan
	if( new_state.pos(0) != new_state.pos(0))
	{
		std::cerr << new_state.pos.transpose() << std::endl;
		exit(1);
	}

	_x = new_state.pos;
	_v = new_state.vel;

	Eigen::VectorXd log = Eigen::VectorXd::Zero(9);
	log.segment(0,3) = _x;
	log.segment(3,3) = _v;
	traj.push_back(log);
}

Particle::intersections Particle::align_normal(intersections is, Eigen::Vector3d v)
{
	CollisionGeometry::coll_struct cs = is.cs;

	if( v.transpose() * cs.n > 0)
	{
		cs.n = -cs.n;
	}
	is.cs = cs;
	return is;
}
std::vector<Particle::intersections> Particle::get_coll_vec(std::vector<cg_ptr > v, particle_state particle)
{
	Sphere S = Sphere(particle.pos, _r);
	std::vector<intersections> ret;
	for(int i = 0; i != v.size(); ++i)
	{
		//Plane P = Plane(Eigen::Vector3d(0,0,0), Eigen::Vector3d(0,1,0));
		CollisionGeometry::coll_struct cs;
		std::shared_ptr<CollisionGeometry> c = v[i];

		if(c->intersects(&S, cs))
		{
			intersections is;
			is.geom = c;

			// Aligning the normal to be opposite the velocity
			if( (particle.vel.transpose() * cs.n) > 0)
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

Eigen::Vector3d Particle::get_energy(Eigen::Vector3d g)
{
	Eigen::Vector3d E;
	// Ek
	double a = _v.transpose()*_v;
	E(0) = a/ 2.0;
	// Ep
	//E(1) = g.transpose() * _x;
	E(1) =- g(1)* _x(1);
	// Etot
	E(2) = E(0) + E(1);
	//std::cout << E.transpose() << std::endl;
	return E;
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
				new Plane(Eigen::Vector3d(0,box_r,0), Eigen::Vector3d(0,-1,0)) ));
	coll_geom_vec.push_back( std::shared_ptr<CollisionGeometry>( 
				new Plane(Eigen::Vector3d(0,-box_r,0), Eigen::Vector3d(0,1,0)) ));
			

	// Left and right wall
	coll_geom_vec.push_back( std::shared_ptr<CollisionGeometry>( 
				new Plane(Eigen::Vector3d(box_r,0,0), Eigen::Vector3d(-1,0,0)) ));
	coll_geom_vec.push_back( std::shared_ptr<CollisionGeometry>( 
				new Plane(Eigen::Vector3d(-box_r,0,0), Eigen::Vector3d(1,0,0)) ));

	// Front back wall
	coll_geom_vec.push_back( std::shared_ptr<CollisionGeometry>( 
				new Plane(Eigen::Vector3d(0,0,box_r), Eigen::Vector3d(0,0,-1)) ));
	coll_geom_vec.push_back( std::shared_ptr<CollisionGeometry>( 
				new Plane(Eigen::Vector3d(0,0, -box_r), Eigen::Vector3d(0,0,1)) ));

	return coll_geom_vec;
}



std::vector< cg_ptr > Particle::build_sphere_and_plane()
{
	double box_r = 5;
	std::vector< cg_ptr > coll_geom_vec = 
		std::vector< cg_ptr >();
	return coll_geom_vec;
}
		// Floor and roof
		/*
	coll_geom_vec.push_back( std::shared_ptr<CollisionGeometry>( 
				new Plane(Eigen::Vector3d(0,box_r,0), Eigen::Vector3d(0,-1,0)) ));
	coll_geom_vec.push_back( std::shared_ptr<CollisionGeometry>( 
				new Plane(Eigen::Vector3d(0,-box_r,0), Eigen::Vector3d(0,1,0)) ));
			

		// Left and right wall
	coll_geom_vec.push_back( std::shared_ptr<CollisionGeometry>( 
				new Plane(Eigen::Vector3d(box_r,0,0), Eigen::Vector3d(-1,0,0)) ));
	coll_geom_vec.push_back( std::shared_ptr<CollisionGeometry>( 
				new Plane(Eigen::Vector3d(-box_r,0,0), Eigen::Vector3d(1,0,0)) ));

	// Front back wall
	coll_geom_vec.push_back( std::shared_ptr<CollisionGeometry>( 
				new Plane(Eigen::Vector3d(0,0,box_r), Eigen::Vector3d(0,0,-1)) ));
	coll_geom_vec.push_back( std::shared_ptr<CollisionGeometry>( 
				new Plane(Eigen::Vector3d(0,0, -box_r), Eigen::Vector3d(0,0,1)) ));
	
	*/
//	coll_geom_vec.push_back( std::shared_ptr<CollisionGeometry>( 
//				new Plane(Eigen::Vector3d(0,-3,0), Eigen::Vector3d(-1,1,0)) ));
//	coll_geom_vec.push_back( std::shared_ptr<CollisionGeometry>( 
//				new Plane(Eigen::Vector3d(0, -3,0), Eigen::Vector3d(1,1,0)) ));

/*	
	coll_geom_vec.push_back( std::shared_ptr<CollisionGeometry>( 
				new Cylinder(3.0, Eigen::Vector3d(-0.5,-4,2), Eigen::Vector3d(-0.5,-3,-2)) ));
	coll_geom_vec.push_back( std::shared_ptr<CollisionGeometry>( 
				new Sphere(Eigen::Vector3d(-0.5,-4, 2), 3.0) ));

	coll_geom_vec.push_back( std::shared_ptr<CollisionGeometry>( 
				new Sphere(Eigen::Vector3d(-0.5,-3,-2), 3.0) ));
*/
	//coll_geom_vec.push_back( std::shared_ptr<CollisionGeometry>( 
	//			new Cylinder(Eigen::Vector3d(0, -3,0), Eigen::Vector3d(1,1,0)) ));
	
//	coll_geom_vec.push_back( std::shared_ptr<CollisionGeometry>( 
//				new Sphere(Eigen::Vector3d(5,-22,0), 20.0) ));
//	coll_geom_vec.push_back( std::shared_ptr<CollisionGeometry>( 
//				new Sphere(Eigen::Vector3d(-5,-22,0), 20.0) ));
//	return coll_geom_vec;
//}




















