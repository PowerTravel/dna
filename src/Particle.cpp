#include "Particle.hpp"

#include <iostream>
Particle::Particle(double rad, Eigen::Array3d pos,Eigen::Array3d vel, CollisionGrid* gr)
{
	grid = gr;

	_x = pos.matrix();
	_r = rad;
	_v = vel.matrix();
	first_step_taken = false;

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

void Particle::update(double dt, Eigen::Array3d a)
{
	//Static Collision Geometries
	std::vector<std::shared_ptr< CollisionGeometry> > coll_geom_vec = 
		build_sphere_and_plane();
	

	// Gonna implement som basic newtonian bouncing just to se that it works
	// Using leapfrog
	double h = dt;

	if(!first_step_taken)
	{
		first_step_taken = true;
		h = dt/2;
	}

	Eigen::Vector3d vp = _v + h*a.matrix();
	Eigen::Vector3d xp = _x + dt*vp;
	//std::cout << x.transpose() << std::endl;
	// X is the position of the sphere somewhere in the timestep. At this point
	// in the program it is in the very beginning.
	Eigen::VectorXd X = Eigen::VectorXd::Zero(7);
	X(0) = 0;
	X.segment(1,3) = _x; 
	X.segment(4,3) = _v; 

	// X_P is always the position of the sphere at the end of the timestep
	Eigen::VectorXd X_P = Eigen::VectorXd::Zero(7);
	X_P(0) = dt;
	X_P.segment(1,3) = xp;
	X_P.segment(4,3) = vp; 
	int i =0;
	Sphere S = Sphere(X_P.segment(1,3), _r);
	collisions = get_coll_list(coll_geom_vec, S);
	
	while(collisions.size() > 0){
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
			}
			it++;
		}
		// Handle the first collision, update the position of the particle and do this all over again
		// Flip the normal of the collision - geometry such that v dot n < 0
		intersections c = align_normal(collisions.front(), _v);

		Eigen::VectorXd tmp = do_one_collision(dt, X,a, c);
		X = tmp.segment(0,7);
		X_P = tmp.segment(7,7);
	
		S = Sphere(X_P.segment(1,3), _r);
		collisions = get_coll_list(coll_geom_vec, S);
	}

	_x = X_P.segment(1,3);
	_v = X_P.segment(4,3);
	_E = get_energy(a.matrix());

	Eigen::VectorXd log = Eigen::VectorXd::Zero(9);
	log.segment(0,3) = _x;
	log.segment(3,3) = _v;
	log.segment(6,3) = _E;

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
std::list<Particle::intersections> Particle::get_coll_list(std::vector<std::shared_ptr< CollisionGeometry> > v, Sphere s)
{
	std::list<intersections> ret;
	for(auto cg = v.begin(); cg != v.end(); cg++)
	{
		//Plane P = Plane(Eigen::Vector3d(0,0,0), Eigen::Vector3d(0,1,0));
		CollisionGeometry::coll_struct cs;
		std::shared_ptr<CollisionGeometry> c = *cg;
		if(c->intersects(&s, cs))
		{
			intersections is;
			is.geom = c;
			is.cs = cs;
			ret.push_back(is);
		}
	}
	return ret;
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

std::vector< std::shared_ptr<CollisionGeometry> > Particle::build_sphere_and_plane()
{
	double box_r = 5;
	std::vector<std::shared_ptr< CollisionGeometry> > coll_geom_vec = 
		std::vector<std::shared_ptr<CollisionGeometry> >();

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
	
	coll_geom_vec.push_back( std::shared_ptr<CollisionGeometry>( 
				new Cylinder(3.0, Eigen::Vector3d(-0.5,-4,2), Eigen::Vector3d(-0.5,-3,-2)) ));
	coll_geom_vec.push_back( std::shared_ptr<CollisionGeometry>( 
				new Sphere(Eigen::Vector3d(-0.5,-4, 2), 3.0) ));

	coll_geom_vec.push_back( std::shared_ptr<CollisionGeometry>( 
				new Sphere(Eigen::Vector3d(-0.5,-3,-2), 3.0) ));

	//coll_geom_vec.push_back( std::shared_ptr<CollisionGeometry>( 
	//			new Cylinder(Eigen::Vector3d(0, -3,0), Eigen::Vector3d(1,1,0)) ));
	
//	coll_geom_vec.push_back( std::shared_ptr<CollisionGeometry>( 
//				new Sphere(Eigen::Vector3d(5,-22,0), 20.0) ));
//	coll_geom_vec.push_back( std::shared_ptr<CollisionGeometry>( 
//				new Sphere(Eigen::Vector3d(-5,-22,0), 20.0) ));
	return coll_geom_vec;
}




















