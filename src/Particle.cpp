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
	
//	if(collisions.size()>0){
	bool next = false;
	while(collisions.size() > 0){
		if(collisions.size() > 1 || next)
		{
			std::cout << collisions.size() << std::endl;
			if(collisions.size() > 1 )
			{
				next = true;
			}else{
				next = false;
			}
		}
		// Move the collision with highest penetration-depth to the top to be
		// resolved first.
		// The assumption being that that was the first collision.
		collisions.sort(sort_after_penetration_depth);


		// Handle the first collision, update the position of the particle and do this all over again
		// Flip the normal of the collision - geometry such that v dot n < 0
		intersections c = align_normal(collisions.front(), _v);

		Eigen::VectorXd tmp = do_one_collision(dt, X,a, c);
		X = tmp.segment(0,7);
		X_P = tmp.segment(7,7);
	
	//	std::cout << X.transpose() << std::endl;
	//	std::cout << X_P.transpose() << std::endl;
	//	exit(0);
		
		S = Sphere(X_P.segment(1,3), _r);
		collisions = get_coll_list(coll_geom_vec, S);

		//if( i > 10)
		//{
		//	std::cout << "program failed" << std::endl;
		//	exit(1);
		//}else{
		//	i++;
		//}
	}

	_x = X_P.segment(1,3);
	_v = X_P.segment(4,3);
	traj.push_back(X_P.segment(1,6));

//	x = xp;
//	v = vp;
//	Eigen::VectorXd tt = Eigen::VectorXd::Zero(6);
//	tt.segment(0,3) = x;
//	tt.segment(3,3) = v;
//	traj.push_back(tt);
//	std::cout << tt.transpose() << std::endl;
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
	double len = v_c.transpose() * cs.n;
	Eigen::Vector3d v_norm  = len * cs.n;
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




















