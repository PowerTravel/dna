#include "Particle.hpp"

#include <iostream>
Particle::Particle(double rad, Eigen::Array3d pos,Eigen::Array3d vel, CollisionGrid* gr)
{
	grid = gr;

	x = pos.matrix();
	r = rad;
	v = vel.matrix();
	first_step_taken = false;

	traj = std::vector<Eigen::VectorXd>();
}

Particle::~Particle()
{

}

Eigen::Array3d Particle::get_position()
{
	return x;
}

Eigen::Array3d Particle::get_velocity()
{
	return v;
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

	Eigen::Vector3d vp = v + h*a.matrix();
	Eigen::Vector3d xp = x + dt*v;





	bool foundIntersections;

	//do{
	foundIntersections = false;
	Sphere S = Sphere(xp, r);

	collisions = std::list<intersections>();
	// FIND ALL INTERSECTIONS
	for(auto coll_geom = coll_geom_vec.begin(); coll_geom != coll_geom_vec.end();
		coll_geom++)
	{

		//Plane P = Plane(Eigen::Vector3d(0,0,0), Eigen::Vector3d(0,1,0));
		CollisionGeometry::coll_struct cs;
		std::shared_ptr<CollisionGeometry> c = *coll_geom;
		if(c->intersects(&S, cs))
		{
			intersections is;
			is.geom = c;
			is.cs = cs;
			collisions.push_back(is);
			foundIntersections = true;
		
	/*	
			Eigen::Vector3d pn = cs.n;
			if( v.transpose() * pn > 0)
			{
				pn = -pn;	
			}			
			// Hitta kollsionspunkten som är 
			// Porjicera en linje mellan planet och sfären för att hitta
			// first contact punkten
			Eigen::Vector3d contact_point = x - r * pn;

			double dt_t = c->line_intersection_point(contact_point, vp);


			Eigen::Vector3d x_c = x +  dt_t * vp;

			// Komposantuppdela hastigheten runt plane normal
			double len = v.transpose() * pn;
			Eigen::Vector3d v_paralell  = len * pn;
			Eigen::Vector3d v_antiparalell  = vp - v_paralell;
		
			vp = v_antiparalell - v_paralell;
			xp = x_c +(dt-dt_t) * vp;
		*/
		}
	}
	
	//if(collisions.size() > 1)
	//{
//		std::cerr << "AJSBAJSBDAJBD " << collisions.size() << std::endl;
//	}

	// SORT THEM SO THAT THE ONE WITH THE LARGEST PENTETRATION DEPTH GETS HANDLED FIRST
	
	collisions.sort(sort_after_penetration_depth);

	// Handle the first collision, update the position of the particle and do this all over again
	CollisionGeometry::coll_struct cs;
	if(collisions.size() > 0)
	{
	
		intersections c = collisions.front();
	if(c.geom->intersects(&S, cs))
	{
		//Eigen::Vector3d pn = P.getPlaneNormal();	
		CollisionGeometry::coll_struct cs = c.cs;

		Eigen::Vector3d pn = cs.n;
		if( v.transpose() * pn > 0)
		{
			pn = -pn;	
		}			
		// Hitta kollsionspunkten som är 
		// Porjicera en linje mellan planet och sfären för att hitta
		// first contact punkten
		Eigen::Vector3d contact_point = x - r * pn;

		double dt_t = c.geom->line_intersection_point(contact_point, vp);
		Eigen::Vector3d x_c = x +  dt_t * vp;

		std::cout << x_c.transpose() << std::endl;

		// Komposantuppdela hastigheten runt plane normal
		double len = vp.transpose() * pn;
		Eigen::Vector3d v_paralell  = len * pn;
		Eigen::Vector3d v_antiparalell  = vp - v_paralell;
		
		vp = v_antiparalell - v_paralell;
		xp = x_c +(dt-dt_t) * vp;
	}
	}
//	}else{
//		std::cerr  << "asgnalsdgkjnlakgn" << std::endl; 
//	}
	//}while(foundIntersections);

	v = vp;
	x = xp;
	S = Sphere(x.array(),r);

	Eigen::VectorXd tmp = Eigen::VectorXd::Zero(6);
	tmp.segment(0,3) = x;
	tmp.segment(3,3) = v;
	traj.push_back(tmp);
	
}
void Particle::do_one_collision(intersections is )
{

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
//	coll_geom_vec.push_back( std::shared_ptr<CollisionGeometry>( 
//				new Sphere(Eigen::Vector3d(0,0,0), 1.0) ));
	
	return coll_geom_vec;
}




















