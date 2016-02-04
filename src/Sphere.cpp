#include "Sphere.hpp"
#include "Plane.hpp"
#include "Cylinder.hpp"
#include <iostream>

Sphere::Sphere()
{
	_r = 0.5;
	_x = Eigen::Vector3d::Zero();
}

Sphere::Sphere(Eigen::Array3d xp, double rad)
{
	_r = rad;
	_x = xp.matrix();
}

Sphere::~Sphere()
{

}

std::string Sphere::text_type()
{
	return std::string("Sphere");
}

/*
bool Sphere::intersects(Cylinder* c, coll_struct& cs)
{
	Eigen::Vector3d cx = c->get_pos();
	Eigen::Vector3d cd = c->get_orientation();
	double cr = c->get_radius();
	double ch = c->get_height();

	Eigen::Vector3d scs = _x - cx;
	double len = scs.transpose() * cd;
	Eigen::Vector3d scs_parallel = len * cd;
	Eigen::Vector3d scs_anti_parallel = scs - scs_parallel;

	// This means that the sphere lies outside the cylinder radius
	if( scs_anti_parallel.norm() > (_r+cr) )
	{
		return false;
	}


	// The sphere lies below of the cylinder
	if( scs_parallel.dot( cd ) <= 0 )
	{
		if( scs.norm() < (cr+_r)  )
		{
			// Set the collision struct
			return true;
		}
			
	}else{
	
		// The sphere lies above the top of the cylinder
		if(scs_parallel.norm() < ( ch+_r ) )
		{
			return false;
		}
		
	}

	//std::cerr << "intersects Sphere -> Cylinder not implemented " << std::endl;
	return false;
}
*/

bool Sphere::intersects(Cylinder* c, coll_struct& cs)
{
//	std::cerr << "BAJSBAJSBAJSBAJSBAJS!!!!" << std::endl;
	// Cylinder
	double rho_c = c->_r;
	double h = c->_h;
	Eigen::Vector3d P = c->_P;
	Eigen::Vector3d Q = c->_Q;
	
	// Sphere
	Eigen::Vector3d A = _x;
	double rho_s = _r;

	Eigen::Vector3d PQ = Q-P;
	Eigen::Vector3d PA = A-P;


	double PQPA = PQ.dot(PA);
	double h2 = h*h;
	
	Eigen::Vector3d B = (PQPA/h2) * PQ + P;

	// radial vec from sphere center towards cylinder axis
	Eigen::Vector3d er = (B-A);
	double delta = er.norm();
	er = er/delta;

	double rho_sp = 0;
//	std::cout << "=====================" << std::endl;
//	std::cout << delta<< std::endl;
	// P->Q->B
	if(PQPA > h2)
	{	
//		std::cout <<"A = " << A.transpose()<< std::endl;
//		std::cout <<"P = " << P.transpose();
//		std::cout <<"; Q = " << Q.transpose();
//		std::cout <<"; B = " << B.transpose() << std::endl;
		double d = (PQPA-h2)/h;
	//	std::cout << "d = " << d << "  r_s = " << rho_s << std::endl;
		rho_sp =std::sqrt(rho_s*rho_s - d*d);
	
		// Sphere is bumping into cylinder corner.
		if(delta > rho_c )
		{
			cs.n = -(er*rho_sp - c->_d*d).normalized();
		// Sphere is bumping into the top
		}else{
			cs.n = c->_d;
		}
	// B->P->Q
	}else if(PQPA < 0){
//		std::cout <<"A = " << A.transpose()<< std::endl;
//		std::cout <<"B = " << B.transpose();
//		std::cout <<"; P = " << P.transpose();
//		std::cout <<"; Q = " << Q.transpose() << std::endl;

		Eigen::Vector3d QA = A-Q;
		Eigen::Vector3d QP = P-Q;
		double QPQA = QP.dot(QA);

		// This is not supposed to happen normally
		// But if it does ill print out this line just for saftey.
		if(QPQA < h2)
		{
			std::cerr << "Something weird going on with cylinder intersection test" << std::endl;
		}
		double d = (QPQA-h2)/h;

		rho_sp =std::sqrt(rho_s*rho_s - d*d);

		// Sphere is bumping into cylinder corner.
		if(delta > rho_c )
		{
			cs.n = -(er*rho_sp + c->_d*d).normalized();
		// Sphere is bumping into the bottom 
		}else{
			cs.n = -c->_d;
		}

	// P->B->Q
	}else{
	//std::cout << _x.transpose() << std::endl; 
//		std::cout <<"A = " << A.transpose()<< std::endl;
//		std::cout <<"P = " << P.transpose();
//		std::cout <<"; B = " << B.transpose();
//		std::cout <<"; Q = " << Q.transpose() << std::endl;
		
		cs.n = er;
		rho_sp = rho_s;
	}

		//std::cout << rho_sp << std::endl;
	//	std::cout << delta << std::endl;
	if(delta < rho_sp +rho_c)
	{
		std::cout <<"A = " << A.transpose();
		std::cout <<"; P = " << P.transpose();
		std::cout <<"; B = " << B.transpose();
		std::cout <<"; Q = " << Q.transpose() << std::endl;
		cs.p = (rho_sp + rho_c) - delta;
		std::cout << "cs.p = " << cs.p << " cs.n =  " << cs.n.transpose() << std::endl;
		return true;
	}else{
		cs.p = 0;
		cs.n = Eigen::Vector3d::Zero();
		return false;
	}
}


bool Sphere::intersects(Sphere* s, coll_struct& cs)
{
	Eigen::Vector3d R =  s->_x - this-> _x;
	double separation = R.norm();
	double contact_distance = s->_r+this->_r;
	
	if( separation  < contact_distance )
	{
		cs.n = R.normalized();
		cs.p = contact_distance - separation;	
		return true;

	}else{
		return false;
	}
}

bool Sphere::intersects(Plane* p, coll_struct& cs)
{
	Eigen::Vector3d pc = p->x;
	Eigen::Vector3d pn = p->n;

	// Find the distance from the plane to the sphere center
	double plane_sphere_distance =std::abs( ( _x - pc ).transpose() * pn);
	if( plane_sphere_distance < _r )
	{
		cs.p = _r - plane_sphere_distance; // Penetration depth
		cs.n = pn;						  // Collision plane normal
		return true;
	}

	return false;
}

double Sphere::line_intersection_point(Eigen::ArrayXd x, Eigen::ArrayXd v)
{
	// Sphere data
	Vec3d sphere_center = _x;
	double sphere_radius = _r;

	// line data
	Vec3d line_point = x.matrix();
	Vec3d line_direction = v.matrix();

	Eigen::Vector3d separation = line_point-sphere_center;
	//  dotproducts
	double ld_dot_ld = line_direction.transpose() * line_direction;
	double ld_dot_separation =   (line_direction.transpose() * separation);
	double sep_dot_sep = separation.transpose() * separation;
	
	double p =2.0 * ld_dot_separation / ld_dot_ld;
	double q = (sep_dot_sep - sphere_radius*sphere_radius) / ld_dot_ld;

	double p_half = (p*0.5);
	double p2 = p_half * p_half;

	if(p2 < q)
	{
		std::cerr << "Sphere::line_intersection_point" << std::endl;
		std::cerr << "	Immaginary solution" << std::endl;
		std::cerr << "	Sphere center: " << sphere_center.transpose() << std::endl;
		std::cerr << "	Sphere radius: " << sphere_radius << std::endl;
		std::cerr << "	line point: " << line_point.transpose() << std::endl;
		std::cerr << "	line_direction: " << line_direction.transpose() << std::endl;
		return 0;
	}
	
	double intersection_scalar_1 = - p_half - std::sqrt( p2 - q );
	double intersection_scalar_2 = - p_half + std::sqrt( p2 - q );

	if(intersection_scalar_1 < intersection_scalar_2){
		return intersection_scalar_1;
	}else{
		return intersection_scalar_2;
	}
}

// Returns min max values along the axis 
Eigen::ArrayXd Sphere::get_span()
{
	Eigen::ArrayXd ret = Eigen::ArrayXd::Zero(6);
	ret(0) = _x(0)-_r; // x_min
	ret(1) = _x(0)+_r; // x_max
	ret(2) = _x(1)-_r; // y_min
	ret(3) = _x(1)+_r; // y_max
	ret(4) = _x(2)-_r; // z_min
	ret(5) = _x(2)+_r; // z_max
	return ret;
}

Vec3d Sphere::get_center()
{
	return _x;
}
