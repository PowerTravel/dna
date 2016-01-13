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
	Eigen::Vector3d xv = x.matrix();
	Eigen::Vector3d vv = v.matrix();
	Eigen::Vector3d separation = _x-xv;
	double v_dot_v = vv.transpose() * vv;
	double p =   (vv.transpose() * separation);
	p =2.0 * p / v_dot_v;
	double q = (separation.transpose() * separation - _r*_r) / v_dot_v;

	double p2 = std::pow(p/2.0,2);
	if(p2 < q)
	{
		std::cerr << "Immaginary solution to sphere - line intersection" << std::endl;
		return 0;
	}
	
	double ret_1 = - p/2.0 - std::sqrt( p2 - q );
	double ret_2 = - p/2.0 + std::sqrt( p2 - q );

	// May be overkill but always works
	Eigen::Vector3d v1 =  xv + ret_1 * vv;
	Eigen::Vector3d v2 =  xv + ret_2 * vv;
	if( vv.transpose() *  (v1 - xv ) >= 0)
	{
		return ret_1;
	}else{
		return ret_2;
	}

/*
// this assumes that the penetration depth is lower than the radius of the sphere.
	if(ret_1 <= ret_2)
	{
		return ret_1;
	}else{
		return ret_2;
	}
*/	
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
