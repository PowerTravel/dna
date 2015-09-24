#ifndef CHAIN_HPP
#define CHAIN_HPP


#ifndef MAX_GRID_SIZE
#define MAX_GRID_SIZE 1000
#endif // MAX_GRID_SIZE

#include <vector>
#include <Eigen/Dense>
#include <random>
#include <ctime>
#include <map>
#include <string>

#include "Sphere.hpp"
#include "Spring.hpp"
class Chain
{
	public:
		Chain();
		Chain(int seed);
		virtual ~Chain();
		
		void update(double dt = 0.01);

		// Returns position of all the links
		Eigen::VectorXd getPos();
		// Returns Velocity of all the links
		Eigen::VectorXd getVel();

		// Returns potential and kinetic energy of the system
		Eigen::Vector2d getEnergy();

		void printChain();

		// Generera kedjan i en spatial haschmap
		void generateGlobule(int N);
	private:

		struct link{
			int nr;
			Eigen::Vector3d pos;
			link* _prev;
			link* _next;
			bool knot;
		};

		static std::default_random_engine _generator;
		static std::map<int, std::string > _int_dir;
		static std::map<std::string, int > _dir_int;

		int _N;
		std::vector<Sphere> _links;
		std::vector<Spring> _spring;

		std::map<int, int> _globule;

		int hash_fun(Eigen::Vector3d x);

		Eigen::Vector3d int_to_coord(int i);
		int coord_to_int(Eigen::Vector3d pos);

		Eigen::VectorXd get_stepping_PDF(Eigen::Vector3d pos);
		Eigen::VectorXd PDF_to_CDF(Eigen::VectorXd f);
		Eigen::Vector3d getNewPos(Eigen::Vector3d pos);

		int _knots;
		int knot_length;

		int print_dots(double i, double N, double printed_dots, double tot_dots);

		struct Dir{
			static std::map<int, std::string> int_to_dir_map()
			{
				std::map<int, std::string> m;
				m[0] = "+x";	
				m[1] = "-x";	
				m[2] = "+y";	
				m[3] = "-y";	
				m[4] = "+z";	
				m[5] = "-z";	
				return m;
			};
			
			static std::map<std::string, int> dir_to_int_map()
			{
				std::map<std::string, int> m;
				m["+x"] = 0;	
				m["-x"] = 1;	
				m["+y"] = 2;	
				m["-y"] = 3;	
				m["+z"] = 4;	
				m["-z"] = 5;	
				return m;
			};
		};
};

#endif // CHAIN_HPP
