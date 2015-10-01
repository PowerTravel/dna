#ifndef CHAIN_HPP
#define CHAIN_HPP



#ifndef MAX_GRID_SIZE
#define MAX_GRID_SIZE 10000
#endif // MAX_GRID_SIZE

#ifndef NR_OF_DIRECTIONS
#define NR_OF_DIRECTIONS 6
#endif // NR_OF_DIRECTIONS

#ifndef PRINT_SPACING
#define PRINT_SPACING 4
#endif // PRINT_SPACING

#include <Eigen/Dense>
#include <vector>
#include <random>
#include <map>
#include <string>
#include <fstream>

#include "Sphere.hpp"
#include "Spring.hpp"
class Chain
{
	public:
		Chain();
//		Chain(int seed);
		virtual ~Chain();
		
		void update(double dt = 0.01);

		// Returns position of all the links
//		Eigen::VectorXd getPos();
		// Returns Velocity of all the links
//		Eigen::VectorXd getVel();

		// Returns potential and kinetic energy of the system
//		Eigen::Vector2d getEnergy();

		// Generera kedjan i en spatial haschmap
		void generateGlobule(int N);
		
		friend std::ostream& operator<<(std::ostream& os, const Chain& c);
		
//		void print_knots();

		double get_mean_squared_distance(int start, int end)
		{
			if(_globule.empty() || (start>=0 && end < _globule.size() ))
			{
				return 0.0;
			}
		
			Eigen::Vector3d len = _globule[end].pos - _globule[start].pos;
			return sqrt(len.norm());
		}

		double get_mean_squared_distance()
		{
			if(_globule.empty())
			{
				return 0.0;
			}
			Eigen::Vector3d len = _globule.back().pos - _globule.front().pos;
			return sqrt(len.norm());
		}
		int _redo;
	private:


		struct knot{
			int start;
			int len;
		};

		struct link{
			link(){
				nr=0;
				pos = Eigen::Vector3d(0,0,0);
			};
			link(int n, Eigen::Vector3d p)
			{
				nr = n;
				pos = p;
			};
		
			int nr;
			Eigen::Vector3d pos;
		};

		bool _verbose;

		static std::default_random_engine _generator;
//		static std::map<int, std::string > _int_dir;
//		static std::map<std::string, int > _dir_int;

		int _N;
		int _n;
//		std::vector<Sphere> _links;
//		std::vector<Spring> _spring;

		//std::map<int, link*> _globule_map;
		std::vector< link > _globule;
		std::vector< knot > _knots;
		int _knots_check;

		int hash_fun(Eigen::Vector3d x);

		Eigen::Vector3d int_to_coord(int i);
		int coord_to_int(Eigen::Vector3d pos);

	//	link getNextLink(std::map<int,int>& m);
		Eigen::Vector3d getNextStep(std::map<int,int>& m);
		Eigen::VectorXd get_stepping_PDF(std::map<int,int>& m);
		Eigen::VectorXd PDF_to_CDF(Eigen::VectorXd f);
		Eigen::Vector3d nextStep(Eigen::VectorXd F);
/*
		struct DirMapConstructor{
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
*/		
};

#endif // CHAIN_HPP
