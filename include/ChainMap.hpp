#ifndef CHAIN_MAP_HPP
#define CHAIN_MAP_HPP

#ifndef DEFAULT_MAP_SIZE
#define DEFAULT_MAP_SIZE 10000
#endif

#ifndef MAX_MAP_SIZE
#define MAX_MAP_SIZE UINT_MAX
#endif

#ifndef INDEX_TYPE
typedef unsigned int idx_type;
#endif 

#ifndef HASH_PRIMES
#define X_PRIME 137
#define Y_PRIME 149
#define Z_PRIME 163
#endif  

//#include<array>
//typedef array m_list
#include<vector>
#include<Eigen/Dense>
class ChainMap{

	public:
		ChainMap(int size =DEFAULT_MAP_SIZE);
		virtual ~ChainMap();

		void push(Eigen::Array3d pos);
		Eigen::ArrayXXd as_array();


		void print();
		int get_nr_neigbours(Eigen::Array3d pos);
	private:
		struct link{
			link()
			{
				pos = Eigen::Array3d(0,0,0);
			};
			link(Eigen::Array3d p)
			{
				pos = p;
			};

			Eigen::Array3d pos;
		};

		int x_prime, y_prime, z_prime;
		idx_type mod_size;

		Eigen::ArrayXi _map;
		Eigen::ArrayXi _neighbours;
		std::vector<link> _chain;
		int _i;
		
		Eigen::Array3d hash_function(Eigen::Array3d pos);
		void add_position(Eigen::Array3d pos, idx_type idx);

		void increment_neighbours(Eigen::Array3d pos);
		Eigen::Array3d int_to_coord(int i);
		idx_type get_idx(Eigen::Array3d hashed_pos);

		bool compare_position(Eigen::Array3d pos1,Eigen::Array3d pos2);
};

#endif // CHAIN_MAP_HPP
