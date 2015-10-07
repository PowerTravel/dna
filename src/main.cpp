#include <iostream>
#include "RChain.hpp"
#include "ChainStatistics.hpp"


/*
 * 	Program dna
 *	Generates a fractal globule and prints data to a file
 */


void build_initial_state(int n);
void get_average_radious_of_gyration(int N);



int main(int argc, char* argv[])
{
/*
	if(argc == 3)
	{
		int N = atoi(argv[2]);
		if(argv[1][0] == 'i')
		{
			build_initial_state(N);
		}else if( argv[1][0] == 'r'){
			get_average_radious_of_gyration(N);
		}
	}
	*/
	ChainStatistics cs = ChainStatistics();
	cs.generate();
	return 0;
}

void build_initial_state(int n)
{
	RChain c = RChain();
	c.build(n,RChain::ChainType::SAW);
	// Print to file
	std::ofstream file;
	file.open(INITIAL_STATE, std::fstream::out | std::fstream::trunc);
	if(file.is_open())
	{
		file << c << std::endl;
	}
	file.close();
}

void get_average_radious_of_gyration(int N)
{
	int tests = 100;

	double mean = 0;
	Eigen::ArrayXd arr = Eigen::ArrayXd::Zero(tests);
	double var = 0;

	for(int i=0; i<tests; i++)
	{
		RChain c = RChain();
		c.build(N , RChain::ChainType::PHANTOM);
		arr(i) =  c.get_rad_of_gyr();
	}
	mean = arr.sum()/((double) tests);	
	
	for(int i=0; i<tests; i++)
	{
		double ax =(mean - arr(i));
		var += ax*ax; 
	}
	var = var/((double)N);
	std::cout << "Rad = " << mean << "  stdavvikelse = " << sqrt(var) << std::endl; 
}
