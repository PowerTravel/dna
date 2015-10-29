#ifndef VISUALIZE_HPP
#define VISUALIZE_HPP

#include "Simulation.hpp"
#include "PhantomChain.hpp"
class Visualize : public Simulation {

	public:
		Visualize();
		Visualize(std::map<std::string, std::string> sm);
		virtual ~Visualize();

		void apply();
		void print();	
		void print(std::ostream& os);
	
	private:
		int _size;
};

#endif // VISUALIZE_HPP
