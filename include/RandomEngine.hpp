#ifndef RANDOM_ENGINE_HPP
#define RANDOM_ENGINE_HPP

#include <random>
#include <ctime>

class RandomEngine{
	public:

		RandomEngine( double min = 0.0, double max = 1.0);
		virtual ~RandomEngine();

		double get();
	private:
	
		static std::default_random_engine _generator;
		std::uniform_real_distribution<double> _distribution;
};

#endif // RANDOM_ENGINE_HPP
