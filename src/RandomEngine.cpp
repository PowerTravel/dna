#include "RandomEngine.hpp"

std::default_random_engine RandomEngine::_generator = std::default_random_engine(time(NULL));

RandomEngine::RandomEngine( double min, double max)
{
	 _distribution = std::uniform_real_distribution<double>(min,max);
}

RandomEngine::~RandomEngine()
{

}

double RandomEngine::get()
{
	return _distribution(_generator);
}

