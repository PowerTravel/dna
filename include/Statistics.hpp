#ifndef STATISTICS_hpp
#define STATISTICS_hpp

#include "PFloat.hpp"
#include <Eigen/Dense>

class Statistics{

	public:
		Statistics();
		virtual ~Statistics(); 
		static Eigen::Vector2d get_mean_and_variance(Eigen::ArrayXd in_data, std::vector<PFloat>& weight);
		static Eigen::Vector2d get_mean_and_variance(Eigen::ArrayXd);

};

#endif // STATISTICS_HPP

