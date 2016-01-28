#ifndef STATISTICS_hpp
#define STATISTICS_hpp

#include "PFloat.hpp"
#include <Eigen/Dense>
#include <vector>

class Statistics{

	public:
		Statistics();
		virtual ~Statistics(); 
		static Eigen::Vector2d get_mean_and_variance(Eigen::ArrayXd in_data, std::vector<PFloat>& weight);
		static Eigen::Vector2d get_mean_and_variance(Eigen::ArrayXd);


		static Eigen::ArrayXd make_exponential_points_array(double N, double steps, double start_point );
		static Eigen::ArrayXd make_linear_points_array(double N, double steps, double start_point );

};

#endif // STATISTICS_HPP

