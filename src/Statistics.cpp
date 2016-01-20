#include "Statistics.hpp"

Statistics::Statistics()
{

}
Statistics::~Statistics()
{

}

Eigen::Vector2d Statistics::get_mean_and_variance(Eigen::ArrayXd in_data, std::vector<PFloat>& weight)
{
	Eigen::Vector2d ret = Eigen::Vector2d::Zero();
	
	// Mean
	double M = weight.size();
	double N = in_data.size();

	PFloat sum = PFloat();
	for(int i=0; i < M; i++ )
	{
		sum = sum + weight[i];
	}

	PFloat unity = 1;
	PFloat scale = unity / sum;
	Eigen::ArrayXd s_weight = Eigen::ArrayXd(M);
	for(int i=0; i<M; i++)
	{
		PFloat tmp = scale * weight[i];
		s_weight(i) = tmp.as_float();
	}
	// Weights are scaled to be unity and are written out just for clarity
	double w_sum = 1.0;
	ret(0) = (in_data * s_weight).sum() / w_sum;
	double var_denominator = ((M-1)/(M)) * w_sum; 
	double var_numerator   = ( s_weight * (in_data-ret(0)).pow(2) ).sum();
	ret(1) = var_numerator / var_denominator;

	return ret;
}
