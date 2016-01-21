#include "Statistics.hpp"

Statistics::Statistics()
{

}
Statistics::~Statistics()
{

}

Eigen::Vector2d Statistics::get_mean_and_variance(Eigen::ArrayXd in_data)
{
	Eigen::Vector2d ret = Eigen::Vector2d::Zero();
	// Mean
	double N = in_data.size();
	if(N == 1)
	{
		return Eigen::Vector2d(in_data(0), 0);
	}
	// Weights are scaled to be unity and are written out just for clarity
	ret(0) = in_data.sum() / N;
	double var_denominator = (N-1); 
	double var_numerator   = (in_data-ret(0)).pow(2).sum();
	ret(1) = var_numerator / var_denominator;
	return ret;
}

Eigen::Vector2d Statistics::get_mean_and_variance(Eigen::ArrayXd in_data, std::vector<PFloat>& weight)
{
	Eigen::Vector2d ret = Eigen::Vector2d::Zero();
	
	// Mean
	double M = weight.size();
	double N = in_data.size();

	if(M!=N)
	{	
		std::cerr << "Error, size of weights not equal to size of data" << std::endl;
		std::cerr << "Error thrown from Statistics::get_mean_and_variance" <<std::endl;
		return Eigen::Vector2d(0,0);
	}

	PFloat sum = PFloat();
	for(int i=0; i < N; i++ )
	{
		sum = sum + weight[i];
	}

	PFloat unity = 1;
	PFloat scale = unity / sum;
	Eigen::ArrayXd s_weight = Eigen::ArrayXd(N);
	for(int i=0; i<N; i++)
	{
		PFloat tmp = scale * weight[i];
		s_weight(i) = tmp.as_float();
	}
	// Weights are scaled to be unity and are written out just for clarity
	double w_sum = 1.0;
	ret(0) = (in_data * s_weight).sum() / w_sum;
	double var_denominator = ((N-1)/(N)) * w_sum; 
	double var_numerator   = ( s_weight * (in_data-ret(0)).pow(2) ).sum();
	ret(1) = var_numerator / var_denominator;

	return ret;

}

Eigen::ArrayXd Statistics::make_exponential_points_array(double N, double steps, double start_point )
{
	Eigen::ArrayXd n = Eigen::ArrayXd::Zero(steps+1);
	double k = log(N / start_point) / steps; // exponential
	
	for(int i = 0; i<steps+1; i++){
		n(i) = start_point * exp(k*i);
	}
	return n;
}

Eigen::ArrayXd Statistics::make_linear_points_array(double N, double steps, double start_point )
{
	Eigen::ArrayXd n = Eigen::ArrayXd::Zero(steps+1);
	
	double dn = (N - start_point) / steps; // linear

	for(int i = 0; i<steps+1; i++){
		n(i) = i * dn + start_point;	
	}
		
	return n;
}
