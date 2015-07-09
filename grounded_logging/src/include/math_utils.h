#ifndef _math_utils_
#define _math_utils_

vector<double> compute_normalized_deviations(vector<double> values, vector<double> means, vector<double> stdevs){
  vector<double> devs = vector<double>(values.size());
  for (unsigned int j = 0; j < values.size(); j++)
    devs[j]=abs((values[j]-means[j])/stdevs[j]);
  return devs;
};

double get_vector_sum(vector<double> values){
  double sum=0;
  for (unsigned int i = 0; i < values.size(); i++)
    sum+= values[i];
  

  return sum;
};

/* returns the mean of the input vector of values */
double get_vector_mean(vector<double> values){
  double mean=0;
  for (unsigned int i = 0; i < values.size(); i++)
    mean+= values[i];

  return mean/values.size();
};

/* returns the standard deviation given a vector of values, and their mean */	
double get_vector_stdev(vector<double> values, double mean){

  std::vector<double> squared_diffs = std::vector<double>((int)values.size());

  for (unsigned int i = 0; i < values.size(); i++){
    squared_diffs[i]= (values[i]-mean)*(values[i]-mean) ;
  }

  double expected_sq_diff = get_vector_mean(squared_diffs);
  return sqrt(expected_sq_diff);
};


#endif
