#ifndef _background_model_fft_
#define _background_model_fft_

//some standard C++
#include <vector>
#include <stdio.h>
#include <string>
#include <cstdio>
#include <deque>
#include <math.h>

//my math utils
#include "math_utils.h"
#include "CircularBuffer.h"

#define FRAC_BINS_THRESH 0.3
#define STDEV_DIFF_THRES 2.5

using std::vector;

class BackgroundModelFFT {
	public:

		BackgroundModelFFT(){
			last_answer = false; //assume we start in background
			num_time_steps_in_state=1;
		};


		//estimates the mean and standard deviation 
		void estimateModel(vector< vector<double> > fft){
			
			num_fft_bins = fft[0].size();

			bin_means = vector<double>(num_fft_bins);
			bin_stdevs = vector<double>(num_fft_bins);


			for (int i = 0; i < num_fft_bins; i++){
				vector<double> bin_values = vector<double>(fft.size());
				
				for (int j = 0; j < (int)fft.size(); j++){
					bin_values[j]=fft[j][i];
				}

				//compute mean and standard deviation of values for that bin
				double mean_i = get_vector_mean(bin_values);
				double stdev_i = get_vector_stdev(bin_values, mean_i);
			
				bin_means[i]=mean_i;
				bin_stdevs[i]=stdev_i;
				std::cout << "Bin #"<<i<<"\t"<<mean_i<<"\t+/-\t"<<stdev_i<<"\n";
			}

			//compute average volume
			vector<double> vol_values;
			for (unsigned int i = 0; i < fft.size(); i++){
				vol_values.push_back(get_vector_sum(fft[i]));
			}

			vol_mean = get_vector_mean(vol_values);
			vol_stdev = get_vector_stdev(vol_values, vol_mean);
			printf("Vol. mean and stdev: %f\t+/-\t%f\n",vol_mean,vol_stdev);
		};
		
		
		
		//classifies a single FFT sample as background (false) or foreground (true)
		bool classify(vector<double> fft_column){
		  int frac_bins = (int)floor( FRAC_BINS_THRESH * (double) num_fft_bins ); //this many bins have to exceed threshold
		  classify_answer = label_fft_column_by_bin_deviations(fft_column,frac_bins,STDEV_DIFF_THRES);
		  
		  //state transition
		  if (classify_answer != last_answer){
				num_time_steps_in_state=1;
				last_answer=classify_answer;
		  } else {
				num_time_steps_in_state++;
		  }
		  
		  
		  return classify_answer;
		}
		
		
		
		//labels the FFT column/msg as either background or foreground using only volume information
		//if the total intensity of the column is greater by more than k standard deviations from the mean
		//volume of the background, then it is labeled as non-background
		bool label_fft_column_by_volume(vector<double> fft_column, double k_stdev_coefficient){
		  double column_intensity = get_vector_mean(fft_column);
		  if (column_intensity > vol_mean + k_stdev_coefficient*vol_stdev)
		    return true;
		  else return false;
		};
		
		
		//if frac_bins*num_fft_bins bins deviate from the mean by more than dev_theta, then the column
		//is labelled as non-background
		bool label_fft_column_by_bin_deviations(vector<double> fft_column, int frac_bins, double dev_theta){
		  vector<double> dev_column = compute_normalized_deviations(fft_column,bin_means,bin_stdevs);
		  count_dev = 0;
		  amount_dev=0.0;
		  pct_dev_bins = 0.0;
		  mean_dev_bins_amount_deviation=0.0;
		  
		  for (unsigned int i = 0; i < dev_column.size(); ++i){
				amount_dev += dev_column[i];
				
				if (dev_column[i] > dev_theta){
					count_dev++;
					mean_dev_bins_amount_deviation+=dev_column[i];
				}
		  }
		  amount_dev=amount_dev/(double)dev_column.size();
		 
		  if (count_dev!=0){
				mean_dev_bins_amount_deviation=mean_dev_bins_amount_deviation/(double)count_dev;
				pct_dev_bins = ((double)count_dev)/(double)dev_column.size();
		  }
		 
		  //add to filters
		  amount_dev_f.push(amount_dev);
		  pct_dev_bins_f.push(pct_dev_bins);
		  mean_dev_bins_amount_deviation_f.push(mean_dev_bins_amount_deviation);
		  
		  if (count_dev > frac_bins)
		    return true; //not background
		  else return false; //background
		};
		
		//returns the number of frequency bins that deviate by more than the threshold in the last measurement
		int get_num_deviating_bins(){
			return count_dev;
		};
		
		//returns the total amount of deviation from the last measurement, summer over all bins, in standard deviations
		double get_deviation_amount(){
			//return amount_dev;
			return amount_dev_f.mean();
		};
		
		double get_pct_deviating_bins(){
			return pct_dev_bins_f.mean();
		};
		
		double get_deviating_bins_deviation_amount(){
			//return mean_dev_bins_amount_deviation;
			return mean_dev_bins_amount_deviation_f.mean();
		};
		
		unsigned long get_num_time_steps_in_state(){
			return num_time_steps_in_state;
		};
		
		//for each frequency bin, look at how much the value in the input deviates from the expected mean
		//measured in terms of standard deviations
		vector<double> compute_combined_binned_deviation(vector<double> fft_column){
		  vector<double> dev_column = vector<double>(num_fft_bins);
		  for (unsigned int j = 0; j < fft_column.size(); j++)
		      dev_column[j]=abs((fft_column[j]-bin_means[j])/bin_stdevs[j]);
		  return dev_column;
		};

		vector< vector<double> > compute_deviation_from_background_matrix(vector< vector<double> > fft_in){
			vector< vector<double> > fft = fft_in;
			vector< vector<double> > dev;

			for (unsigned int i = 0; i < fft.size(); i++){
				vector<double> dev_column = vector<double>(num_fft_bins);
				vector<double> fft_column = fft[i];

				for (unsigned int j = 0; j < dev_column.size(); j++)
					dev_column[j]=   abs((fft_column[j]-bin_means[j])/bin_stdevs[j]);

				dev.push_back(dev_column);

			}

			return dev;
		}

		//returns a matrix of zeros and ones, where zero at (i,j) indicates that
		//that at time j, the dB level for bin i was above the threshold standard deviations
		vector< vector<double> > zero_one_filter(vector< vector<double> > fft_in, double threshold){
			vector< vector<double> > dev_matrix = compute_deviation_from_background_matrix(fft_in);
			
			for (unsigned int i = 0; i < dev_matrix.size(); i++){
				for (unsigned int j = 0; j < dev_matrix[i].size(); j++){
					if (dev_matrix[i][j]>threshold)
						dev_matrix[i][j]=1;
					else dev_matrix[i][j]=0;
				}
			}

			return dev_matrix;
		}

		//labels each time samples in the input FFT as either above (1) or below (0) the expected volume
		vector<int> label_by_volume(vector< vector<double> > fft_in, double threshold){
			vector< vector<double> > fft = fft_in;
		
			vector<int> labels = vector<int>(fft.size());

			for (unsigned int i = 0; i < labels.size(); i++){
				double volume_i = get_vector_sum(fft[i]);
				
				double num_devs_diff = abs(volume_i-vol_mean)/vol_stdev;

				if (num_devs_diff > threshold){
					labels[i]=1;
				}
				else labels[i]=0;
			}
			return labels;
		}



	private:
		int num_fft_bins;
		vector<double> bin_means;
		vector<double> bin_stdevs;
		
		double threshold;

		double vol_mean, vol_stdev;
		
		//how many frequency bins diverge from background model
		int count_dev;
		
		//by how much the values in all frequency bins diverge from background model (in terms of standard deviations)
		double amount_dev;
		CircularBuffer amount_dev_f;
		
		//by how much the values in frequency bins labeled as deviating diverge from background model (in terms of standard deviations)
		double mean_dev_bins_amount_deviation;
		CircularBuffer mean_dev_bins_amount_deviation_f;
		
		//pct (0-1) bins that deviate from background
		double pct_dev_bins;
		CircularBuffer pct_dev_bins_f;
		
		//stores the answer to the classify function
		bool classify_answer;
		bool last_answer; //keeps track of previous answer in order to detect transitions
		
		//stores for how many time steps the background model has been in the currnet state
		unsigned long num_time_steps_in_state;

		
};

#endif
