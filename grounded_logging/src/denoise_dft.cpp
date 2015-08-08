#include "ros/ros.h"
#include "std_msgs/String.h"
#include <math.h>  
#include <fstream>
#include <iostream>
// For traversing the filesystem
#include <boost/filesystem.hpp>
#include <boost/foreach.hpp> 

using namespace std;
using namespace boost::filesystem;

// The general file path
std::string generalFilePath = "/home/users/pkhante/grounded_learning_experiments/";

//Name of the audio folder to search for
const std::string & audio_folder = "audio_data";

//Name of the hold behaviour folder to search for
const std::string & hold_folder = "hold";

//Total number of objects and trials for ease of file traversal
int total_objects = 1, total_trials = 1;

// Create a vector to put in the dft datapoints
vector<vector<double> > dft_data;

// Create a vector to put the intermediate values to calculate the variance
vector<vector <double> > distance_from_mean;

// Some variables for the various calculations
double mean, variance, standard_deviation, threshold;

int main (int argc, char** argv)
{
	// Initialize ROS
	ros::init (argc, argv, "denoise_dft");
	ros::NodeHandle nh;
	ROS_INFO("Ready to denoise the dft files...");
	
	for(int object_num = 1; object_num <= total_objects; object_num++){
		for(int trial_num = 1; trial_num <= total_trials; trial_num++){
			std::stringstream convert1, convert2;
			convert1 << object_num;
			convert2 << trial_num;
			string filePath = generalFilePath + "obj_" + convert1.str() + "/trial_" + convert2.str();
			path dir_path(filePath);
			if(!exists(dir_path))
				return 1;
				
			directory_iterator end_itr;       // default construction yields past-the-end
			for(directory_iterator itr(dir_path); itr != end_itr; ++itr){
				if(itr->path().filename().string() == hold_folder){    // if the folder is the hold behaviour then get the txt file
					directory_iterator end_itr2;       // default construction yields past-the-end
					for(directory_iterator itr2(itr->path()); itr2 != end_itr2; ++itr2){
						if(itr2->path().filename().string() == audio_folder){
							//Go through all the files and find the .pcd files
							directory_iterator itr3(itr2->path()), eod;
							BOOST_FOREACH(path const &p, std::make_pair(itr3, eod)){ 
								if(is_regular_file(p)){
									// Get the dft.txt file
									if(p.extension() == ".txt"){
										//cout << p.stem();
										string filePath = itr2->path().string() + "/" + p.filename().string();
										//ROS_INFO("Filepath: %s", filePath.c_str());
										
										// Read in the file
										ifstream infile(filePath.c_str());
										string line;
										int num_of_lines = 0; 
										
										// Count the number of rows in the file as the columns stay 65 always
										for(int i=0; getline(infile, line); ++i){
											num_of_lines++;
										}
				
										//ROS_INFO("Number of lines: %d", num_of_lines);
										
										ifstream infile2(filePath.c_str());
										while(infile2.good()){
											string str_value, newline;
											for(int i=0; i<num_of_lines; i++){
												getline(infile2, newline, '\n');
												//ROS_INFO("value: %s",newline.c_str());
												std::istringstream buffer(newline);
												vector <double> dft_row;
												for(int j=0; j<65; j++){
													getline(buffer, str_value, ',');
													//ROS_INFO("value: %s",str_value.c_str());
													double datapoint = atof(str_value.c_str());
													//ROS_INFO("datapoint: %f", datapoint);
													dft_row.push_back(datapoint);
												}
												dft_data.push_back(dft_row);
											}
										}
										
										// Calcuate the sum of all the points
										double total_sum;
										for(int i=0; i<num_of_lines; i++){
											for(int j=0; j<65; j++){
												total_sum = total_sum + dft_data[i][j]; 
											}
										}
										ROS_INFO("Total sum: %f",total_sum);
										
										// Calculate the mean of all the points
										mean = total_sum/(num_of_lines*65);
										ROS_INFO("Mean: %f",mean);
										
										for(int i=0; i<num_of_lines; i++){
											vector <double> difference_row;
											for(int j=0; j<65; j++){
												double squared_difference = pow((dft_data[i][j] - mean), 2);
												difference_row.push_back(squared_difference);
											}
											distance_from_mean.push_back(difference_row);
										}
										
										double sum;
										for(int i=0; i<num_of_lines; i++){
											for(int j=0; j<65; j++){
												sum = sum + distance_from_mean[i][j]; 
											}
										}
										
										// Calculate the variance
										variance = sum/(num_of_lines*65);
										//ROS_INFO("Variance: %f",variance);
										
										// Calculate the standard deviation
										standard_deviation = sqrt(variance);
										ROS_INFO("Standard deviation: %f",standard_deviation);
										
										// Calculate the threshold below which to make the datapoints to be zero in the dft
										threshold = mean + 2*standard_deviation;
										ROS_INFO("Threshold: %f",threshold);
									}
								}
							}
						}
					}
				}
			}
		}
	}
	
	return 0;
}
