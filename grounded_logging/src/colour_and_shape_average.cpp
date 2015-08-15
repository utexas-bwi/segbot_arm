#include "ros/ros.h"
#include "std_msgs/String.h"
#include <math.h>  
#include <fstream>
#include <iostream>
// For traversing the filesystem
#include <boost/filesystem.hpp>
#include <boost/foreach.hpp>
#include <boost/tokenizer.hpp>

using namespace std;
using namespace boost::filesystem;

// The general file path
std::string generalFilePath = "/home/users/pkhante/extracted_feature_vectors/";

//Name of the audio folder to search for
const std::string & color_folder = "look_shape";

//Total number of objects and trials for ease of file traversal
int total_objects = 32;

// Vector to store the file data
vector<string> fileData;

// Vector to put in the matrix of each object
vector<vector<double> > objectData;

// Matrix to store the averaged values for each object
vector<double> averaged_values;


int main (int argc, char** argv)
{
	// Initialize ROS
	ros::init (argc, argv, "color_and_shape_average");
	ros::NodeHandle nh;
	ROS_INFO("Ready to average features vectors across the 6 trials...");
	
	path dir_path(generalFilePath);
	directory_iterator end_itr;       // default construction yields past-the-end
	for(directory_iterator itr(dir_path); itr != end_itr; ++itr){
		if(itr->path().filename().string() == color_folder){
			directory_iterator end_itr2;       // default construction yields past-the-end
			//for(directory_iterator itr2(itr->path()); itr2 != end_itr2; ++itr2){
				//string folder_name = itr2->path().filename().string();
				//Go through all the files and find the audio files
				directory_iterator itr2(itr->path()), eod;
				BOOST_FOREACH(path const &p, std::make_pair(itr2, eod)){ 
					if(is_regular_file(p)){
						// Get the dft.csv file
						if(p.extension() == ".csv"){
							//cout << p.stem();
							string filePath = itr2->path().string();
							ROS_INFO("Filepath: %s", filePath.c_str());
							
							// Read the contents of the file into a vector
							ifstream infile(filePath.c_str());
							if (infile.is_open())
							{
								string line;
								while(getline(infile, line)){
									fileData.push_back(line);
								}
							}
							
							typedef boost::escaped_list_separator<char> Separator;
							typedef boost::tokenizer<Separator> Tokenizer;
							
							//ROS_INFO("Beginning: Size of object_data matrix: %d", (int)objectData.size());
							
							// Fill the trial_data matrix
							int counter = 0;
							for(int object_num = 0; object_num < total_objects; object_num++){
								int add = 6 * counter;
								for(int i=0; i<6; i++){
									Tokenizer tokenizer(fileData[i+add]);
									int count = 0;
									vector<double> data_row;
									for (Tokenizer::iterator iter = tokenizer.begin(); (iter != tokenizer.end()) && (count < 309); ++iter)
									{
										if ((count >= 1) & (count <=308))
										{
											stringstream sstr(*iter);
											double value;
											while(sstr >> value){
												data_row.push_back(value);
											}
										}
										count++;
									}
									objectData.push_back(data_row);
								}
								
								//ROS_INFO("Object data value: %f", objectData[0][0]);
								
								//Compute average
								for(int k=0; k<308; k++){
									double value = 0;
									for(int m=0; m<6; m++){	
										value = value + objectData[m][k];
									}
									double average = value/6;
									averaged_values.push_back(average);
								}
				
								//ROS_INFO("Size of average_values matrix: %d", (int)averaged_values[0].size());
								
								// Write the averaged_value vector to a file
								string file_to_write = generalFilePath+"look_shape/averaged_shape_vectors.txt";
								ROS_INFO("File to write path: %s", file_to_write.c_str());
								ofstream audio_file(file_to_write.c_str(), std::ios::out | std::ios::app);
								ROS_INFO("Writing to the file...");
								if(audio_file.is_open()){
									std::stringstream convert;
									convert << object_num+1;
									audio_file << "obj_" + convert.str();
									audio_file << ",";
									for (int y=0; y<308; y++){
										audio_file << averaged_values[y];
										if(y==307)
											audio_file << "\n";
										else
											audio_file << ",";
									}
									audio_file.close();
									ROS_INFO("File saved");
								}
								else 
									ROS_ERROR("Cannot open file");
								
								counter++;
								averaged_values.clear();
								
								// Clear out the data for the next object
								objectData.clear();	
							}
							fileData.clear();
						}
					}
				}
			}
		}
	
	return 0;
}
