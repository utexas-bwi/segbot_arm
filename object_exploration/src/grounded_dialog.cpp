/*
 * Question srv contents
	 # Constants defining question type
	int32 DISPLAY = 0
	int32 CHOICE_QUESTION = 1
	int32 TEXT_QUESTION = 2
	# Constants regarding timeout
	float32 NO_TIMEOUT=0.0
	# Constants for response index
	int32 NO_RESPONSE=-1
	int32 TIMED_OUT=-2
	int32 TEXT_RESPONSE=-3
	int32 PREEMPTED=-4
	int32 type
	string message
	string[] options # used in choice questions
	float32 timeout
	---
	int32 index
	string text
 *
 */


#include <std_msgs/String.h>
#include <std_msgs/Int32.h>
#include <boost/lexical_cast.hpp>
#include <boost/thread.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>
#include "bwi_msgs/QuestionDialog.h"
#include "ros/ros.h"
#include <string>
#include <vector>
#include <stdlib.h>
#include <stdio.h>
#include <map>
#include <iterator>
#include <sstream>

#include <sys/stat.h>
#include <iostream>
#include <fstream>

#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"

/* 
 * size constants for the size of a single icon
 * If sizes end up being scaled based on number of images, these represent minimums
 *
 * Optimized for Marvin's laptop screen, which is 1024x768 meaning if half the vertical
	realestate is used for images:
		9 images across
		3 images down
 */
const int img_width 			= 95; 		//pixels
const int img_height 			= 127; 		//pixels
const float aspect_ratio 		= 0.75;
const int border_size 			= 5; 		//pixels
const int abs_width 			= 1500; 	//pixels
const int abs_height 			= 450; 		//pixels
const int title_height			= 12;		//pixels
const int images_row			= round(abs_width / (img_width + border_size));
const std::string filePath		= "/home/users/max/";
const std::string responseName	= "groundedResponse.txt";
const std::string requestName	= "groundedRequest.txt";

std::map<std::string, std::vector<std::string> > label_table;
int clusterNum;
std::string clusterAttribute;
static std::vector<std::string> cur_cluster;
using namespace cv;

Mat dst,frame,img,ROI;
ros::ServiceClient gui_client;


/*
 * Returns whether or not response file exists
 */
bool responseFileExists(){
	struct stat buffer;
	std::string fullPath = filePath + responseName;
	return (stat (fullPath.c_str(), &buffer) == 0);
}

/*
 * Writes request file
 * ID: 0 = remove -> name of object, its label
 		1 = get next cluster -> current cluster number, its label
 		2 = recluster -> no other arguments

 	Returns success

 	Input is ID and a vector, which can be null if the ID specifies an action not related to object IDS
 */
bool writeRequestFile(int ID, std::vector<std::string> objects){
	std::ofstream myfile((filePath + requestName).c_str());
	if(myfile.is_open()){
		myfile << ID << "\n";
		if(ID == 0){
			ROS_INFO("Requesting object removal");
			for(int i = 0; i < objects.size(); i++){
				myfile << objects[i] << "\n";
			}
			//print labels
		}
		else if(ID == 1){
			ROS_INFO("Requesting next cluster");
			myfile << clusterNum << "\n";
			//myfile << label << "\n";
		}
		else
			ROS_INFO("Requesting a recluster");
		myfile.close();
	}
	else {
		ROS_INFO("Unable to create file");
		return false;
	}
	return true;
}

/*
 * Reads the response file from external program
 * IDs : N/A
 * Expected format: 
 	behavior_modality
 	cluster_num
 	object names
 */

bool readResponseFile(){
	std::string line;
	int lineNum = 0;
	int ID;
	std::vector<std::string> objects;
	std::string fullPath = (filePath + responseName);
	std::ifstream myfile(fullPath.c_str());
	if(myfile.is_open()){
		while(getline(myfile,line)){
			if(lineNum == 0){
				//modality
			}
			else if(lineNum == 1)
				clusterNum = atoi(line.c_str());
			else{
				objects.push_back(line.c_str());
			}
			lineNum++;
		}
		myfile.close();
	}
	else{
		ROS_INFO("Unable to open response file");
		return false;
	}
	cur_cluster = objects;


	if( remove( fullPath.c_str()) != 0 )
    	ROS_ERROR( "Error deleting response file!" );
	else
		ROS_INFO("Response file parsed and deleted successfully.");
	return true;
}

/*
 * General method to send free response questions to the GUI
 */
std::string ask_free_resp(std::string question){
	bwi_msgs::QuestionDialog srv;

	srv.request.type = 2;
	srv.request.message = question;
	srv.request.timeout = 0.0;

	if(gui_client.call(srv)){
		ROS_INFO("Hey, I got a response");
		return srv.response.text;
	} else {
		ROS_INFO("Something went wrong");
	}
	return NULL;
}

/*
 * General method for sending multiple choice questions to the GUI
 * Returns whether or not the answer was yes to choice2
 */
bool ask_mult_choice(std::string question, std::string choice1, std::string choice2){
	bool response = false;
	bwi_msgs::QuestionDialog srv;

	srv.request.type = 1;
	srv.request.message = question;
	srv.request.timeout = 0.0;
	std::vector<std::string> temp;
	temp.push_back(choice1);
	temp.push_back(choice2);
	srv.request.options = temp;
	//boost::thread workerThread(writeToScreen, photo_temp);
	//writeToScreen(photo_temp);
	if(gui_client.call(srv)){
		response = srv.response.index == 0 ? false : true;
		ROS_INFO("Hey, I got a response: %d", response);
	} else {
		ROS_INFO("Something went wrong");
	}
	return response;
}

/*
 * General method for displaying messages to the GUI
 */
 void print_to_gui(std::string message){
 	bwi_msgs::QuestionDialog srv;

	srv.request.type = 0;
	srv.request.message = message;
	srv.request.timeout = 0.0;
	if(gui_client.call(srv)){
		ROS_INFO("Outputing message");
	} else {
		ROS_INFO("Something went wrong outputting the message to gui");
	}
 }

/*
 * Function uses input structures as a map for displaying images.
 * Images are displayed on half the screen.
 *
 * For concurrent windows (GUI and output of this function), must run this on a separate thread.
 *
 */

int writeToScreen(std::vector<std::string> *object_names){
	int roi_x, roi_y, text_x, text_y;
	cv::namedWindow("OpenCV Window");

	while(true){
		std::vector<std::string>& cluster = *object_names;
		int num_rows = (cluster.size() / images_row) + 1;
		cv::Rect roi(cv::Rect(0,0,img_width, img_height));
		cv::Rect backgrnd(cv::Rect(0,0,abs_width, abs_height));
		cv::Mat targetROI = dst(backgrnd);
		targetROI = cv::Scalar(0,0,0);
		targetROI = dst(roi);

		ROS_INFO("numrows: %d", images_row);
		roi_x = 0;
		roi_y = 0;
		text_x = 0;
		text_y = title_height;
		for(int i = 0; i < num_rows; i++){
			roi_y += border_size + title_height + border_size;
			text_y = roi_y - title_height/2;
			roi_x = border_size;
			text_x = border_size + (img_width/2);
			
			if((i+1) == num_rows){ //output non standard no. images
				roi_x = border_size;
				ROS_INFO("Placing a non-standard row of images");
				for(int j = 0; j < cluster.size() % images_row; j++){
					int object_num = j + (i*images_row);
					std::string str = boost::lexical_cast<std::string>(object_num);
					std::string path ="/home/maxwell/Pictures/object_exploration/cropped/" + cluster.at(object_num) + ".JPG";
					ROS_INFO("Getting %s", path.c_str());
					Mat src = imread(path);
					Mat img;
					if(!src.data)
						ROS_INFO("Couldn't get image data");
					resize(src,img,Size(img_width, img_height));
					ROS_INFO("Placing image %d at ROI (%d,%d)", object_num,roi_x,roi_y);
					ROS_INFO("Placing text %d at (%d,%d)", object_num,text_x,text_y);
					if(object_num > images_row){ //if a double digit number, center text
						text_x -= 6;
					}
					cv::putText(dst, boost::lexical_cast<std::string>(object_num),cv::Point(text_x,text_y), CV_FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255,255,255,0),1,8,false);
					targetROI = dst(cv::Rect(roi_x,roi_y,img.cols, img.rows));
					img.copyTo(targetROI);
					roi_x += img_width;
					text_x += img_width;
					targetROI = dst(cv::Rect(roi_x,roi_y,img.cols, img.rows));
					roi_x += border_size;
					text_x += border_size + border_size;
				}
			} else{
				for(int j = 0; j <= images_row; j++){
					int object_num = j + (i*images_row);
					std::string str = boost::lexical_cast<std::string>(object_num);
					std::string path ="/home/users/max/Pictures/object_exploration/" + cluster.at(object_num) + ".JPG";
					ROS_INFO("Getting %s", path.c_str());
					Mat src = imread(path);
					Mat img;
					if(!src.data)
						ROS_INFO("Couldn't get image data");
					resize(src,img,Size(img_width, img_height));
					ROS_INFO("Placing image %d at ROI (%d,%d)", object_num,roi_x,roi_y);
					ROS_INFO("Placing text %d at (%d,%d)", object_num,text_x,text_y);

					cv::putText(dst, boost::lexical_cast<std::string>(object_num),cv::Point(text_x,text_y), CV_FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255,255,255,0),1,8,false);
					targetROI = dst(cv::Rect(roi_x,roi_y,img.cols, img.rows));
					img.copyTo(targetROI);
					if(j != images_row){
						roi_x += img_width;
						text_x += img_width;
						targetROI = dst(cv::Rect(roi_x,roi_y,img.cols, img.rows));
						roi_x += border_size;
						text_x += border_size;
					}
				}
				roi_y += img_height;
		}
	}

	// show the image on window

		cv::imshow("OpenCV Window", dst);
		cv::waitKey(3000);
		//break;
	}

}

/*
 * Breaks input string delimited by ' ' (spaces) and stuffs a vector
 */
std::vector<std::string> splitString(std::string input){
	std::vector<std::string> vec;
	std::istringstream iss(input);
	copy(std::istream_iterator<std::string>(iss), std::istream_iterator<std::string>(),
		back_inserter(vec));
	return vec;
}

/*
 * To be the script & main loop for the question-answer interface logic
 */

void sequence(std::vector<std::string> photo_temp){
	bool firstTime;

	boost::thread workerThread(writeToScreen, &cur_cluster);
	while(true){
		firstTime = false;
		bool common_att = ask_mult_choice("Do any shown objects share a common attribute?", "No", "Yes");
		if(common_att){ //user input 'Yes' to "Any attributes common to all objects?"
			//ask only once per tree : "Can you specify that attribute?"
			if(!firstTime){
				firstTime = true;
				std::string resp = ask_free_resp("Please specify what attribute is shared");

				//parse resp, checking if each attribute exists in table
				std::vector<std::string> feature_vec = splitString(resp);
				clusterAttribute = feature_vec.at(0);

				for(int i = 0; i < feature_vec.size(); i++){ //check if attributes are consistent with existing labels
					std::map<std::string, std::vector<std::string> >::iterator it;
					it = label_table.find(feature_vec.at(i));

					if(it != label_table.end()){
						std::vector<std::string> values = it->second;
						std::vector<std::string> values_buffer; //used to track new labels to be added to values
						for(int j = 0; j < values.size(); j++){
							int mult_choice_ans = ask_mult_choice("Are they " + values.at(j) + " in " + feature_vec.at(i),
									"No", "Yes");
							if(mult_choice_ans)
								values_buffer.push_back(values.at(j));
						}
					}
					else {
						ROS_INFO("Attribute not found in table.");
						std::string att_from_above = ask_free_resp("What " + feature_vec.at(i) + " are they?");
						label_table.insert(std::pair<std::string,std::vector<std::string> >(feature_vec.at(i),
							splitString(att_from_above)));
					}
				}

				/*
				for reference, the algorithm coded above:

				parse here

				if(contained in table)
					grab labels as values, using attribute as key
					for each label
						int answer = ask_mult_choice("Are they " + <label> + " in " + attribute)
						if(answer)
							put(label)
				else
					std::string answer = ask_free_resp("What <att-from-above> are they?")
					put(answer)  as label
				*/
			}
		}
		else{
			if(ask_mult_choice("Is there any attribute common to most of the objects?", "No", "Yes")){
				if(!firstTime){
					firstTime = true;
					std::string resp = ask_free_resp("Please specify what attribute is shared");
					std::vector<std::string> feature_vec = splitString(resp);
					clusterAttribute = feature_vec.at(0);
				}
				if(ask_mult_choice("How many objects don't fit the attribute?", ">2", "1 or 2")){
					std::vector<std::string> outliers = splitString(
							ask_free_resp("Please specify the names of the outlier(s), separated by spaces"));
					for(int i = 0; i < outliers.size(); i++){
						//display only outliers.at(i);
						std::string answer = ask_free_resp("What is the attribute of this object?");
						//store answer as label
						//store in outlier map to be combined to any cluster with the same label
					}
					/*writeRequestFile(0,outliers);	//send request to Java prog
					while(!responseFileExists){		//wait for Java to respond with updated cluster
						sleep(.01);
					}
					readResponseFile();
					*/
					cur_cluster.clear();
					cur_cluster.push_back("tin_can");
					sleep(3); //waits for the cv window to update
				}
			}
			else{
				//writeRequestFile(2,NULL); //recluster
			}
		}
			/*
			else
				int answer = ask_mul_choice("Is there any attribute common to most of the objects?")
				if(answer)
					//weird arrow on tree, ask about this
					int answer = ask_mult_choice("How many objects don't fit the attribute?", "1 or 2", ">2")
					if(answer)
						recluster()
					else
						std::string answer = ask_free_resp("Please specify the object numbers of the outliers")
						for each OBJECT : answer
							display only object number OBJECT
							std::string answer = ask_free_resp("What is the attribute of this object?")
							store answer as a label for object
						display previous cluster sans all outliers
						std::string answer = ask_free_resp("What <attr-from-above> are they?")
						store answer as label for cluster
				else
					recluster()
			*/
		
		print_to_gui("Thank you for clearing that up!");

		//get next cluster
		/*writeRequestFile(1,NULL);
		while(!responseFileExists){		//wait for Java to respond with updated cluster
				sleep(.01);
		}
		readResponseFile();				//grab next cluster if successful
		*/
	}
}

int main (int argc, char **argv){
	ros::init(argc, argv, "grounded_dialog_gui");
	ros::NodeHandle n;

	gui_client = n.serviceClient<bwi_msgs::QuestionDialog>("question_dialog");
	bwi_msgs::QuestionDialog srv;
	dst = cv::Mat(abs_height, abs_width, CV_8UC3, cv::Scalar(0,0,0));

	sleep(2); //must wait for gui to initizalize

	//simulated vector recieved from clustering alg.
	std::vector<std::string> photo_temp;
	photo_temp.push_back("big_red_pop_can");
	photo_temp.push_back("blue_salt_can");

	

	cur_cluster = photo_temp;
	sequence(photo_temp);
	return(0);
}