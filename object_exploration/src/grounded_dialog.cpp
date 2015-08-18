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
#include "bwi_msgs/QuestionDialog.h"
#include "ros/ros.h"
#include <string>
#include <vector>
#include <stdlib.h>
#include <stdio.h>

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
const int img_width 		= 95; 		//pixels
const int img_height 		= 127; 		//pixels
const float aspect_ratio 	= .75;
const int border_size 		= 5; 		//pixels
const int abs_width 		= 1024; 	//pixels
const int abs_height 		= 384; 		//pixels
const int title_height		= 12;		//pixels
using namespace cv;

Mat dst,frame,img,ROI;
ros::ServiceClient gui_client;

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
 */
bool ask_mult_choice(std::string question){
	bool response = false;
	bwi_msgs::QuestionDialog srv;

	srv.request.type = 1;
	srv.request.message = question;
	srv.request.timeout = 0.0;
	std::vector<std::string> temp;
	temp.push_back("No");
	temp.push_back("Yes");
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
 * TODO: able to update window/thread without closing the window completely.
 		 resize images once, rather than on-the-fly (for speed)
 */
int writeToScreen(std::vector<int> object_names){
	int num_rows = (object_names.size() / 9) + 1;
	int roi_x, roi_y, text_x, text_y;

	cv::Rect roi(cv::Rect(0,0,img_width, img_height));
	cv::Mat targetROI = dst(roi);
	ROS_INFO("numrows: %d", num_rows);
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
			for(int j = 0; j < object_names.size() % 9; j++){
				int object_num = 1 + j + (i*9);
				std::string str = boost::lexical_cast<std::string>(object_num);
				std::string path ="/home/users/max/Pictures/object_exploration/" + str + ".JPG";
				ROS_INFO("Getting %s", path.c_str());
				Mat src = imread(path);
				Mat img;
				if(!src.data)
					ROS_INFO("Couldn't get image data");
				resize(src,img,Size(img_width, img_height));
				ROS_INFO("Placing image %d at ROI (%d,%d)", object_num,roi_x,roi_y);
				ROS_INFO("Placing text %d at (%d,%d)", object_num,text_x,text_y);
				if(object_num > 9){ //if a double digit number, center text
					text_x -= 6;
				}
				cv::putText(dst, boost::lexical_cast<std::string>(object_num),cv::Point(text_x,text_y), CV_FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255,255,255,0),1,8,false);
				targetROI = dst(cv::Rect(roi_x,roi_y,img.cols, img.rows));
				img.copyTo(targetROI);
				roi_x += img_width;
				text_x += img_width;
				targetROI = dst(cv::Rect(roi_x,roi_y,img.cols, img.rows));
				roi_x += border_size;
				text_x += border_size;
			}
		} else{
			for(int j = 1; j <= 9; j++){
				std::string str = boost::lexical_cast<std::string>(j + (i*9));
				std::string path ="/home/users/max/Pictures/object_exploration/" + str + ".JPG";
				ROS_INFO("Getting %s", path.c_str());
				Mat src = imread(path);
				Mat img;
				if(!src.data)
					ROS_INFO("Couldn't get image data");
				resize(src,img,Size(img_width, img_height));
				ROS_INFO("Placing image %d at ROI (%d,%d)", j,roi_x,roi_y);
				cv::putText(dst, boost::lexical_cast<std::string>(j + (i*9)),cv::Point(text_x,text_y), CV_FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255,255,255,0),1,8,false);
				targetROI = dst(cv::Rect(roi_x,roi_y,img.cols, img.rows));
				img.copyTo(targetROI);
				roi_x += img_width;
				text_x += img_width;
				targetROI = dst(cv::Rect(roi_x,roi_y,img.cols, img.rows));
				roi_x += border_size;
				text_x += border_size;
			}
			roi_y += img_height;
		}
	}
	cv::namedWindow("OpenCV Window");
	// show the image on window
	while(true){
		cv::imshow("OpenCV Window", dst);
		cv::waitKey(0);
		break;
	}

}

/*
 * To be the script & main loop for the question-answer interface logic
 */

void sequence(std::vector<int> photo_temp){
	bool askedSharedAtt = false;

	boost::thread workerThread(writeToScreen, photo_temp);

	bool common_att = ask_mult_choice("Do any shown objects share a common attribute?");
	if(common_att){ //user input 'Yes' to "Any attributes common to all objects?"
		//ask only once per tree : "Can you specify that attribute?"
		if(!askedSharedAtt){
			askedSharedAtt = true;
			std::string resp = ask_free_resp("Please specify what attribute is shared");
			//parse resp, checking if each attribute exists in table
			/*

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
	}
	print_to_gui("Thank you for clearing that up!");
}

int main (int argc, char **argv){
	ros::init(argc, argv, "grounded_dialog_gui");
	ros::NodeHandle n;

	gui_client = n.serviceClient<bwi_msgs::QuestionDialog>("question_dialog");
	bwi_msgs::QuestionDialog srv;
	dst = cv::Mat(abs_height, abs_width, CV_8UC3, cv::Scalar(0,0,0));

	sleep(2);
	//simulated vector recieved from clustering alg.
	std::vector<int> photo_temp;
	for(int i = 1; i <= 10; i++)
		photo_temp.push_back(i);

	sequence(photo_temp);

  return(0);
}