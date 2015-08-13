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

using namespace cv;

Mat dst,frame,img,ROI;


int writeToScreen(std::vector<int> object_names){
	int num_rows = (object_names.size() / 9) + 1;
	int roi_x, roi_y;

    cv::Rect roi(cv::Rect(0,0,img_width, img_height));
    cv::Mat targetROI = dst(roi);
    ROS_INFO("numrows: %d", num_rows);
	for(int i = 0; i < num_rows; i++){
		roi_y += border_size;
		roi_x += border_size;
		if((i+1) == num_rows){ //output non standard no. images
			for(int j = 0; j < object_names.size() % 9; j++){
				std::string str = boost::lexical_cast<std::string>(1+ j + (i*9));
				std::string path ="/home/users/max/Pictures/" + str + ".png";
				ROS_INFO("Getting %s", path.c_str());
				Mat src = imread(path);
				Mat img;
				if(!src.data)
					ROS_INFO("Couldn't get image data");
				resize(src,img,Size(img_width, img_height));
    			targetROI = dst(cv::Rect(roi_x,roi_y,img.cols, img.rows));
				img.copyTo(targetROI);
				roi_x += img_width;
    			targetROI = dst(cv::Rect(roi_x,roi_y,img.cols, img.rows));
    			roi_x += border_size;
			}
		}
		else{
			for(int j = 0; i < 9; i++){
				//
			}
		}
	}
	cv::namedWindow("OpenCV Window");
    // show the image on window
    cv::imshow("OpenCV Window", dst);
    cv::waitKey(1000);

}

int showImage(){
    Mat src1 = imread("/home/users/max/Pictures/temp.png");//, CV_LOAD_IMAGE_COLOR);   // Read the file
    Mat src2 = imread("/home/users/max/Pictures/temp2.png");//, CV_LOAD_IMAGE_COLOR);   // Read the file
	Mat image1, image2;
	int dstWidth = 900;
	int dstHeight = 600;
	resize(src1,image1,Size(dstWidth/2,dstHeight/2));
	resize(src2,image2,Size(dstWidth/2,dstHeight/2));

    cv::Rect roi(cv::Rect(0,0,image1.cols, image1.rows));
    cv::Mat targetROI = dst(roi);
    image1.copyTo(targetROI);
    targetROI = dst(cv::Rect(0,image1.rows,image1.cols, image1.rows));

    // create image window named "My Image"
    cv::namedWindow("OpenCV Window");
    // show the image on window
    cv::imshow("OpenCV Window", dst);
    // wait key for 5000 ms
    cv::waitKey(1000);
    image2.copyTo(targetROI);
    cv::imshow("OpenCV Window", dst);
    cv::waitKey(1000);


}

int main (int argc, char **argv){
  ros::init(argc, argv, "grounded_dialog_gui");
  ros::NodeHandle n;

  ros::ServiceClient gui_client = n.serviceClient<bwi_msgs::QuestionDialog>("question_dialog");
  bwi_msgs::QuestionDialog srv;
  dst = cv::Mat(abs_height, abs_width, CV_8UC3, cv::Scalar(0,0,0));
  std::vector<int> photo_temp;
  photo_temp.push_back(1);
  photo_temp.push_back(2);

  if(1){
  	srv.request.type = 1;
  	srv.request.message = "Does this work?";
  	srv.request.timeout = 0.0;
  	std::vector<std::string> temp;
  	temp.push_back("Yes");
  	temp.push_back("No");
  	srv.request.options = temp;
  	//showImage();
  	writeToScreen(photo_temp);
  	if(gui_client.call(srv)){
  		ROS_INFO("Hey, I got a response: %s", srv.response.text.c_str());
  	}
  }

  return(0);
}