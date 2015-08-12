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


#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"

using namespace cv;

Mat frame,img,ROI;

int showImage(){
    Mat src1 = imread("/home/users/max/Pictures/temp.png");//, CV_LOAD_IMAGE_COLOR);   // Read the file
    Mat src2 = imread("/home/users/max/Pictures/temp2.png");//, CV_LOAD_IMAGE_COLOR);   // Read the file
	Mat image1, image2;
	int dstWidth = 900;
	int dstHeight = 600;
	resize(src1,image1,Size(dstWidth/2,dstHeight/2));
	resize(src2,image2,Size(dstWidth/2,dstHeight/2));

    cv::Mat dst = cv::Mat(dstHeight, dstWidth, CV_8UC3, cv::Scalar(0,0,0));
    cv::Rect roi(cv::Rect(0,0,image1.cols, image1.rows));
    cv::Mat targetROI = dst(roi);
    image1.copyTo(targetROI);
    targetROI = dst(cv::Rect(0,image1.rows,image1.cols, image1.rows));
    image2.copyTo(targetROI);

    // create image window named "My Image"
    cv::namedWindow("OpenCV Window");
    // show the image on window
    cv::imshow("OpenCV Window", dst);
    // wait key for 5000 ms
    cv::waitKey(1000);
}

int main (int argc, char **argv){
  ros::init(argc, argv, "grounded_dialog_gui");
  ros::NodeHandle n;

  ros::ServiceClient gui_client = n.serviceClient<bwi_msgs::QuestionDialog>("question_dialog");
  bwi_msgs::QuestionDialog srv; 
  if(1){
  	srv.request.type = 1;
  	srv.request.message = "Does this work?";
  	srv.request.timeout = 10.0;
  	std::vector<std::string> temp;
  	temp.push_back("Yes");
  	temp.push_back("No");
  	srv.request.options = temp;
  	showImage();
  	if(gui_client.call(srv)){
  		ROS_INFO("Hey, I got a response: %s", srv.response.text.c_str());
  	}
  }

  return(0);
}