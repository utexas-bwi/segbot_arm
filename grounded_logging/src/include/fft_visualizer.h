/*
*	Class for visualizing a Fast-Fourrir Transform of a signal using OpenCV
*/

#ifndef _fft_viz_
#define _fft_viz_

//some standard C++
#include <vector>
#include <stdio.h>
#include <string>
#include <cstdio>
#include <deque>
#include <vector>
#include <math.h>

//open cv
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#define MAX_BIN_VALUE 0.5

const double weights[6] = {0.0,0.12,0.4,0.6,0.88,1.0};
const double colors[6][3] = { {0.56,0,0}, {1.0,0.0,0.0}, {1.0,1.0,0.0}, {0.0,1.0,1.0}, {0.0,0.0,1.0},{0.21,0.39,0.0}};
double w_temp[6];
double sum;
const int num_colors = 6;

void trackbar_callback( int i, void* t );

class FFT_visualizer {
  friend void trackbar_callback(int i, void *); 
  public:
    FFT_visualizer(int n_fft, int n_time, std::vector< std::vector<double> > result){
      N_FFT=n_fft/2 + 1;
      N_TIME=n_time;
      t_counter = 0; 
      max_bin_value=0.25 ; //to do: should be 0.5 for 64 N_FFT, 1.0 for 32 N_FFT, etc...  
      
      //horizontal axis of image is time, vertical is frequency bins
      fft_img=cvCreateImage(cvSize(N_TIME,N_FFT),IPL_DEPTH_32F,3);
     
      for (int i = 0; i < fft_img->width; i ++){
		for (int j = 0; j < fft_img->height; j++){
		   s.val[0]=0;
		   s.val[1]=0;
		   s.val[2]=255;
		   cvSet2D(fft_img, j, i, s);

		}
      }
      cv::waitKey(10);
      cv::namedWindow("fft");
      cv::imshow("fft",cv::Mat(fft_img));
      
      bar_value=20;
      cv::createTrackbar("threshold","fft",&bar_value,100,trackbar_callback,this);

      cv::waitKey(10);
      
      for (int k = 0; k < result.size(); k++){
		 std::vector<double> next_column;
		 for (int m = 0; m < result[k].size(); m++){
			next_column.push_back(result[k][m]); 
		}
		 update(next_column, 1.0, 1.0, 1.0);
	  }
    };
    
    
    void show_fft(){
      cv::imshow("fft",cv::Mat(fft_img));
      cv::waitKey(10);
    };
     
    void update(std::vector<double> next_column, double blue_max=1.0, double green_max=1.0, double red_max=1.0){
      for (int j = 0; j < fft_img->height; j++){
		double v = (double)next_column[j];
		if (v > max_bin_value){
			v=max_bin_value;
		}
	
	s.val[0]=(double)(blue_max*((v-0.0)/(max_bin_value)));
	s.val[1]=(double)(green_max*((v-0.0)/(max_bin_value)));
	s.val[2]=(double)(red_max*((v-0.0)/(max_bin_value)));
	cvSet2D(fft_img, fft_img->height-1-j, t_counter, s);
	
      }
      
      //ROS_INFO("max = %f",max_bin_value);
      
      t_counter++;
      if (t_counter >= N_TIME)
		t_counter=0;
    };
 

  private:
    int N_FFT, N_TIME;
    IplImage* fft_img;
    
    int t_counter;
    CvScalar s;
    
    double max_bin_value;
    
    int bar_value;
    
    

};

void trackbar_callback( int i, void* t ){
	((FFT_visualizer*)t)->max_bin_value=((double)i/(50))*0.25;
};


#endif
