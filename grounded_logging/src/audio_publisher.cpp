#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Float64MultiArray.h"
#include <stdio.h>
#include <iostream>
#include <fstream>
#include <stdlib.h>
#include <alsa/asoundlib.h>                         //Have to install libasound2-dev
#include "include/alsa_utils.h"

using namespace std;
	      
const static int SAMPLE_RATE = 44100; 
const static int BUFF_SIZE = 128;
const static int PUBLISH_THRESHOLD = 4410;      //So as to publish it 10 times a second
	      
#define NFFT 128	//number of frequency bins in FFT, must be a power of 2
#define FFT_WIN 128	//length of window used to compute FFT (in terms of number of raw signal samples)
#define NOVERLAP 64	//by how much the windows should overlap - typically, half of FFT_WIN

bool g_caught_sigint = false;

//data for audio samples
std::vector<float> sampleData;

snd_pcm_t *capture_handle;
     
main (int argc, char *argv[])
	{
		// Initialize ROS
		ros::init (argc, argv, "audio_sample_publisher");
		ros::NodeHandle nh;
	    
	    ros::Publisher pub = nh.advertise<std_msgs::Float64MultiArray>("/audio_samples", 10);
		ROS_INFO("Ready to publish audio data.");
	
		//open a handle to the microphone
		capture_handle = open_mic_capture("hw:2,0",SAMPLE_RATE);

		//read from the microphone
		int i,j;
		int err;
		short buf[128];
		
		while (ros::ok()){
			if ((err = snd_pcm_readi (capture_handle, buf, BUFF_SIZE)) != BUFF_SIZE) {
				fprintf (stderr, "read from audio interface failed (%s)\n",snd_strerror (err));
				exit (1);
			}
			else {
				for (j = 0; j < BUFF_SIZE; j++){
					//FFT->add_sample_RT((float)buf[j]);
					
					float sample_j = (float)buf[j]/(float)32768.0;
					sampleData.push_back(sample_j);
					
					if((int)sampleData.size() == PUBLISH_THRESHOLD){
						std_msgs::Float64MultiArray msg;
						for(unsigned int i=0; i < sampleData.size(); i++){
							msg.data.push_back(sampleData[i]);
						}
						pub.publish(msg);
						sampleData.clear();
					}
		    	}
			}
		}
		snd_pcm_close (capture_handle);
		exit (0);
}
