// Run the server in the following way 
// Run the publisher first --> rosrun grounded_logging audio_publisher
// rosrun grounded_logging audio_logging_server
// rosservice call <arg1> <arg2>

#include "ros/ros.h"
#include "std_msgs/String.h"
#include <stdio.h>
#include <signal.h> 
#include <iostream>
#include <fstream>
#include <stdlib.h>
#include "std_msgs/Float64MultiArray.h"
#include "grounded_logging/ProcessAudio.h"
#include "include/alsa_utils.h"
#include "include/fft_calculator_dq.h"				//Have to install libfftw3-dev
#include <sndfile.hh>								//Have to install libsndfile1-dev

using namespace std;

//For writing to the .wav file
const int format = SF_FORMAT_WAV | SF_FORMAT_DOUBLE;        // SF_FORMAT_FLOAT or SF_FORMAT_PCM_16 depending on what you are storing
const int channel = 1;        								// 1-Mono and 2-Stereo
const static int SAMPLE_RATE = 44100;						// Frequency of the microphone
const static int BUFF_SIZE = 128;

#define NFFT 128	//number of frequency bins in FFT, must be a power of 2
#define FFT_WIN 128	//length of window used to compute FFT (in terms of number of raw signal samples)
#define NOVERLAP 64	//by how much the windows should overlap - typically, half of FFT_WIN

bool g_caught_sigint = false;

snd_pcm_t *capture_handle;

int i,j;
int err;
short buf[128];
std::vector<double> sampleData;
string wavFileName;

//bool startRecordingReceived;      // do we start recording
//bool stopRecordingReceived;       // do we stop recording
bool recording_samples;          //are we recording or not

void sig_handler(int sig)
{
	g_caught_sigint = true;
	snd_pcm_close (capture_handle);
	exit (0);
};

void listen_audio_data(const std_msgs::Float64MultiArrayConstPtr& msg){
	if(recording_samples == true){
		for(unsigned int i=0;i<msg->data.size();i++){
			sampleData.push_back((double)msg->data[i]);
		}
	}
}

//add a callback function
//in the callback function, if recording_samples is true, then add the samples to the vector that we store
bool audio_service_callback(grounded_logging::ProcessAudio::Request &req, 
							grounded_logging::ProcessAudio::Response &res){	
	if (req.start == 1){
		//startRecordingReceived = true;
		recording_samples = true;
		//also store the filename that was in the request
		wavFileName = req.outputRawFileName;
		// string outputDftFileName = req.outputDftFileName;      ----> Add back when doing DFT
		
		//clear out results
		sampleData.clear();
	}
	else {
		//stopRecordingReceived = true;
		//set a flag for stop recording
		recording_samples = false;
		
		//save the data to the file
		SndfileHandle outputRawFile(wavFileName.c_str(), SFM_WRITE, format, channel, SAMPLE_RATE);
		if(not outputRawFile){
			("Could not create the .wav file");
			return -1;
		}
		//convert vector to array
		double *data = &sampleData[0];
		// write to file
		outputRawFile.write(&data[0], sampleData.size());
	}
	
	res.success = true;
	return true;
}

int main (int argc, char** argv)
{
	// Initialize ROS
	ros::init (argc, argv, "audio_logging_server");
	ros::NodeHandle nh;
		
	//Set up the service
	ros::ServiceServer service = nh.advertiseService("audio_logger_service", audio_service_callback);
	
	//subscribe to audio topic
	ros::Subscriber sub = nh.subscribe ("/audio_samples", 1000, listen_audio_data);
	
	//register ctrl-c
	signal(SIGINT, sig_handler);
	
	ROS_INFO("Ready to record and process audio data.");
	ros::spin();
	
	return 0;
}

