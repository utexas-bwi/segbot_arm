// Run the server in the following way 
// Run the publisher first --> rosrun grounded_logging audio_publisher
// rosrun grounded_logging audio_logging_server
// rosservice call <arg1> <arg2> <arg3>

#include "ros/ros.h"
#include "ros/time.h"
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
#include <boost/lexical_cast.hpp>

using namespace std;

//For writing to the .wav file
const int format = SF_FORMAT_WAV | SF_FORMAT_FLOAT;        // SF_FORMAT_FLOAT or SF_FORMAT_PCM_16 depending on what you are storing
const int channel = 1;        								// 1-Mono and 2-Stereo
const static int SAMPLE_RATE = 44100;						// Frequency of the microphone
const static int BUFF_SIZE = 128;

#define NFFT 128	//number of frequency bins in FFT, must be a power of 2
#define FFT_WIN 128	//length of window used to compute FFT (in terms of number of raw signal samples)
#define NOVERLAP 64	//by how much the windows should overlap - typically, half of FFT_WIN

bool g_caught_sigint = false;

//array to store the raw data
std::vector<float> sampleData;
//matrix to store the fft
std::vector< std::vector<double> > fftData;
//strings to store the filenames
string wavFileName;
string outputDftFileName;
int c = 0;
int num_fft_columns = 0;
FFT_calculator* fft_calc;
time_t ltime;   

snd_pcm_t *capture_handle;

// should start recording or not				
bool recording_samples;          

// function to handle Ctrl-C
void sig_handler(int sig)
{
	g_caught_sigint = true;
	snd_pcm_close (capture_handle);
	exit (0);
};

// function to return the timestamp in local time
/*std::string get_time_stamp(){
	ltime = time(NULL);        // get current calendar time
	string timeStamp = asctime(localtime(&ltime));
	// Replace spaces, colons and slashes as they are not allowed in filenames
	for(int i = 0; i<timeStamp.length(); ++i){
        if (timeStamp[i] == '/' || timeStamp[i] == ':' || timeStamp[i] == '\n')
            timeStamp[i] = '-';
        if (timeStamp[i] == ' ')
			timeStamp[i] = '_';
    }
    return timeStamp;
}*/

// callback function to process the published audio msgs
void listen_audio_data(const std_msgs::Float64MultiArrayConstPtr& msg){
	if(recording_samples == true){
		for(unsigned int i=0;i<msg->data.size();i++){
			sampleData.push_back((float)msg->data[i]);
		}
		
		for (unsigned int j = 0; j < msg->data.size(); j++){	
			if (fft_calc->add_sample_RT((double)msg->data[j])){
				//get last sample
				vector<double> next_fft = fft_calc->get_last_fft_column();			
				fftData.push_back(next_fft);
				num_fft_columns++;
			}
			c++;
		}
		printf("Listened for %i samples and %i fft columns...\n",c,num_fft_columns);
	}
}

//in the callback function, if recording_samples is true, then add the samples to the vector that we store
bool audio_service_callback(grounded_logging::ProcessAudio::Request &req, 
							grounded_logging::ProcessAudio::Response &res){	
	if (req.start == 1){
		recording_samples = true;
		
		//also store the filenames that are in the request
		wavFileName = req.outputRawFilePath;
		outputDftFileName = req.outputDftFilePath;
		
		//get the start time of recording
		double begin = ros::Time::now().toSec();
		string startTime = boost::lexical_cast<std::string>(begin);
		
		// append start timestamp with filenames
		wavFileName.append("_"+startTime+".wav");
		outputDftFileName.append("_"+startTime+".txt");
		ROS_INFO("Filename1: %s", wavFileName.c_str());
		ROS_INFO("Filename2: %s", outputDftFileName.c_str());
		
		//clear out results
		sampleData.clear();
		fftData.clear();
		c = 0;
		num_fft_columns = 0;
	}
	else {
		//set a flag to stop recording
		recording_samples = false;
			
		//save the fft data to the file
		std::ofstream outputDftFile(outputDftFileName.c_str());
		if(!outputDftFile.is_open()){
			std::cout<< "Could not open the file to store\n";
			return -1;
		}
		for (unsigned int k=0; k < fftData.size(); k++){
			for(unsigned int l=0; l<fftData[k].size(); l++){
				if(l == fftData[k].size()-1)
					outputDftFile << fftData[k].at(l) << endl;
				else
					outputDftFile << fftData[k].at(l) << ",";
			}
		}
		
		//save the raw data to the file
		SndfileHandle outputRawFile(wavFileName.c_str(), SFM_WRITE, format, channel, SAMPLE_RATE);
		if(not outputRawFile){
			("Could not create the .wav file");
			return -1;
		}
		//convert vector to array
		float *data = &sampleData[0];
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
	
	//to store the topic to publish to
	string audio_samples_;
	
	// Intialize FFT
	fft_calc = new FFT_calculator(NFFT,FFT_WIN,NOVERLAP);	
	fft_calc->init_RT_mode();	
	
	//get the topic from the launch file
	nh.param<std::string>("audio_samples", audio_samples_, "/audio_samples");
		
	//Set up the service
	ros::ServiceServer service = nh.advertiseService("audio_logger_service", audio_service_callback);
	
	//subscribe to audio topic
	ros::Subscriber sub = nh.subscribe (audio_samples_, 1000, listen_audio_data);
	
	//register ctrl-c
	signal(SIGINT, sig_handler);
	
	ROS_INFO("Ready to record and process audio data.");
	ros::spin();
	
	return 0;
}

