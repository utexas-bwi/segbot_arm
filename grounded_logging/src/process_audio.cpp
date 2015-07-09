// Run the server in the following way
// rosrun grounded_logging process_audio

#include "ros/ros.h"
#include "std_msgs/String.h"
#include <stdio.h>
#include <iostream>
#include <fstream>
#include <stdlib.h>
#include <alsa/asoundlib.h>                         //Have to install libasound2-dev
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
int sampleDuration;
short buf[128];
std::vector<double> sampleData;

void sig_handler(int sig)
{
	g_caught_sigint = true;
	snd_pcm_close (capture_handle);
	exit (0);
};

vector< vector<double> > record_fft_columns(FFT_calculator* fft_calc, std::ofstream& outputDftFile) {
	int num_samples_to_record = (int)((double)SAMPLE_RATE*sampleDuration);
	int c = 0;
	int num_fft_columns = 0;
	int err;
	
	//matrix to store the fft
	vector< vector<double> > result;
	
	while (true){
		if ((err = snd_pcm_readi (capture_handle, buf, BUFF_SIZE)) != BUFF_SIZE) {
			fprintf (stderr, "Read from audio interface failed (%s)\n",snd_strerror (err));
			exit (1);
		}
		else {
				for (j = 0; j < BUFF_SIZE; j++){	
					//float sample_j = (float)buf[j]/(float)32768.0;
					double sample_d = (double)buf[j]/(double)32768.0;
					sampleData.push_back(sample_d);
		
					if (fft_calc->add_sample_RT((float)sample_d)){
						//get last sample
						vector<double> next_fft = fft_calc->get_last_fft_column();
						
						for (int k =0; k < next_fft.size(); k++){
							outputDftFile << next_fft.at(k)<< " ";
						}
						outputDftFile << endl;
						
						result.push_back(next_fft);
						num_fft_columns++;
					}
					c++;
				}
		}
		
		if (c > num_samples_to_record)
			break;
	}
	
	printf("Listened for %i samples and %i fft columns...\n",c,num_fft_columns);
	return result;
}

bool audio_data(grounded_logging::ProcessAudio::Request &req, 
				grounded_logging::ProcessAudio::Response &res)
{	
		string wavFileName = req.outputRawFileName;
		string outputDftFileName = req.outputDftFileName;
		sampleDuration = req.sampleDuration;
		
		std::ofstream outputDftFile(outputDftFileName.c_str());
		
		if(!outputDftFile.is_open()){
			std::cout<< "Could not open the file to store\n";
			return -1;
		}
		else{
			//open a handle to the microphone
			capture_handle = open_mic_capture("hw:2,0",SAMPLE_RATE);

			/* INITIALIZE FFT */
			FFT_calculator* FFT = new FFT_calculator(NFFT,FFT_WIN,NOVERLAP);
			FFT->init_RT_mode();
			sleep(2);
			
			vector< vector<double> > temp = record_fft_columns(FFT, outputDftFile);
		}
		snd_pcm_close (capture_handle);
		
		// Writing to the .wav file
		SndfileHandle outputRawFile(wavFileName.c_str(), SFM_WRITE, format, channel, SAMPLE_RATE);
		if(not outputRawFile){
			("Could not create the .wav file");
			return -1;
		}
		//convert vector to array
		double *data = &sampleData[0];
		// write to file
		outputRawFile.write(&data[0], sampleData.size());
	return true;
}

int main (int argc, char** argv)
{
	// Initialize ROS
	ros::init (argc, argv, "audio_processing");
	ros::NodeHandle nh;
	
	//Set up the service
	ros::ServiceServer service = nh.advertiseService("process_audio", audio_data);
	
	ROS_INFO("Ready to record and process audio data.");
	ros::spin();
	
	return 0;
}
