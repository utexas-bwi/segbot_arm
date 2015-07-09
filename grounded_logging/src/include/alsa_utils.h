#include <stdio.h>
#include <stdlib.h>
#include <string>
#include <alsa/asoundlib.h>

snd_pcm_t* open_mic_capture(std::string dev_name, int sample_rate){
	
	int i;
	int err;
	
	snd_pcm_t *capture_handle;
	snd_pcm_hw_params_t *hw_params;
	
	if ((err = snd_pcm_open (&capture_handle, dev_name.c_str(), SND_PCM_STREAM_CAPTURE, 0)) < 0) {
			fprintf (stderr, "cannot open audio device %s (%s)\n", 
				 dev_name.c_str(),
				 snd_strerror (err));
			exit (1);
	}
		   
	if ((err = snd_pcm_hw_params_malloc (&hw_params)) < 0) {
		fprintf (stderr, "cannot allocate hardware parameter structure (%s)\n",
			 snd_strerror (err));
		exit (1);
	}
				 
	if ((err = snd_pcm_hw_params_any (capture_handle, hw_params)) < 0) {
		fprintf (stderr, "cannot initialize hardware parameter structure (%s)\n",
			 snd_strerror (err));
		exit (1);
	}
	
	if ((err = snd_pcm_hw_params_set_access (capture_handle, hw_params, SND_PCM_ACCESS_RW_INTERLEAVED)) < 0) {
		fprintf (stderr, "cannot set access type (%s)\n",
			 snd_strerror (err));
		exit (1);
	}
	
	if ((err = snd_pcm_hw_params_set_format (capture_handle, hw_params, SND_PCM_FORMAT_S16_LE)) < 0) {
		fprintf (stderr, "cannot set sample format (%s)\n",
			 snd_strerror (err));
		exit (1);
	}
	
	
	if ((err = snd_pcm_hw_params_set_rate_resample (capture_handle, hw_params, sample_rate)) < 0) {
		fprintf (stderr, "cannot set sample rate (%s)\n",snd_strerror (err));
		exit (1);
	}
	

	
	if ((err = snd_pcm_hw_params_set_channels (capture_handle, hw_params, 1)) < 0) {
		fprintf (stderr, "cannot set channel count (%s)\n",
			 snd_strerror (err));
			exit (1);
	}
	
	if ((err = snd_pcm_hw_params (capture_handle, hw_params)) < 0) {
		fprintf (stderr, "cannot set parameters (%s)\n",
			 snd_strerror (err));
		exit (1);
	}
	
	snd_pcm_hw_params_free (hw_params);

	if ((err = snd_pcm_prepare (capture_handle)) < 0) {
		fprintf (stderr, "cannot prepare audio interface for use (%s)\n",
			 snd_strerror (err));
		exit (1);
	}
	
	return capture_handle;
}
