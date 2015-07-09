#ifndef _fft_calc_dq_
#define _fft_calc_dq_

#define PI 3.14159

//some standard C++
#include <vector>
#include <stdio.h>
#include <string>
#include <cstdio>
#include <deque>
#include <math.h>

//some audio and boost libs
#include <signal.h>

//for computing fft
#include <fftw3.h> //need to install the following: "libfftw3-3" and "libfftw3-dev"

//how many FFT column vectors to keep in memory, used in real time mode
#define MAX_NUM_FFT_COLUMNS 640

using std::deque;
using std::vector;
using std::string;

class FFT_calculator {
	public:
		FFT_calculator(){
			set_params(256,256,128);//default params
		};

		FFT_calculator(int n, int w, int o){
			set_params(n,w,o);
		};

		void set_params(int n, int w, int o){
			NFFT=n;
			FFT_WIN = w;
			NOVERLAP = o;
		
			win = (double*)malloc(FFT_WIN * sizeof(double));

			//init variables for fftw
		        fft_in = (double*)fftw_malloc(sizeof(double)*NFFT);
        		fft_out = (fftw_complex*)fftw_malloc(sizeof(fftw_complex)*(NFFT/2+1));
        		plan = fftw_plan_dft_r2c_1d(NFFT, fft_in, fft_out,FFTW_ESTIMATE);


		};

		int get_num_fft(){
			return NFFT;
		};

		int get_num_fft_samples(){
			return (int)fft_data.size();			
		}
		
		vector<double> get_last_fft_column(){
		    return fft_data[(int)fft_data.size()-1];
		};

		//initializes real time mode
		void init_RT_mode(){
		    win = (double*)malloc(FFT_WIN * sizeof(double));
		    sample_counter = 0;
		}
		
		//process next audio sample - returns true if new FFT column was generated
		bool add_sample_RT(float sample_i){
		    
		    bool new_column = false;
		  
		    //check if it is time to compute the fft of the current window
		    if (sample_counter == FFT_WIN){
				/* process the next window of length FFT_WIN */
				process_window(win, FFT_WIN, fft_in, fft_out, &plan);

				/* move the last noverlap samples in the window to its front */
				memmove(win, &win[FFT_WIN-NOVERLAP], sizeof(double)*NOVERLAP);
				sample_counter=NOVERLAP;
				
				//check if the vector of columns is at the max limit
				if (fft_data.size() > MAX_NUM_FFT_COLUMNS)
					fft_data.pop_front();
				
				new_column = true;
		    }
		    
		    //add sample to the next position in current window
		    win[sample_counter++]=(double)sample_i;
		    
		    return new_column;
		}


		/* computes the fft of the input vector */
		void compute_fft( vector<float> clip_samples){
			int winsamps = 0;

		        fft_data = deque< vector<double> >();

       			for (int i = 0; i < (int)clip_samples.size(); i++){
                		if (winsamps == FFT_WIN){
                        		/* process the next window of length FFT_WIN */
					process_window(win, winsamps, fft_in, fft_out, &plan);

                        		/* move the last noverlap samples in the window to its front */
                        		memmove(win, &win[FFT_WIN-NOVERLAP], sizeof(double)*NOVERLAP);
                        		winsamps = NOVERLAP;

                		}
                		win[winsamps++] = (double)clip_samples[i];
        		}
		};		

		/* computes the FFT of a single window of length FFT_WIN; to be used in case FFT is needed in real time */
		vector<double> compute_single_fft( double *single_window){

			/* process the data */
			return compute_fft_column(win, FFT_WIN, fft_in, fft_out, &plan);

			/* return the single column vector */
			//return fft_data[(int)fft_data.size()-1];	
		}


		/* returns the FFT matrix; first dimension is time, second is frequency bin */
		deque< vector<double> > getFFT(){
			return fft_data;
		};

		void save_fft(char* filename){
		        FILE *fp = fopen(filename, "w");
			fprintf(fp,"%d\t%d\n",NFFT,(int)fft_data.size());

       			for (unsigned int i = 0; i < fft_data.size(); i++){
	        	        vector<double> next_fft = fft_data[i];
        	        	for (int j = 0; j < NFFT/2 +1; j++)
                        		//fprintf(fp,"%.4f%s",next_fft[j],j == (NFFT/2+1-1) ? "\n":"\t");

                        		fprintf(fp,"%.4f%s",fft_data[i][j],j == (NFFT/2+1-1) ? "\n":"\t");


		        }
        		fclose(fp);
		};



		void save_as_matrix(char* filename){
		        FILE *fp = fopen(filename, "w");
       			for (unsigned int i = 0; i < fft_data.size(); i++){
	        	        vector<double> next_fft = fft_data[i];
        	        	for (int j = 0; j < NFFT/2 +1; j++)
                        		//fprintf(fp,"%.4f%s",next_fft[j],j == (NFFT/2+1-1) ? "\n":"\t");

                        		fprintf(fp,"%.4f%s",fft_data[i][j],j == (NFFT/2+1-1) ? "\n":"\t");


		        }
        		fclose(fp);
		};


	
	private:
		
		//parameters for FFT
		int NFFT; // the number of frequency bins in the FFT will be (NFFT/2)+1
		int FFT_WIN; // the length of the fft window
		int NOVERLAP; // the overlap - by default, it will be half the length of the fft window

		//variables used by fftw	
		double *win;
		double *fft_in;
		fftw_complex *fft_out;
		fftw_plan plan;

		//where the fft is stored
		deque< vector<double> > fft_data;
		
		
		int sample_counter;//used in real time mode
		

		/* Computes the Hamming window function for a window of size N */
		static inline double hamming(unsigned int n, const unsigned int N) {
  			return 0.54 - 0.46*cos(2*M_PI*n/(N - 1));
		}

		static inline double tophat(unsigned int n, const unsigned int N){
			return 0.215578 - 0.41663*cos(2*M_PI*n/N) + 0.277263*cos(4*M_PI*n/N) - 0.08358*cos(6*M_PI*n/N) + 0.006947*cos(8*M_PI*n/N); 			
		}
		
		/* computes the FFT spectrum of a single window of size FFT_WIN */
		vector<double> compute_fft_column(double *win, int winsamps, double *fft_in, fftw_complex *fft_out, fftw_plan *plan) {
		    vector<double> column = vector<double>(NFFT/2+1);
		  
		    int i;
        		for (i=0; i<winsamps; ++i) {
                		fft_in[i % NFFT] = tophat(i, FFT_WIN)*win[i];
                		if (!((i+1) % NFFT)) {
                        		fftw_execute(*plan);

                        		
                        		for (int j=0; j<NFFT/2+1; ++j) {
					  double v = (sqrt( ((double)fft_out[j][0])*(double)fft_out[j][0]) + ((double)fft_out[j][1]*((double)fft_out[j][1])));
					 // if (j<7)
					 //   printf("%f ",v);
					  column[j]=v;

					}
                		}
        		}
		    return column;
		}
		

		/* computes the FFT spectrum of a single window of size FFT_WIN */
		void process_window(double *win, int winsamps, double *fft_in, fftw_complex *fft_out, fftw_plan *plan) {
		        int i;
        		for (i=0; i<winsamps; ++i) {
                		fft_in[i % NFFT] = tophat(i, FFT_WIN)*win[i];
                		if (!((i+1) % NFFT)) {
                        		fftw_execute(*plan);

                        		vector<double> column;
                        		for (int j=0; j<NFFT/2+1; ++j) 
                                		column.push_back(sqrt( ((double)fft_out[j][0])*(double)fft_out[j][0]) + ((double)fft_out[j][1]*((double)fft_out[j][1])));
                        		fft_data.push_back(column);
                		}
        		}
		}
};

#endif
