#pragma once

#include <sstream>



	class CircularBuffer {
		static const int LIMIT = 5;
		double data[LIMIT];
		int next;
	
	public:
	
		CircularBuffer(){
			for(int x=0; x<LIMIT; x++)
				data[x] = 0;
			next = 0;
		}
		CircularBuffer& operator<<(double v){
			push(v);
			return *this;
		}
		void push(double v){
			data[next] = v;
			next++;
			if( next >= LIMIT )
				next=0;
		}
		
		double mean()const{
			double MEAN=0.0;
			for( int x=1; x<LIMIT; x++)
				MEAN+=data[x];
			
			return MEAN/(double)LIMIT;
		}
		
		double median()const{
			//median filter
			double MAX = data[0];
			for( int x=1; x<LIMIT; x++){
				if( data[x] > MAX )
					MAX = data[x];
			}
			int belowMax = 0;
			do{
				double max = 0;
				for( int x=0; x<LIMIT; x++ ){
					if( data[x] < MAX && data[x] > max )
						max = data[x];
				}
				MAX = max;
				belowMax = 0;
				for( int x=0; x<LIMIT; x++ ){
					if( data[x] < MAX )
						++belowMax;
				}
			}while( belowMax > LIMIT/2 );
				
			
			return MAX;
		}
		std::string toString(){
			std::stringstream ss;
			ss << "[";
			for( int x=0; x<LIMIT; x++)
				ss << data[x] << " ";
			ss << "]";
			return ss.str();
		}
	};
		
