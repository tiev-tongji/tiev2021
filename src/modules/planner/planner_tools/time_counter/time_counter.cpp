#include "time_counter.h"
namespace TiEV{
	TimeCounter::TimeCounter() : isEnable(0){}

	int TimeCounter::start(){
		isEnable = 1;
		gettimeofday(&counterBeginTime, NULL);
		return isEnable;
	}

	long long TimeCounter::durationUs(){
		if (!isEnable) return 0;
		struct timeval currentTime;
		gettimeofday(&currentTime, NULL);
		long long durationTime = (currentTime.tv_sec - counterBeginTime.tv_sec)* 1000000 + currentTime.tv_usec - counterBeginTime.tv_usec;
		return durationTime;
	}

	long long TimeCounter::durationMs() {
		return durationUs() / 1000;
	}

	long long TimeCounter::duration(){
		return durationUs() / 1000000;
	}

}
