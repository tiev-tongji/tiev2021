#ifndef _TIME_COUNTER_
#define _TIME_COUNTER_
#include <iostream>
#include <ostream>
#include <sys/time.h>

namespace TiEV{

class TimeCounter{
private:
	struct timeval counterBeginTime;
	int isEnable;

public:
	TimeCounter();
	int start();
	long long duration();
	long long durationUs();
	long long durationMs();
};

}

#endif //_TIME_COUNTER_
