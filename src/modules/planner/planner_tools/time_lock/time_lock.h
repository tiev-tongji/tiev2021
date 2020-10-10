#ifndef _TIME_LOCK_
#define _TIME_LOCK_
#include <iostream>
#include <ostream>
#include <sys/time.h>

namespace TiEV{

class TimeLock{
private:
	int isLockMark;
	struct timeval lockBeginTime;
	long long lockTime;

public:
	TimeLock();
	int lock(long long _lockTime);
	long long isLock();
	long long isLockUs();
	void unlock();
};

}

#endif //_TIME_LOCK_
