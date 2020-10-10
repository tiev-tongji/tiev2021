#include "time_lock.h"
#include <algorithm>
namespace TiEV{
	TimeLock::TimeLock() : isLockMark(0), lockTime(0){}

	int TimeLock::lock(long long _lockTime){
		isLockMark = 1;
		lockTime = std::max(_lockTime * 1000, isLockUs());
		if(_lockTime * 1000 == lockTime) 
			gettimeofday(&lockBeginTime, NULL);
		isLockMark = 1;
		return lockTime;
	}

	long long TimeLock::isLockUs() {
		if (!isLockMark) return false;
		struct timeval currentTime;
		gettimeofday(&currentTime, NULL);
		long long durationTime = (currentTime.tv_sec - lockBeginTime.tv_sec)* 1000000 + currentTime.tv_usec - lockBeginTime.tv_usec;
		if (durationTime > lockTime) isLockMark = 0;
		if (isLockMark) return (lockTime - durationTime);
		return false;
	}
	
	void TimeLock::unlock() {
		isLockMark = 0;
		return ;
	}

	long long TimeLock::isLock(){
		return (1 + isLockUs()) / 1000000;
	}

}
