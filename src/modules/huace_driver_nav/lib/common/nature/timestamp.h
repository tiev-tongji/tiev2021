#ifndef _TIMESTAMP_H_
#define _TIMESTAMP_H_
#include <ctime>
#include <stdint.h>
namespace TiEV{
    std::time_t getTimeStamp();
    std::tm* gettm(int64_t timestamp);
}

#endif
