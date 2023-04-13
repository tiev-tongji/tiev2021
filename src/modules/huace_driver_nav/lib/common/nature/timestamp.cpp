#include <chrono>
#include <ctime>
namespace TiEV{

	/*get current TimeStamp
	  Return: time_t / int64 in microseconds
	*/
	std::time_t getTimeStamp()
	{
		std::chrono::time_point<std::chrono::system_clock,std::chrono::microseconds> tp =
		std::chrono::time_point_cast<std::chrono::microseconds>(std::chrono::system_clock::now());
		auto tmp=std::chrono::duration_cast<std::chrono::microseconds>(tp.time_since_epoch());
		std::time_t timestamp = tmp.count();
    		//std::time_t timestamp = std::chrono::system_clock::to_time_t(tp);
		return timestamp;
	}

	/*parse TimeStamp into tm struct
	  Return: tm
	*/
	std::tm* gettm(int64_t timestamp)
	{
	    int64_t milli = timestamp + (int64_t)8*60*60*1000*1000;//此处转化为东八区北京时间，如果是其它时区需要按需求修改
	    auto mTime = std::chrono::microseconds(milli);
	    auto tp=std::chrono::time_point<std::chrono::system_clock,std::chrono::microseconds>(mTime);
	    auto tt = std::chrono::system_clock::to_time_t(tp);
	    std::tm* now = std::gmtime(&tt);
	    //printf("%4d年%02d月%02d日 %02d:%02d:%02d\n",now->tm_year+1900,now->tm_mon+1,now->tm_mday,now->tm_hour,now->tm_min,now->tm_sec);
	    return now;
	}
}
