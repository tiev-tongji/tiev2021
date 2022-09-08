#include <iostream>
#include <hiredis/hiredis.h>

class Redis{
    redisContext *context;

public:
    void redisInit();
    template <class Msg>
    void redisPublish(const std::string& channel, const Msg* msg);
};


//使用类模板的函数体和头文件需在一个文件中
template <class Msg>
void Redis::redisPublish(const std::string& channel, const Msg* msg){
    uint32_t len = msg->getEncodedSize();
    uint8_t* buf = new uint8_t[len];
    if (!buf) return;
    int encodeRet = msg->encode(buf, 0, len);
    if (encodeRet < 0 || (uint32_t) encodeRet != len) {
        delete[] buf;
        return;
    }

    redisReply *reply = NULL;
    reply = (redisReply *)redisCommand(context, "publish %s %b", channel.c_str(), buf, encodeRet);
    
    delete[] buf;

    if (NULL == reply || reply->type != REDIS_REPLY_INTEGER){ 
      std::cout << channel + std::string("发送失败") << std::endl;
    }

    return ;
}