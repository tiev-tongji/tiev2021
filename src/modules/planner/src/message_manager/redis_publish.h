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
    char *send_buf = new char [msg->getEncodedSize()];
    int send_cnt = msg->encode(send_buf, 0, msg->getEncodedSize());
    redisReply *reply = (redisReply *)redisCommand(context, "publish %s %b", channel.c_str(), send_buf, send_cnt);
    if (NULL == reply || reply->type != REDIS_REPLY_INTEGER){ //成功推送几个就会有几个 integer
      std::cout << channel + std::string("发送失败") << std::endl;
    }
    freeReplyObject(reply);

    delete []send_buf;
}