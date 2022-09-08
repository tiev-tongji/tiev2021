#include <iostream>
#include "redis_publish.h"

void Redis::redisInit(){
    if(context)
        return;
    context = redisConnect("124.222.194.135", 4321);
    redisReply *reply = NULL;
    if (context->err)
    {
        std::cout << "can not connect to redis server" << std::endl;
        std::cout << "reason:" << context->errstr << std::endl;
        redisFree(context);
        context = NULL;
    }
    else{
        // 验证密码
        redisCommand(context, "AUTH %s", "tjredis!!");
    }
}

