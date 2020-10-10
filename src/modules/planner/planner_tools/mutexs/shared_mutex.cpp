#include "mutexs/shared_mutex.h"

namespace TiEV{

void shared_mutex::lock(){
    main_mtx.lock();
}

void shared_mutex::lock_shared(){
    shared_mtx.lock();
    if((++shared_cnt) == 1)
        main_mtx.lock();
    shared_mtx.unlock();
}

void shared_mutex::unlock(){
    main_mtx.unlock();
}

void shared_mutex::unlock_shared(){
    shared_mtx.lock();
    if(--shared_cnt == 0)
        main_mtx.unlock();
    shared_mtx.unlock();
}

bool shared_mutex::try_lock(){
    return main_mtx.try_lock();
}

bool shared_mutex::try_lock_shared(){
    shared_mtx.lock();
    if(shared_cnt == 0){
        if(!main_mtx.try_lock()){
            shared_mtx.unlock();
            return false;
        }
        else ++shared_cnt;
    }
    shared_mtx.unlock();
}


}