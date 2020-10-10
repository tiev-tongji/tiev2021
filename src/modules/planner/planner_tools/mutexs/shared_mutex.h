#ifndef _SHARED_MUTEX_
#define SHARED_MUTEX

#include <mutex>
using namespace std;
namespace TiEV{

class shared_mutex{
public:
    void lock();
    void lock_shared();
    void unlock();
    void unlock_shared();
    bool try_lock();
    bool try_lock_shared();
private:
    unsigned int shared_cnt = 0;
    std::mutex main_mtx, shared_mtx;
};

}



#endif