#include <iostream>
using namespace std;

namespace TiEV {

#define LOG_LEVEL 2

#ifdef LOG_LEVEL
    template <typename T> void write_line(const T &t) {
        cout << t << endl;
    }

    template <typename T, typename... Args>
    void write_line(const T &t, const Args &... rest){
        cout << t;
        write_line(rest...);
    }

    template <typename... Args>
    void log(int level, const Args &... rest){
        if (level <= LOG_LEVEL)
            write_line(rest...);
    }
#endif

}