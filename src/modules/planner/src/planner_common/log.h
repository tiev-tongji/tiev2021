#include <iostream>
using namespace std;

namespace TiEV {

#define LOG_LEVEL_1
#define nameof(x) #x

#define _NUM_ARGS(X100, X99, X98, X97, X96, X95, X94, X93, X92, X91, X90, X89, X88, X87, X86, X85, X84, X83, X82, X81, X80, X79, X78, X77, X76, X75, X74, X73, X72, X71, X70, X69, X68, X67, X66, X65, X64, X63, X62, X61, X60, X59, X58, X57, X56, X55, X54, X53, X52, X51, X50, X49, X48, X47, X46, X45, X44, X43, X42, X41, X40, X39, X38, X37, X36, X35, X34, X33, X32, X31, X30, X29, X28, X27, X26, X25, X24, X23, X22, X21, X20, X19, X18, X17, X16, X15, X14, X13, X12, X11, X10, X9, X8, X7, X6, X5, X4, X3, X2, X1, N, ...)   N
#define NUM_ARGS(...) _NUM_ARGS(__VA_ARGS__, 100, 99, 98, 97, 96, 95, 94, 93, 92, 91, 90, 89, 88, 87, 86, 85, 84, 83, 82, 81, 80, 79, 78, 77, 76, 75, 74, 73, 72, 71, 70, 69, 68, 67, 66, 65, 64, 63, 62, 61, 60, 59, 58, 57, 56, 55, 54, 53, 52, 51, 50, 49, 48, 47, 46, 45, 44, 43, 42, 41, 40, 39, 38, 37, 36, 35, 34, 33, 32, 31, 30, 29, 28, 27, 26, 25, 24, 23, 22, 21, 20, 19, 18, 17, 16, 15, 14, 13, 12, 11, 10, 9, 8, 7, 6, 5, 4, 3, 2, 1)
#define OUTPUT(...) OUTPUT_(NUM_ARGS(__VA_ARGS__), __VA_ARGS__)
#define OUTPUT_(N, ...) OUTPUT__(N, __VA_ARGS__)
#define OUTPUT__(N, ...) cout << OUTPUT_##N(__VA_ARGS__)
#define OUTPUT_1(x, ...) x << endl;
#define OUTPUT_2(x, ...) x << OUTPUT_1(__VA_ARGS__)
#define OUTPUT_3(x, ...) x << OUTPUT_2(__VA_ARGS__)
#define OUTPUT_4(x, ...) x << OUTPUT_3(__VA_ARGS__)
#define OUTPUT_5(x, ...) x << OUTPUT_4(__VA_ARGS__)
#define OUTPUT_6(x, ...) x << OUTPUT_5(__VA_ARGS__)
#define OUTPUT_7(x, ...) x << OUTPUT_6(__VA_ARGS__)
#define OUTPUT_8(x, ...) x << OUTPUT_7(__VA_ARGS__)
#define OUTPUT_9(x, ...) x << OUTPUT_8(__VA_ARGS__)
#define OUTPUT_10(x, ...) x << OUTPUT_9(__VA_ARGS__)
#define OUTPUT_11(x, ...) x << OUTPUT_10(__VA_ARGS__)
#define OUTPUT_12(x, ...) x << OUTPUT_11(__VA_ARGS__)
#define OUTPUT_13(x, ...) x << OUTPUT_12(__VA_ARGS__)
#define OUTPUT_14(x, ...) x << OUTPUT_13(__VA_ARGS__)
#define OUTPUT_15(x, ...) x << OUTPUT_14(__VA_ARGS__)
#define OUTPUT_16(x, ...) x << OUTPUT_15(__VA_ARGS__)

#if defined(LOG_LEVEL_2)
// fully detailed logs for debug only
    #define LOG_LEVEL_1
    #define log_2(...) OUTPUT(__VA_ARGS__)
#else
    #define log_2(...)
#endif

#if defined(LOG_LEVEL_1)
// rich log mode
    #define LOG_LEVEL_0
    #define log_1(...) OUTPUT(__VA_ARGS__)
#else
    #define log_1(...)
#endif

#if defined(LOG_LEVEL_0)
// pure log mode
    #define log_0(...) OUTPUT(__VA_ARGS__)
#else
    #define log_0(...)
#endif
}