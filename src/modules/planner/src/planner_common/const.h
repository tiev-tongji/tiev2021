#ifndef __CONST__H__
#define __CONST__H__
namespace TiEV {
constexpr int    MAX_ROW         = 501;
constexpr int    MAX_COL         = 251;
constexpr int    CAR_CEN_COL     = 125;
constexpr int    CAR_CEN_ROW     = 350;
constexpr double GRID_RESOLUTION = 0.2;
constexpr double PI              = 3.1415926535898;
constexpr double inf             = 1e8;
constexpr double CAR_R           = 5.5;  // m
constexpr double CAR_MAX_K       = 1 / CAR_R;
constexpr int    ANGLE_NUM       = 72;

constexpr double MIN_OVERTAKE_SPEED = 5;  // mps
}  // namespace TiEV

#endif  //!__CONST__H__