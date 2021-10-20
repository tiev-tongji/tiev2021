#include <string>

#include "const.h"
#include "dubins_table_helper.h"
#include "iostream"
#include "reeds_shepp_table_helper.h"
using namespace std;
using namespace TiEV;

string reeds_shepp_distance_table_file_name =
    "~/tiev2020-code/src/cfg/reeds_shepp_distance_table";
string dubins_distance_table_file_name =
    "~/tiev2020-code/src/cfg/dubins_distance_table";

int main() {
  ReedsSheppTableHelper::generate_distance_table_file(
      560, 0.1, ANGLE_NUM, CAR_R / GRID_RESOLUTION,
      reeds_shepp_distance_table_file_name);
  DubinsTableHelper::generate_distance_table_file(
      560, 0.1, ANGLE_NUM, CAR_R / GRID_RESOLUTION,
      dubins_distance_table_file_name);
  return 0;
}