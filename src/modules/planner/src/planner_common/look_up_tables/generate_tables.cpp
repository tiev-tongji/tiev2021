#include "iostream"
#include "look_up_tables/dubins_table/dubins_table_helper.h"
#include "look_up_tables/reeds_shepp_table/reeds_shepp_table_helper.h"
#include <string>
using namespace std;
using namespace TiEV;

string reeds_shepp_distance_table_file_name = "reeds_shepp_distance_table";
string dubins_distance_table_file_name      = "dubins_distance_table";

int main() {
    ReedsSheppTableHelper::generate_distance_table_file(560, 0.1, 72, 30, reeds_shepp_distance_table_file_name);
    DubinsTableHelper::generate_distance_table_file(560, 0.1, 72, 30, dubins_distance_table_file_name);
    return 0;
}