#include "string"
using namespace std;

namespace TiEV{

class ReedsSheppTableHelper{
public:
    /* For each point in the circle whose radius is specified, calculate the length of the shortest RS curve from (0,0) to that point and output it to the file.
     */
    static void generate_distance_table_file(int r, double step, int angle_num, double turning_radius, string file_name);
};

}

