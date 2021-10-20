#include <string>
using namespace std;
namespace TiEV {

class DistanceTable {
 public:
  DistanceTable(const string& table_file_name);
  double getDistance(double q0[3], double q1[3]) const;
  ~DistanceTable();

 private:
  int       r, angle_num;
  double    step;
  double    turning_radius = 0;
  double*   angles         = NULL;
  double*   mem            = NULL;  //[x][y][angle_index][angle_index]
  long long getPos(int count, int a1, int a2) const;
};

}  // namespace TiEV
