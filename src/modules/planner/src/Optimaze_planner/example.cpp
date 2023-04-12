#include <Eigen/Core>
#include <Eigen/Sparse>
#include <iostream>
#include <opencv2/highgui.hpp>
#include <opencv2/opencv.hpp>

#include "OsqpEigen/OsqpEigen.h"
#include "osqp/osqp.h"

const double EPS = 1e-4;
using namespace cv;
using namespace std;
using namespace Eigen;
struct diff {
  double          d0, d1, d2;
  friend ostream& operator<<(ostream& os, const diff& ths) {
    return cout << "[" << ths.d0 << "," << ths.d1 << "," << ths.d2 << "]"
                << endl;
  }
};
class RefRoad {
 public:
  vector<diff> state_x, state_y;
};

void drawRefRoad(const RefRoad& ref_road){



}

double calRefRoadCost(const RefRoad& ref_road) {
  double ref_road_cost = 0;
  for (auto [d0, d1, d2] : ref_road.state_x) {
    ref_road_cost += (d1 * d1 + d2 * d2);
  }
  for (auto [d0, d1, d2] : ref_road.state_y) {
    ref_road_cost += (d1 * d1 + d2 * d2);
  }
  return ref_road_cost;
}

bool isRefRoadLegal(const RefRoad& ref_road) {
  int state_num = ref_road.state_x.size();
  for (int i = 0; i < state_num; i++) {
    double dx        = ref_road.state_x[i].d1;
    double ddx       = ref_road.state_x[i].d2;
    double dy        = ref_road.state_y[i].d1;
    double ddy       = ref_road.state_y[i].d2;
    double tmpxy     = pow((dx * dx + dy * dy), 1.5);
    double ddx_const = - -(dy) / (tmpxy), ddy_const = dx / (tmpxy);
    double curv = ddx_const * ddx + ddy_const * ddy;
    if (abs(curv) >= 0) {  // curv limit
      cout << " the curv is outer in " << curv << " in pos " << i
           << " when statex is " << ref_road.state_x[i] << " statey is "
           << ref_road.state_y[i] << endl;
      //return false;
    }
  }
  return true;
}
void drawRefRoad(const RefRoad& ref_road){

}
RefRoad OptimazeRefRoad(RefRoad& warm_start) {
  int  poly_num = (int)warm_start.state_x.size() - 1;
  auto state_x  = warm_start.state_x;
  auto state_y  = warm_start.state_y;
  assert(poly_num > 0 &&
         warm_start.state_x.size() == warm_start.state_y.size());
  const int                   param_num = 8;
  Eigen::SparseMatrix<double> hessian(param_num * poly_num,
                                      param_num * poly_num);
  for (int i = 0; i < poly_num; i++) {
    int offset                             = i * param_num;
    hessian.insert(2 + offset, 2 + offset) = 8;
    hessian.insert(2 + offset, 3 + offset) = 12;
    hessian.insert(3 + offset, 2 + offset) = 12;
    hessian.insert(3 + offset, 3 + offset) = 108;
    // for y param
    offset += param_num / 2;
    hessian.insert(2 + offset, 2 + offset) = 8;
    hessian.insert(2 + offset, 3 + offset) = 12;
    hessian.insert(3 + offset, 2 + offset) = 12;
    hessian.insert(3 + offset, 3 + offset) = 108;
  }
  cout << " hessian matrix finish " << endl;

  Eigen::VectorXd gradient = Eigen::VectorXd::Zero(param_num * poly_num);
  // for start_end cons
  const int continue_constraint_num  = (poly_num - 1) * 6;
  const int start_end_constraint_num = 4;
  const int curv_constraint_num      = poly_num + 1;

  const int tot_constraint_num =
      continue_constraint_num + start_end_constraint_num + curv_constraint_num;
  Eigen::SparseMatrix<double> linearMatrix(tot_constraint_num,
                                           param_num * poly_num);
  Eigen::VectorXd             lowerBound(tot_constraint_num);
  Eigen::VectorXd             upperBound(tot_constraint_num);
  /*
   *    continue constraint
   */
  for (int i = 0; i < poly_num - 1; i++) {
    int offset    = i * param_num;
    int nx_offset = (i + 1) * param_num;
    //[ c0 + c1 + c2 + c3 == c0 ]
    linearMatrix.insert(i, offset + 0) = 1;
    linearMatrix.insert(i, offset + 1) = 1;
    linearMatrix.insert(i, offset + 2) = 1;
    linearMatrix.insert(i, offset + 3) = 1;
    linearMatrix.insert(i, nx_offset)  = -1;
    lowerBound[i]                      = -EPS;
    upperBound[i]                      = EPS;
    //[ c1 + 2c2+ 3c3     == c1 ]
    linearMatrix.insert(i + 1, offset + 1)    = 1;
    linearMatrix.insert(i + 1, offset + 2)    = 2;
    linearMatrix.insert(i + 1, offset + 3)    = 3;
    linearMatrix.insert(i + 1, nx_offset + 1) = -1;
    lowerBound[i + 1]                         = -EPS;
    upperBound[i + 1]                         = EPS;
    //[ 2c2+ 6c3          == c2 ]
    linearMatrix.insert(i + 2, offset + 2)    = 2;
    linearMatrix.insert(i + 2, offset + 3)    = 6;
    linearMatrix.insert(i + 2, nx_offset + 2) = -1;
    lowerBound[i + 2]                         = -EPS;
    upperBound[i + 2]                         = EPS;

    offset += param_num / 2, nx_offset += param_num / 2;
    //[ c0 + c1 + c2 + c3 == c0 ]
    linearMatrix.insert(i + 3, offset + 0) = 1;
    linearMatrix.insert(i + 3, offset + 1) = 1;
    linearMatrix.insert(i + 3, offset + 2) = 1;
    linearMatrix.insert(i + 3, offset + 3) = 1;
    linearMatrix.insert(i + 3, nx_offset)  = -1;
    lowerBound[i + 3]                      = -EPS;
    upperBound[i + 3]                      = EPS;
    //[ c1 + 2c2+ 3c3     == c1 ]
    linearMatrix.insert(i + 4, offset + 1)    = 1;
    linearMatrix.insert(i + 4, offset + 2)    = 2;
    linearMatrix.insert(i + 4, offset + 3)    = 3;
    linearMatrix.insert(i + 4, nx_offset + 1) = -1;
    lowerBound[i + 4]                         = -EPS;
    upperBound[i + 4]                         = EPS;
    //[ 2c2+ 6c3          == c2 ]
    linearMatrix.insert(i + 5, offset + 2)    = 2;
    linearMatrix.insert(i + 5, offset + 3)    = 6;
    linearMatrix.insert(i + 5, nx_offset + 2) = -1;
    lowerBound[i + 5]                         = -EPS;
    upperBound[i + 5]                         = EPS;
  }

  cout << " finish continue constraint " << endl;
  /*
   * start end constraint
   */
  const double start_x = state_x[0].d0;
  const double end_x   = state_x[state_x.size() - 1].d0;

  linearMatrix.insert(continue_constraint_num + 0, 0) = 1;
  lowerBound[continue_constraint_num]                 = -EPS + start_x;
  upperBound[continue_constraint_num]                 = EPS + start_x;
  cout << " for the s is " << start_x << endl;
  int offset_x = (poly_num-1) * param_num;
  linearMatrix.insert(continue_constraint_num + 1,
                      offset_x + 0) = 1;
  linearMatrix.insert(continue_constraint_num + 1,
                      offset_x + 1) = 1;
  linearMatrix.insert(continue_constraint_num + 1,
                      offset_x + 2) = 1;
  linearMatrix.insert(continue_constraint_num + 1,
                      offset_x + 3) = 1;
  lowerBound[continue_constraint_num + 1]             = -EPS + end_x;
  upperBound[continue_constraint_num + 1]             = EPS + end_x;
  cout << " for the e is " << end_x << endl;

  const double start_y = state_y[0].d0;
  const double end_y   = state_y[state_y.size() - 1].d0;

  linearMatrix.insert(continue_constraint_num + 2, param_num/2) = 1;
  lowerBound[continue_constraint_num + 2]             = -EPS + start_y;
  upperBound[continue_constraint_num + 2]             = EPS + start_y;

  int offset = (poly_num - 1) * param_num + param_num / 2;
  linearMatrix.insert(continue_constraint_num + 3, offset + 0) = 1;
  linearMatrix.insert(continue_constraint_num + 3, offset + 1) = 1;
  linearMatrix.insert(continue_constraint_num + 3, offset + 2) = 1;
  linearMatrix.insert(continue_constraint_num + 3, offset + 3) = 1;
  lowerBound[continue_constraint_num + 3]                      = -EPS + end_y;
  upperBound[continue_constraint_num + 3]                      = EPS + end_y;
  cout << " finish start end constraint " << endl;

  /*
   * curv constraint
   */
  const int pre_constraint_num =
      continue_constraint_num + start_end_constraint_num;
  assert(state_x.size() == curv_constraint_num);
  for (int i = 0; i < curv_constraint_num; i++) {
    int    constraint_id = pre_constraint_num + i;
    int    parax_id      = param_num * i + 2;
    int    paray_id      = param_num * i + 2 + 4;
    double dx            = state_x[i].d1;
    double dy            = state_y[i].d1;
    double tmpxy         = pow((dx * dx + dy * dy), 1.5);
    double ddx_const = - -(dy) / (tmpxy), ddy_const = dx / (tmpxy);

    if (i + 1 == curv_constraint_num) {
      int offset                                     = param_num * (i - 1);
      linearMatrix.insert(constraint_id, offset + 1) = ddx_const;
      linearMatrix.insert(constraint_id, offset + 2) = 2 * ddx_const;
      linearMatrix.insert(constraint_id, offset + 3) = 3 * ddx_const;

      offset += param_num / 2;
      linearMatrix.insert(constraint_id, offset + 1) = ddy_const;
      linearMatrix.insert(constraint_id, offset + 2) = 2 * ddy_const;
      linearMatrix.insert(constraint_id, offset + 3) = 3 * ddy_const;
      lowerBound[constraint_id]                      = -1;
      upperBound[constraint_id]                      = 1;
      continue;
    }
    linearMatrix.insert(constraint_id, parax_id) = ddx_const;
    linearMatrix.insert(constraint_id, paray_id) = ddy_const;
    // fit into dynamic model
    lowerBound[constraint_id] = -1;
    upperBound[constraint_id] = 1;
  }

  cout << " finish for all " << endl;
  OsqpEigen::Solver solver;
  solver.settings()->setVerbosity(false);
  solver.settings()->setWarmStart(true);
  solver.settings()->setMaxIteration(100);
  // Initialize the solver
  solver.data()->setNumberOfVariables(param_num * poly_num);
  solver.data()->setNumberOfConstraints(tot_constraint_num);
  if (!solver.data()->setHessianMatrix(hessian)) return warm_start;
  if (!solver.data()->setGradient(gradient)) return warm_start;
  if (!solver.data()->setLinearConstraintsMatrix(linearMatrix))
    return warm_start;
  if (!solver.data()->setLowerBound(lowerBound)) return warm_start;
  if (!solver.data()->setUpperBound(upperBound)) return warm_start;

  // Solve the problem
  if (!solver.initSolver()) return warm_start;
  if (solver.solveProblem() != OsqpEigen::ErrorExitFlag::NoError)
    return warm_start;

  // Get the solution
  const auto& solution = solver.getSolution();

  // reconstruct the ref_path
  vector<diff> nx_state_x, nx_state_y;
  for (int i = 0; i < poly_num; i++) {
    int  offset = param_num * i;
    diff dx;
    dx.d0 = solution[offset + 0];
    dx.d1 = solution[offset + 1];
    dx.d2 = 2 * solution[offset + 2];
    nx_state_x.push_back(dx);

    offset += param_num / 2;
    diff dy;
    dy.d0 = solution[offset + 0];
    dy.d1 = solution[offset + 1];
    dy.d2 = 2 * solution[offset + 2];
    nx_state_y.push_back(dy);
  }
  diff dx;
  cout << " for the solution " << endl;
  for (int i = 0; i < param_num; i++) {
    cout << solution[(poly_num - 1) * param_num + i] << ",";
  }
  cout << endl;
  dx.d0 = solution[((poly_num - 1) * param_num) + 0];
  dx.d0 += solution[((poly_num - 1) * param_num) + 1];
  dx.d0 += solution[((poly_num - 1) * param_num) + 2];
  dx.d0 += solution[((poly_num - 1) * param_num) + 3];

  dx.d1 = solution[((poly_num - 1) * param_num) + 1];
  dx.d1 += solution[((poly_num - 1) * param_num) + 2] * 2;
  dx.d1 += solution[((poly_num - 1) * param_num) + 3] * 3;

  dx.d2 = solution[((poly_num - 1) * param_num) + 2] * 2;
  dx.d2 += solution[((poly_num - 1) * param_num) + 3] * 6;
  nx_state_x.push_back(dx);

  diff dy;
  dy.d0 = solution[((poly_num - 1) * param_num) + 0 + param_num / 2];
  dy.d0 += solution[((poly_num - 1) * param_num) + 1 + param_num / 2];
  dy.d0 += solution[((poly_num - 1) * param_num) + 2 + param_num / 2];
  dy.d0 += solution[((poly_num - 1) * param_num) + 3 + param_num / 2];

  dy.d1 = solution[((poly_num - 1) * param_num) + 1 + param_num / 2];
  dy.d1 += solution[((poly_num - 1) * param_num) + 2 + param_num / 2] * 2;
  dy.d1 += solution[((poly_num - 1) * param_num) + 3 + param_num / 2] * 3;

  dy.d2 = solution[((poly_num - 1) * param_num) + 2 + param_num / 2] * 2;
  dy.d2 += solution[((poly_num - 1) * param_num) + 3 + param_num / 2] * 6;
  nx_state_y.push_back(dy);

  return {nx_state_x, nx_state_y};
}
void drawParametricCurve(const VectorXd& cx, const VectorXd& cy) {
  // 创建一个空白图像
  Mat img = Mat::zeros(300, 300, CV_8UC3);
  // 设置绘制曲线的颜色和粗细
  Scalar color     = Scalar(0, 255, 0);
  int    thickness = 2;
  // 计算曲线上的点并绘制
  for (double t = 0; t <= 1; t += 0.01) {
    double x = cx[0] + cx[1] * t + cx[2] * pow(t, 2) + cx[3] * pow(t, 3);
    double y = cy[0] + cy[1] * t + cy[2] * pow(t, 2) + cy[3] * pow(t, 3);
    circle(img, Point(x, y), thickness, color, -1);
  }
  // 显示图像
  imshow("Parametric Curve", img);
  waitKey(0);
}

int main() {
  // Set the solver settings
  // make test data

  vector<diff> state_x, state_y;
  state_x.push_back({0, 1, 0});
  state_x.push_back({10, 0.01, 0});
  state_x.push_back({20, 1, 0});

  state_y.push_back({0, 0.01, 0});
  state_y.push_back({10, 1, 0});
  state_y.push_back({20, 0.01, 0});

  RefRoad test_road{state_x, state_y};

  cout << " before optimaze " << calRefRoadCost(test_road);

  RefRoad res_road = OptimazeRefRoad(test_road);

  cout << " for x " << endl;

  for (auto state : res_road.state_x) {
    cout << state << endl;
  }
  cout << " for y " << endl;
  for (auto state : res_road.state_y) {
    cout << state << endl;
  }

  cout << " after optimaze " << calRefRoadCost(res_road) << endl;
  isRefRoadLegal(res_road);
}
