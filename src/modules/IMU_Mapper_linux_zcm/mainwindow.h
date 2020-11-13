#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>

#include "structNAVINFO.hpp"
#include "structSLAMLOC.hpp"
#include "zcm/zcm-cpp.hpp"

#include <QMutexLocker>
#include <pthread.h>
// zz
#include "coordinate_converter.h"
#include "linux/input.h"
#include <fcntl.h>
#include <math.h>
#include <mutex>
#include <stdio.h>
#include <termios.h>
#include <thread>
#include <vector>

namespace Ui {
class MainWindow;
}

class MainWindow : public QMainWindow {
  Q_OBJECT

public:
  explicit MainWindow(QWidget *parent = 0);
  ~MainWindow();

  struct roadpoint {
    int id;
    double lon;
    double lat;
    double utmX;
    double utmY;
    float heading;
    float curvature;
    int mode;
    int speedmode;
    int eventmode;
    int opposite_side_mode;
    int laneno;
    int laneseq;
    float lanewidth;
  };

  enum ReceiveMode : int { // localization Mode
    RTK = 0,
    SLAM
  }; //定位方式的枚举

  std::string getReceiveModeName(ReceiveMode mode) {
    switch (mode) {
    case ReceiveMode::RTK:
      return "RTK";
      break;
    case ReceiveMode::SLAM:
      return "SLAM";
      break;
    default:
      return "error";
      break;
    }
  }

  enum SpeedMode : int { //速度的状态Mode1
    BACK = 0,
    STOP,
    VERYLOW,
    LOW,
    MID,
    HIGH,
    VERYHIGH
  }; //速度的枚举

  enum OppositeSideMode : int { //借道Mode
    LEFT_NO_RIGHT_NO = 0,       //左右均不封
    LEFT_NO_RIGHT_YES,          //左不封右封
    LEFT_YES_RIGHT_NO,          //左封右不封
    LEFT_YES_RIGHT_YES,         //左右均封
    todo1                       //预留
  };                            //借道的枚举

  enum Mode : int {     //事件Mode2
    NORMAL_DRIVING = 0, //正常行驶，使用默认权值
    INTERSECTION_SOLID,
    INTERSECT_WITHLIGHT, //路口
    PARK,                //停车点
    CHANGE_LANE,         //换道采路
    UTURN                // U字调头
    /*
    START,           //车辆启动
    PEDESTRIAN,          //人行道
    INTERSECT_NOLIGHT,   //汇入车流
    MAPFREE,             //无可靠地图
    OBSTACLFREE,         //无障碍物探测
    CLIMB,               //上下坡
    FOGGY,               //雨雾区
    SIDEPARKING,         //侧方位停车
    BRIDGE,              //桥
    UNDERGROUND         //地下
    */
  };

  enum Event : int { //事件Mode3
    NOEVENT = 0,     //无事件
    ENTRY_INTERSECTION,
    EXIT_INTERSECTION,
    CONDITION_STOPPOINT, //停止点
    CHANGE_PATH_FILE     //预留
  };

  std::vector<roadpoint> data;
  bool record_roadpoint;

  ReceiveMode ReceiveMode_;
  int num = 0;
  int mode = 0;
  int speedmode = 0;
  int eventmode = 0;
  int opposite_side_mode = 0;
  int laneno = 0;
  int laneseq = 0;
  int event_point_num = 0;
  float lanewidth = 0;
  bool event_record;
  roadpoint get_rtk_data(int, structNAVINFO);
  roadpoint get_slam_data(int, structSLAMLOC);
  void PickPoint(std::vector<roadpoint> &data);

private:
  Ui::MainWindow *ui;
public slots:
  void showInfo();
  void record_status_change();
  void save_roadmap();
  void SimplifyPointBtn();
  void ModeChange();
  void LocModeChange();
  void PokeEventPoint();
};

#endif // MAINWINDOW_H
