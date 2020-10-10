#include "mainwindow.h"
#include "ui_mainwindow.h"
#include <QTime>
#include <QTimer>
#include <string.h>
#include <cstring>
#include <QMessageBox>
#include <QFileDialog>
#include <iomanip>
#include <fstream>
#include "common/coordinate_converter/coordinate_converter.h"
#include "common/nature/angle.h"
#include <iostream>


using namespace std;

structNAVINFO NAVI_temp;
bool bRecvNAVINFO = false;
std::mutex NAVI_mtx;

structSLAMLOC SLAMLOC_temp;
bool bRecvSLAMLOC = false;
std::mutex SLAMLOC_mtx;

using namespace std;
class Handler
{
public:
    ~Handler() {}

    void handleMessage_odom(const zcm::ReceiveBuffer* rbuf,
       const std::string& chan,
       const structNAVINFO* msg)
    {

        NAVI_mtx.lock();
        NAVI_temp = *msg;
        bRecvNAVINFO = true;
        NAVI_mtx.unlock();
    }

    void handleMessage_slamloc(const zcm::ReceiveBuffer* rbuf,
       const std::string& chan,
       const structSLAMLOC* msg)
    {

        SLAMLOC_mtx.lock();
        SLAMLOC_temp = *msg;
        bRecvSLAMLOC = true;
        SLAMLOC_mtx.unlock();
    }

};

//for zcm
void zcm_func()
{
    zcm::ZCM zcm{"udpm://239.255.76.67:7667?ttl=1"};
    if (!zcm.good())
        return;
    Handler handlerObject;
    zcm.subscribe("NAVINFO", &Handler::handleMessage_odom, &handlerObject);
    zcm.subscribe("SLAMLOC", &Handler::handleMessage_slamloc, &handlerObject);

    while (0 == zcm.handle());

}

MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{
    ui->setupUi(this);
    std::thread Zcm_thread(zcm_func);
    Zcm_thread.detach();

    ReceiveMode_ = ReceiveMode::RTK;

    //SpeedMode
    ui->comboBox->addItem(QWidget::tr("BACK"));
    ui->comboBox->addItem(QWidget::tr("STOP"));
    ui->comboBox->addItem(QWidget::tr("VERYLOW"));
    ui->comboBox->addItem(QWidget::tr("LOW"));
    ui->comboBox->addItem(QWidget::tr("MID"));
    ui->comboBox->addItem(QWidget::tr("HIGH"));
    ui->comboBox->addItem(QWidget::tr("VERYHIGH"));

    //Mode
    ui->comboBox_2->addItem(QWidget::tr("车辆启动")); //START
    ui->comboBox_2->addItem(QWidget::tr("U字调头")); //UTURN
    ui->comboBox_2->addItem(QWidget::tr("人行道")); //PEDESTRIAN
    ui->comboBox_2->addItem(QWidget::tr("交叉路口")); //INTERSECT_WITHLIGHT
    ui->comboBox_2->addItem(QWidget::tr("红绿灯"));//INTERSECT_NOLIGHT
    ui->comboBox_2->addItem(QWidget::tr("无可靠地图"));//MAPFREE
    ui->comboBox_2->addItem(QWidget::tr("无障碍物探测"));//OBSTACLFREE
    ui->comboBox_2->addItem(QWidget::tr("上下坡"));//CLIMB
    ui->comboBox_2->addItem(QWidget::tr("雨雾区"));//FOGGY
    ui->comboBox_2->addItem(QWidget::tr("侧方位停车"));//SIDEPARKING
    ui->comboBox_2->addItem(QWidget::tr("正常行驶"));//NORMAL_DRIVING
    ui->comboBox_2->addItem(QWidget::tr("停车点"));//PARK
    ui->comboBox_2->addItem(QWidget::tr("桥"));//BRIDGE
    ui->comboBox_2->addItem(QWidget::tr("地下"));//UNDERGROUND

    //Event
    ui->comboBox_3->addItem(QWidget::tr("无事件"));//NOEVENT
    ui->comboBox_3->addItem(QWidget::tr("定时停止点"));//TIME_STOPPOINT
    ui->comboBox_3->addItem(QWidget::tr("停止点"));//CONDITION_STOPPOINT
    ui->comboBox_3->addItem(QWidget::tr("停止线"));//STOPLINE
    ui->comboBox_3->addItem(QWidget::tr("预留1"));//TODO1
    ui->comboBox_3->addItem(QWidget::tr("预留2"));//TODO2

    //OppositeSideMode
    ui->comboBox_4->addItem(QWidget::tr("左右均封"));//LEFT_YES_RIGHT_YES
    ui->comboBox_4->addItem(QWidget::tr("左不封右封"));//LEFT_NO_RIGHT_YES
    ui->comboBox_4->addItem(QWidget::tr("左封右不封"));//LEFT_YES_RIGHT_NO
    ui->comboBox_4->addItem(QWidget::tr("左右均不封"));//LEFT_NO_RIGHT_NO
    ui->comboBox_4->addItem(QWidget::tr("todo1"));//todo1
    data.clear();
    record_roadpoint = false;

    connect(ui->pushButton,SIGNAL(clicked()),this,SLOT(record_status_change()));
    connect(ui->pushButton_2,SIGNAL(clicked()),this,SLOT(save_roadmap()));
    connect(ui->pushButton_3,SIGNAL(clicked()),this,SLOT(SimplifyPointBtn()));
    connect(ui->pushButton_4,SIGNAL(clicked()),this,SLOT(ModeChange()));
    connect(ui->pushButton_5,SIGNAL(clicked()),this,SLOT(PokeEventPoint()));
    connect(ui->pushButton_6,SIGNAL(clicked()),this,SLOT(LocModeChange()));

    QTimer *timer = new QTimer(this);
    connect(timer,SIGNAL(timeout()),this,SLOT(showInfo()));
    timer -> start(10);
}

MainWindow::~MainWindow()
{
    delete ui;
}

void MainWindow::showInfo()
{
    structNAVINFO NAVI_local;
    NAVI_mtx.lock();
    NAVI_local = NAVI_temp;
    NAVI_mtx.unlock();
    QString rtk_utmX = "utmX: " + QString::number(NAVI_local.utmX, 10, 3);
    QString rtk_utmY = "utmY: " + QString::number(NAVI_local.utmY, 10, 3);
    QString rtk_mHeading = "mHeading: " + QString::number( NAVI_local.mHeading * 180.0 / M_PI, 10, 7);

    structSLAMLOC SLAMLOC_local;
    SLAMLOC_mtx.lock();
    SLAMLOC_local = SLAMLOC_temp;
    SLAMLOC_mtx.unlock();
    QString slam_utmX = "utmX: " + QString::number(SLAMLOC_local.x, 10, 3);
    QString slam_utmY = "utmY: " + QString::number(SLAMLOC_local.y, 10, 3);
    QString slam_mHeading = "mHeading: " + QString::number(SLAMLOC_local.mHeading * 180.0 / M_PI, 10, 7);

    if(record_roadpoint)
    {
        roadpoint data_temp, data_rtk_for_slam_test;
        if(event_record == true)
        {
            if( ReceiveMode_ == ReceiveMode::RTK ){
                data_temp = get_rtk_data(num,NAVI_local);
            }else{
                data_temp = get_slam_data(num,SLAMLOC_local);
            }

            data.push_back(data_temp);
            num ++;
            event_point_num ++;
            eventmode = 0;
            event_record = false;
        }else
        {
            if( ReceiveMode_ == ReceiveMode::RTK ){
                data_temp = get_rtk_data(num,NAVI_local);
            }else{
                data_temp = get_slam_data(num,SLAMLOC_local);
                data_rtk_for_slam_test = get_rtk_data(num,NAVI_local);
            }
            double dist = (data_temp.utmX - data_rtk_for_slam_test.utmX) * (data_temp.utmX - data_rtk_for_slam_test.utmX)
                    + (data_temp.utmY - data_rtk_for_slam_test.utmY) * (data_temp.utmY - data_rtk_for_slam_test.utmY);
            double delta_yaw = abs(data_temp.heading - data_rtk_for_slam_test.heading );
            cout << "rtk-slam distance: " << sqrt(dist) << "delta mheading: " << delta_yaw << endl;

            data.push_back(data_temp);
            num ++ ;
            // data_temp // todo
        }

    }
    ui->label->setText(rtk_utmX);
    ui->label_2->setText(rtk_utmY);
    ui->label_14->setText(rtk_mHeading);
    ui->label_3->setText("RTKStatus: " + QString::number(NAVI_local.mRTKStatus,10));
    ui->label_4->setText("point_num: " + QString::number(num,10));

    ui->label_10->setText(slam_utmX);
    ui->label_11->setText(slam_utmY);
    ui->label_15->setText(slam_mHeading);
    ui->label_12->setText("SLAMStatus: " + QString::number(SLAMLOC_local.SLAMStatus,10));

    string info = "-Mode INFO-> SpeedMode: " + std::to_string(speedmode)
            + "; Mode: " + std::to_string(mode)
            + "; Opposite Side Mode: " + std::to_string(opposite_side_mode)
            + ";"+ "\n"
            +"-Lane INFO-> LaneNum: " + std::to_string(laneno)
            +"; LaneSeq: " + std::to_string(laneseq)
            +"; LaneWidth: " + std::to_string(lanewidth)
            +";\n-EventPoiuntNum: "+ std::to_string(event_point_num)
            +";\n-LocalizationMode: "+ getReceiveModeName(ReceiveMode_)+";";
    ui->label_5->setText(QString::fromStdString(info));
}

void MainWindow::record_status_change()
{
    if(!bRecvNAVINFO)
    {
        QMessageBox::information(this, QString::fromLocal8Bit("警告"),
                                 QString::fromLocal8Bit("ZCM未接受NAVINFO"));


    }
    else
    {
        if(ui->pushButton->text() == "开始采集")
        {
            ui->pushButton->setText("暂停采集");
            record_roadpoint = true;
        }
        else if(ui->pushButton->text() == "暂停采集")
        {
            ui->pushButton->setText("开始采集");
            record_roadpoint = false;
        }
    }

}
void MainWindow::save_roadmap()
{
    if(record_roadpoint)
    {
        QMessageBox::information(this, QString::fromLocal8Bit("警告"),
                                 QString::fromLocal8Bit("请先点击“暂停采集”"));
    }
    else
    {
        QString fileName = QFileDialog::getSaveFileName(this,
                QString::fromLocal8Bit("文件另存为"),
                "",
                tr("Config Files (*.txt)"));
        ofstream f1(fileName.toStdString());           //打开文件用于写,若文件不存在就创建它
        if(!f1)return;                 //打开文件失败则结束运行

        f1<<"Id Lon Lat heading curvature mode SpeedMode EventMode OppositeSideMode LaneNum LaneSeq LaneWidth"<<endl;
        for(int i = 0;i<data.size();i++)
        {
            f1<< fixed<< setprecision(14)
              << i <<" "
              << data[i].lon <<" "
              << data[i].lat <<" "
              << data[i].heading <<" "
              << data[i].curvature <<" "
              << data[i].mode <<" "
              << data[i].speedmode <<" "
              << data[i].eventmode <<" "
              << data[i].opposite_side_mode <<" "
              << data[i].laneno <<" "
              << data[i].laneseq <<" ";

             f1 << data[i].lanewidth<<endl;     //使用插入运算符写文件内容
        }
        f1.close();                   //关闭文件
    }
}

void MainWindow::SimplifyPointBtn(){
    if(record_roadpoint)
    {
        QMessageBox::information(this, QString::fromLocal8Bit("警告"),
                                 QString::fromLocal8Bit("请先点击“暂停采集”"));
    }
    else
    {
        int num_before = data.size();
        PickPoint(data);
        int num_after = data.size();
        string msg = "point_num: " +  std::to_string(num_before) + " -> point_num: " + std::to_string(num_after);
        QMessageBox::information(this, QString::fromLocal8Bit("筛点完成"),
                                  QString::fromStdString(msg));
    }

}

void MainWindow::ModeChange()
{
    speedmode = ui->comboBox->currentIndex();
    mode = ui->comboBox_2->currentIndex();
    opposite_side_mode = ui->comboBox_4->currentIndex();
    laneno=ui->textEdit->toPlainText().toInt();
    laneseq=ui->textEdit_2->toPlainText().toInt();
    lanewidth=ui->textEdit_3->toPlainText().toFloat();
}

void MainWindow::LocModeChange()
{
    if(ReceiveMode_ == ReceiveMode::RTK){
        if(!bRecvSLAMLOC)
        {
            QMessageBox::information(this, QString::fromLocal8Bit("警告"),
                                     QString::fromLocal8Bit("ZCM未接受SLAMLOC"));


        }else{
            ReceiveMode_ = ReceiveMode::SLAM;
        }
    }else{
        if(!bRecvNAVINFO)
        {
            QMessageBox::information(this, QString::fromLocal8Bit("警告"),
                                     QString::fromLocal8Bit("ZCM未接受SLAMLOC"));


        }else{
            ReceiveMode_ = ReceiveMode::RTK;
        }
    }
}

void MainWindow::PokeEventPoint()
{
    eventmode = ui->comboBox_3->currentIndex();
    event_record = true;
}

MainWindow::roadpoint MainWindow::get_rtk_data(int num , structNAVINFO navi_data){
    roadpoint data_temp;
    data_temp.id = num;
    data_temp.lon = navi_data.mLon;
    data_temp.lat = navi_data.mLat;
    data_temp.utmX = navi_data.utmX;
    data_temp.utmY = navi_data.utmY;
    data_temp.heading = navi_data.mHeading;
    data_temp.curvature = navi_data.mCurvature;
    data_temp.mode = mode ;
    data_temp.speedmode = speedmode;
    data_temp.laneno = laneno;
    data_temp.laneseq = laneseq;
    data_temp.lanewidth = lanewidth;
    data_temp.eventmode = eventmode;
    data_temp.opposite_side_mode = opposite_side_mode;
    return data_temp;
}

MainWindow::roadpoint MainWindow::get_slam_data(int num , structSLAMLOC slam_data){
    TiEV::WGS84Coor latLonPinvert;//(lat, lon);//convert angle to WGS84Coor
    TiEV::UTMCoor coordinvert;
    coordinvert.x = slam_data.x;
    coordinvert.y = slam_data.y;
    latLonPinvert = UTMXYToLatLon(coordinvert);//convert WGS84Coor to UTMxy


    roadpoint data_temp;
    data_temp.id = num;
    data_temp.lon = latLonPinvert.lon.getDegree();;
    data_temp.lat = latLonPinvert.lat.getDegree();;
    data_temp.utmX = slam_data.x;
    data_temp.utmY = slam_data.y;
    data_temp.heading = slam_data.mHeading;
    data_temp.curvature = 0.0;
    data_temp.mode = mode ;
    data_temp.speedmode = speedmode;
    data_temp.laneno = laneno;
    data_temp.laneseq = laneseq;
    data_temp.lanewidth = lanewidth;
    data_temp.eventmode = eventmode;
    data_temp.opposite_side_mode = opposite_side_mode;
    return data_temp;
}

void MainWindow::PickPoint(std::vector <roadpoint>& data){
    double PointSpace = 0.5;//设定点间距变量
    roadpoint CurrentPoint, rpTemp;
    CurrentPoint = data[0];
    int i = 1;
    while (i < data.size())
    {
        rpTemp = data[i];//取下一个点

        if(rpTemp.eventmode != 0 )
        {
            CurrentPoint = data[i];
            i++;
        }
        else
        {
            TiEV::Angle latAngle1, lonAngle1,latAngle2, lonAngle2;
            latAngle1.setByDegree(CurrentPoint.lat);
            lonAngle1.setByDegree(CurrentPoint.lon);
            latAngle2.setByDegree(rpTemp.lat);
            lonAngle2.setByDegree(rpTemp.lon);

            TiEV::WGS84Coor latLonP1((TiEV::LAT)latAngle1, (TiEV::LON)lonAngle1);
            TiEV::WGS84Coor latLonP2((TiEV::LAT)latAngle2, (TiEV::LON)lonAngle2);
            TiEV::UTMCoor coord1 ,coord2;
            coord1 = TiEV::latLonToUTMXY(latLonP1);
            coord2 = TiEV::latLonToUTMXY(latLonP2);

    //        p1 = CoordinateConverter.LatLonToUTMXY(
    //            CoordinateConverter.DegToRad(CurrentPoint.lat),
    //            CoordinateConverter.DegToRad(CurrentPoint.lon));
    //        p2 = CoordinateConverter.LatLonToUTMXY(
    //            CoordinateConverter.DegToRad(rpTemp.Lat),
    //            CoordinateConverter.DegToRad(rpTemp.Lon));
            //p1 = CoordinateConverter.BLH2XYZ(CurrentPoint.Lat, CurrentPoint.Lon, 0);
            //p2 = CoordinateConverter.BLH2XYZ(rpTemp.Lat, rpTemp.Lon, 0);
            double dist = sqrt((coord2.x-coord1.x)*(coord2.x-coord1.x)+(coord2.y-coord1.y)*(coord2.y-coord1.y));
            if (dist < PointSpace)//如果间距小于设定值
            {
                data.erase(data.begin() + i);//删除当前点
            }
            else//大于设定距离
            {
                CurrentPoint = data[i];
                i++;
            }
        }
    }
}
