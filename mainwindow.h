#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include<QMutex>
#include<iostream>
#include<arpa/inet.h>
#include<unistd.h>
#include<sys/socket.h>
#include<sys/types.h>
#include<stdio.h>
#include<string.h>
#include<stdlib.h>
#include<vector>
#include "CKobuki.h"
#include "Signal.h"
#include "cmath"
#include "rplidar.h"
#include "limits.h"

namespace Ui {
class MainWindow;
}

typedef struct{
    double Krc = 320;
    double Kts = 400;// rozdiel je v metroch rychlost chcem v mm >> chyba v metroch >> 100vky mm
    double Rcirc;
    double TransSp;
    const double max_trans_speed = 400;
}Regstruct;

typedef struct {
    //najblizsi bod
    double minDcrit;
    double minDist;
    double minAngle;
    double forminAngle;
    double minX;
    double minY;
    //lavy kraj
    double DcritL;
    double DistL;
    double AngleL;
    double formAngleL;
    double XL;
    double YL;
    //pravy kraj
    double DcritR;
    double DistR;
    double AngleR;
    double formAngleR;
    double XR;
    double YR;

    double maxDistL;
    double maxAngleL;
    double maxXL;
    double maxYL;

    double maxDistR;
    double maxAngleR;
    double maxXR;
    double maxYR;

    bool minPoint;
    bool minPointL;
    bool minPointR;
    bool maxPointL;
    bool maxPointR;

    double minDistT;
    double minAngleT;
    double forminAngleT;
    double minXT;
    double minYT;
    double minPointT;
}LidarData4Reg;


typedef struct{
    double x;
    double y;
    double fi;
    double dist;
}worldPoint;


typedef struct{
    int mapsize = 320;
    int map[320][320];
}MapType;


typedef struct {
    double minDist2Target;

}WallFollowData;

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit MainWindow(QWidget *parent = 0);
    ~MainWindow();
    void robotprocess();
    void laserprocess();
    void processThisLidar(LaserMeasurement &laserData);
    void encDiff();
    void navigation();

    void createMap();
    void fillMap(double distance, double angle);
    void writeMap();

    worldPoint loadTargetCoord();
    worldPoint setPoint(double x, double y);
    worldPoint findSecurePoint(double edgePointX, double edgePointY);

    double twoPoitDistance(double x1, double y1, double x2, double y2);
    double calcAngle(double x1, double y1, double x2, double y2);
    void rotateRobot();
    void goToTarget();
    bool isPathBlocked();
    double angleFormating(double fi);
    void angleDistFormating();
    void angleDistRegulator();
    void wallDetection();
    void wallFollowing();



    void processThisRobot();
    pthread_t robotthreadHandle; // handle na vlakno
    int robotthreadID;  // id vlakna
    static void *robotUDPVlakno(void *param)
    {
        ((MainWindow*)param)->robotprocess();
        return 0;
    }
    pthread_t laserthreadHandle; // handle na vlakno
    int laserthreadID;  // id vlakna
    static void *laserUDPVlakno(void *param)
    {
        ((MainWindow*)param)->laserprocess();

        return 0;
    }
    //veci na broadcast laser
    struct sockaddr_in las_si_me, las_si_other,las_si_posli;

    int las_s,  las_recv_len;
    unsigned int las_slen;
    //veci na broadcast robot
    struct sockaddr_in rob_si_me, rob_si_other,rob_si_posli;

    int rob_s,  rob_recv_len;
    unsigned int rob_slen;


    QMutex mutex;

private slots:
    void on_pushButton_12_clicked();

    void on_pushButton_11_clicked();

    void on_pushButton_10_clicked();

    void on_pushButton_9_clicked();

    void on_pushButton_2_clicked();

    void on_pushButton_3_clicked();

    void on_pushButton_6_clicked();

    void on_pushButton_5_clicked();

    void on_pushButton_4_clicked();

private:
    Ui::MainWindow *ui;
     void paintEvent(QPaintEvent *event);// Q_DECL_OVERRIDE;
     int updateLaserPicture;
     LaserMeasurement copyOfLaserData;
     LidarData4Reg lD4R;
     std::string ipaddress;
     CKobuki robot;
     TKobukiData robotdata;
     TShellData robotshell;
     MapType mapData;
     int datacounter;
     double pEncL,pEncR,startEncL,startEncR,distanceL,distanceR,pDistanceL,pDistanceR;
     double fi,pFi,x,y,fip = 0.0;
     bool startState = FALSE;
     bool rotateState = FALSE;
     bool translateState = FALSE;
     bool wallFollowState = FALSE;
     bool wallDetectionState = FALSE;
     bool navigationState = FALSE;
     bool changedTargetState = FALSE;
     double shortestDistance = 0.0;
     double desiredAngle = 0.0;
     double desiredDistance = 0.0;
     double robotDistance = 0.0;
     double angleErr = 0.0;
     Signal mysig;
     Regstruct regData;
     worldPoint newTarget;
     worldPoint finalTarget;

public slots:
     void setUiValues(Signal sig);
signals:
     void uiValuesChanged(Signal newsig); ///toto nema telo
};

#endif // MAINWINDOW_H
