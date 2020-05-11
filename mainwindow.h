#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include<QMutex>
#include<iostream>
#include<fstream>
#include<sstream>
#include<arpa/inet.h>
#include<unistd.h>
#include<sys/socket.h>
#include<sys/types.h>
#include<stdio.h>
#include<string.h>
#include<stdlib.h>
#include<vector>
#include<queue>
#include "CKobuki.h"
#include "Signal.h"
#include "cmath"
#include "rplidar.h"
#include "limits.h"
#include "map_loader.h"

#define HUGEVALINT 999999;

namespace Ui {
class MainWindow;
}

typedef struct{
    double Krc = 170; //320
    double Kts = 600;// rozdiel je v metroch rychlost chcem v mm >> chyba v metroch >> 100vky mm
    double Rcirc;
    double TransSp;
    const double max_trans_speed = 700;
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
    int x;
    int y;
    int value;
}MapPoint;


typedef struct{
    int mapsize = 400;
    int map[400][400];
    double resolution = 40 ; //  mm
    MapPoint mstart;
    MapPoint mfinish;
    worldPoint wstart;
    worldPoint wfinish;
}MapType;


typedef struct {
    double minDist2Target;
    double minDist2Fintarget;
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

    void mapNavigate();
    queue<worldPoint>cvrtMapPath2World(list<MapPoint> mappath);
    list<MapPoint> findPath(MapType map);
    MapType secureMap(MapType origmap);
    void fillInitPoint2Map(double xs, double ys);
    worldPoint mapCoord2World( int xm, int ym);
    MapPoint worldCoord2map(double xm, double ym);
    MapType floodMap();
    void floodAlgoritm();
    MapType loadRectMap(string filename);
    MapType createMap(MapType map);
    void fillMap(double distance, double angle);
    void writeMap(MapType map, string name);
    void writeMapCsV(MapType map, string name);

    worldPoint loadTargetCoord();
    worldPoint setPoint(double x, double y);
    MapPoint setPoint(int x,int y,int value);
    worldPoint findSecurePoint(double edgePointX, double edgePointY);

    double twoPoitDistance(double x1, double y1, double x2, double y2);
    double calcAngle(double x1, double y1, double x2, double y2);
    void rotateRobot();
    void goToTarget();
    bool isPathBlocked();
    double angleFormating(double fi);
    void angleDistFormating();
    void angleDistRegulator();
    worldPoint wallDetection();
    worldPoint wallFollowing();
    void updateLidarData();



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
    void on_pushButton_16_clicked();

    void on_pushButton_15_clicked();

    void on_pushButton_14_clicked();

    void on_pushButton_13_clicked();

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
     MapType lmapData;
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
     bool mapNavigateState = FALSE;
     bool mapingState = FALSE;
     bool mapResetState = FALSE;
     bool firsttime = FALSE;
     double shortestDistance = 0.0;
     double desiredAngle = 0.0;
     double desiredDistance = 0.0;
     double robotDistance = 0.0;
     double angleErr = 0.0;
     //TMapArea idmap;
     //map_loader mapLoader;
     Signal mysig;
     Regstruct regData;
     worldPoint newTarget;
     worldPoint finalTarget;
     list<MapPoint> mappath;
     queue<worldPoint> path;
     WallFollowData navigateData;

public slots:
     void setUiValues(Signal sig);
signals:
     void uiValuesChanged(Signal newsig); ///toto nema telo
};

#endif // MAINWINDOW_H
