#ifndef CLIDARFILTER_H
#define CLIDARFILTER_H
#include <QObject>
#include <cmath>
#include <vector>
#include <map>
#include "LidarDef.h"
using namespace std;
class CLidarFilter:public QObject
{
    Q_OBJECT
public:
    CLidarFilter(QObject *parent=Q_NULLPTR);
    void setLidarConfig(stuLidarConfig config);
    void filter(vector<PointXYZI> &cloudPoint);
    void filterCloud_RadiusOutlier(vector<PointXYZI> &cloudPoint);
    void filterCloud_XOutlier(vector<PointXYZI> &cloudPoint);
    void filterCloud_YOutlier(vector<PointXYZI> &cloudPoint);
    void filterCloud_ZOutlier(vector<PointXYZI> &cloudPoint);
private:
    double get3DDistance(PointXYZI p1, PointXYZI p2);
    int findNearbyPoint(int idX, int idY, int idZ, PointXYZI curPoint, map<int,vector<PointXYZI>> &cubeMap);
private:
    int row;
    int column;
    int low;
    int high;
    int height;//三维空间栅格数量

    int m_maxCount;//滤波临界值（点数量）
    double m_radius;//滤波半径
    double m_landscape; //横向范围（左右各有landscape距离）
    double m_lengthways;//纵向（正前方）
    double m_heightLow;//激光雷达水平线以下
    double m_heightHigh;//激光雷达水平线以上
};

#endif // CLIDARFILTER_H
