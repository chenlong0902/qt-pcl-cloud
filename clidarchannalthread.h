#ifndef CLIDARUDP2CLOUDTHREAD_H
#define CLIDARUDP2CLOUDTHREAD_H

#include <QThread>
#include <QVector>
#include <QVariant>
#include <QList>
#include "LidarDef.h"
#include "clidarfilter.h"
Q_DECLARE_METATYPE(QVector<SafeChannel>);


class CLidarChannalThread : public QThread
{
    Q_OBJECT
public:

    static CLidarChannalThread& Instance();

    CLidarChannalThread();

public:

    void readAngle();
    void readCurves();
    void readChannel();
    void readCurves_Rate();

    void initStatus();
    int correctAzimuth(float azimuth_f, int passageway);
    float calibrateIntensity(float intensity, int calIdx, int distance);
    float pixelToDistance(int pixelValue, int passageway);
    bool IsResuseRecv(){return m_bRefuseRecv;}
    void addData(QByteArray ba){baList.append(ba);}
public:

private:
    void run();
    void createSafeChannal(vector<PointXYZI> &m_pointList);

signals:
    void sigChannal(QVariant channalList);

private:
    CLidarFilter m_filter;

    QVector<QByteArray> baList;
    rslidarPic_t pic;

    CLidarFilter m_lidarFilter;
    stuLidarConfig m_lidarConfig;
    float temper = 31.0;
    float VERT_ANGLE[32];
    float HORI_ANGLE[32];
    float aIntensityCal[7][32];
    int g_ChannelNum[32][51];
    float CurvesRate[32];


    vector<PointXYZI> m_pointList;

    QList<vector<PointXYZI>> cloudPoints;
    QList<vector<PointXYZI>> gCloudPoints;

    long m_timestamp = 0;
    bool m_bRefuseRecv = false;

    float lastAzimuth = -1;
    float addedAzimuth = 0;


    char grid[100][100];
    vector<PointXYZI> g_pointList;
    QVector<SafeChannel> channelVec;

    int m_maxCount;//滤波临界值（点数量）
    double m_radius;//滤波半径
    double m_landscape; //横向范围（左右各有landscape距离）
    double m_lengthways;//纵向（正前方）
    double m_heightLow;//激光雷达水平线以下
    double m_heightHigh;//激光雷达水平线以上

};

#endif // CLIDARUDP2CLOUDTHREAD_H
