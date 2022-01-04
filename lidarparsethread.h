#ifndef LIDARPARSETHREAD_H
#define LIDARPARSETHREAD_H

#include <QThread>

#include <QThread>
#include <QObject>
#include <QVector>
#include "LidarDef.h"


class LidarParseThread : public QThread
{
    Q_OBJECT
public:
    static LidarParseThread *m_pThread;
    static LidarParseThread* Instance();
public:
    LidarParseThread(QObject * parent = 0);

    bool IsResuseRecv(){
        return m_bRefuseRecv;
    }
    void addData(QByteArray ba){
        baList.append(ba);
    }

    QVector<QByteArray> baList;
    rslidarPic_t pic;


    float temper = 31.0;
    float VERT_ANGLE[32];
    float HORI_ANGLE[32];
    float aIntensityCal[7][32];
    int g_ChannelNum[32][51];
    float CurvesRate[32];


    QList<PointXYZI> m_pointList;
    QList<PointXYZI> g_pointList;
    QList<vector<PointXYZI>> cloudPoints;
    QList<vector<PointXYZI>> gCloudPoints;
    long m_timestamp = 0;

    bool m_bRefuseRecv = false;
signals:
    void sendPoints();
public:

    void readAngle();
    void readCurves();
    void readChannel();
    void readCurves_Rate();

    void initStatus();
    int correctAzimuth(float azimuth_f, int passageway);
    float calibrateIntensity(float intensity, int calIdx, int distance);
    float pixelToDistance(int pixelValue, int passageway);
protected:
    void run(); // &#x65B0;&#x7EBF;&#x7A0B;&#x5165;&#x53E3;

private:

};

#endif // LIDARPARSETHREAD_H
