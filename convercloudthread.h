#ifndef CONVERCLOUDTHREAD_H
#define CONVERCLOUDTHREAD_H

#include <QThread>
#include <QVector>
#include "LidarDef.h"


class ConverCloudThread : public QThread
{
    Q_OBJECT

public:
    static ConverCloudThread *m_pThread;
    static ConverCloudThread* Instance();

    pcl::PointCloud<pcl::PointXYZ> m_cloud;
    QVector<SafeChannel> channelVec;

private:
    char grid[100][100];
    QList<PointXYZI> m_pointList;
public:
    ConverCloudThread();

protected:
    void run(); // &#x65B0;&#x7EBF;&#x7A0B;&#x5165;&#x53E3;

signals:
    void sendCloud();

public slots:
    void recvPoints();
};

#endif // CONVERCLOUDTHREAD_H
