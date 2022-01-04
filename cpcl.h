#ifndef CPCL_H
#define CPCL_H

#include <QObject>
#include <iostream>
#include <vector>
#include <QDebug>
#include <QThread>
#include <QUdpSocket>
#include <QList>
#include <QTimer>

#include <pcl/visualization/cloud_viewer.h>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>//pcd 读写类相关的头文件。
#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>  //直通滤波器头文件
#include <pcl/filters/voxel_grid.h>  //体素滤波器头文件
#include <pcl/filters/statistical_outlier_removal.h>   //统计滤波器头文件
#include <pcl/filters/conditional_removal.h>    //条件滤波器头文件
#include <pcl/filters/radius_outlier_removal.h>   //半径滤波器头文件

#include "LidarDef.h"
#include "clidarfilter.h"
using namespace std;


class CPcl : public QObject
{
    Q_OBJECT
public:
    explicit CPcl(QObject *parent = nullptr);
    void init();
    void startTimer();
private:
    QTimer *m_timer;
    vector<PointXYZI> m_cloudPoint;

    CLidarFilter m_lidarFilter;
    pcl::visualization::PCLVisualizer m_viewer;
public slots:
    void onTimeOutSlot();
};

#endif // CPCL_H
