#include "cpcl.h"
#include "LidarDef.h"
#include <QDateTime>

CPcl::CPcl(QObject *parent) : QObject(parent)
{

}

void CPcl::init(){
    FILE *fp_txt;
    PointXYZI TxtPoint;

    fp_txt = fopen("za.txt", "r");
    if (fp_txt)
    {
        while (fscanf(fp_txt, "%lf %lf %lf", &TxtPoint.x, &TxtPoint.y, &TxtPoint.z) != EOF)
        {
            m_cloudPoint.push_back(TxtPoint);
        }
    }
    else
    {

    }

    long timplate1 = QDateTime::currentMSecsSinceEpoch();

    stuLidarConfig config;
    config.maxCount = 10;
    config.radius = 2;
    config.landscape = 200;
    config.lengthways = 200;
    config.heightLow = -1;
    config.heightHigh = 2;


    m_lidarFilter.setLidarConfig(config);
    m_lidarFilter.filter(m_cloudPoint);


    long timplate2 = QDateTime::currentMSecsSinceEpoch();
    long dif = timplate2 - timplate1;
    qDebug()<<"use time:"<<dif<<"ms";


    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

    cloud->width = m_cloudPoint.size();;
    cloud->height = 1;
    cloud->is_dense = false;
    cloud->points.resize(cloud->width * cloud->height);
    for (size_t i = 0; i < cloud->points.size(); ++i)
    {
        cloud->points[i].x = m_cloudPoint[i].x;
        cloud->points[i].y = m_cloudPoint[i].y;
        cloud->points[i].z = m_cloudPoint[i].z;
    }

    m_viewer.setCameraPosition(0,-8,10,0,0,0,0,0,0);
    m_viewer.addPointCloud(cloud);
    m_viewer.setBackgroundColor(0, 0, 0);
    m_viewer.spinOnce(100);

}
void CPcl::startTimer(){
    qDebug()<<"CPcl::startTimer()";
    m_timer = new QTimer(this);
    connect(m_timer,SIGNAL(timeout()), this, SLOT(onTimeOutSlot()));

    m_timer->setInterval(50);
    m_timer->start();
}

void CPcl::onTimeOutSlot(){
    qDebug()<<"CPcl::onTimeOutSlot()";


    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    // Fill in the cloud data
    cloud->width = m_cloudPoint.size();;
    cloud->height = 1;
    cloud->is_dense = false;
    cloud->points.resize(cloud->width * cloud->height);

    for (size_t i = 0; i < cloud->points.size(); ++i)
    {
        m_cloudPoint[i].x = m_cloudPoint[i].x + 0.2;

    }

    for (size_t i = 0; i < cloud->points.size(); ++i)
    {
        cloud->points[i].x = m_cloudPoint[i].x;
        cloud->points[i].y = m_cloudPoint[i].y;
        cloud->points[i].z = m_cloudPoint[i].z;
    }

    m_viewer.removeAllPointClouds();
    m_viewer.removeAllShapes();
    m_viewer.addPointCloud(cloud);
    m_viewer.spinOnce(50);

}
