#include "convercloudthread.h"
#include "lidarparsethread.h"

ConverCloudThread* ConverCloudThread::m_pThread = NULL;
ConverCloudThread* ConverCloudThread::Instance(){
    if(m_pThread == nullptr){
        m_pThread = new ConverCloudThread();
    }
    return m_pThread;
}


QMutex sMutex;
ConverCloudThread::ConverCloudThread()
{

}

void ConverCloudThread::run(){
    while(1){
        sMutex.lock();
        if(m_pointList.length() >0){
            pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

            cloud->width = LidarParseThread::Instance()->g_pointList.length();
            cloud->height = 1;
            cloud->is_dense = false;
            cloud->points.resize(cloud->width * cloud->height);

            for(int i = 0; i < LidarParseThread::Instance()->g_pointList.length(); i++){
                PointXYZI posXYZ = LidarParseThread::Instance()->g_pointList.at(i);
                cloud->points[i].x = posXYZ.x;
                cloud->points[i].y = posXYZ.y;
                cloud->points[i].z = posXYZ.z;
            }


            //方法一：直通滤波器对点云进行处理。
            pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_after_PassThrough(new pcl::PointCloud<pcl::PointXYZ>);
            pcl::PassThrough<pcl::PointXYZ> passthrough;
            passthrough.setInputCloud(cloud);//输入点云
            passthrough.setFilterFieldName("x");//对z轴进行操作
            passthrough.setFilterLimits(0.01, 0.5);//设置直通滤波器操作范围
            passthrough.filter(*cloud_after_PassThrough);//执行滤波，过滤结果保存在 cloud_after_PassThrough

            //方法四：条件滤波器(保留指定范围内的点)
            pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_after_Condition(new pcl::PointCloud<pcl::PointXYZ>);
            pcl::ConditionAnd<pcl::PointXYZ>::Ptr range_condition(new pcl::ConditionAnd<pcl::PointXYZ>());
            range_condition->addComparison(pcl::FieldComparison<pcl::PointXYZ>::ConstPtr(new
              pcl::FieldComparison<pcl::PointXYZ>("y", pcl::ComparisonOps::GT, 1)));  //GT表示大于等于
            range_condition->addComparison(pcl::FieldComparison<pcl::PointXYZ>::ConstPtr(new
              pcl::FieldComparison<pcl::PointXYZ>("y", pcl::ComparisonOps::LT, 100)));  //LT表示小于等于
            pcl::ConditionalRemoval<pcl::PointXYZ> condition;
            condition.setCondition(range_condition);
            condition.setInputCloud(cloud_after_PassThrough);                   //输入点云
            condition.setKeepOrganized(false);
            condition.filter(*cloud_after_Condition);


            //删除值为NAN的点
            pcl::PointCloud<pcl::PointXYZ>::iterator it = cloud_after_Condition->points.begin();
            while (it != cloud_after_Condition->points.end())
            {
                if(std::isnan(it->x) || std::isnan(it->y) || std::isnan(it->z)){
                    it = cloud_after_Condition->erase(it);
                }
                else{
                    ++it;
                }
            }

            //半径滤波
            pcl::RadiusOutlierRemoval<pcl::PointXYZ> sor;
            sor.setInputCloud(cloud_after_Condition);
            sor.setRadiusSearch(0.5);
            sor.setMinNeighborsInRadius(3);
            sor.setNegative(false);
            sor.filter(m_cloud);


            //扇区数组大小为180。 扇区逻辑编号为-90至-1,1至90
            float channelArray[180];
            memset(channelArray, 0, 180*sizeof(float));
            it = m_cloud.points.begin();
            while (it != m_cloud.points.end())
            {
                if(it->z < VDISTANCE){
                    grid[GRID_X_CENTER+((int)it->x)][(int)it->y] = 1;
                }

                float angle = atan(it->x/it->y)*180/M_PI;
                float distance = sqrt(pow(abs(it->x),2) + pow(abs(it->y),2));

                int sectionIndex = (int)angle + 89;
                if(sectionIndex == 89 && angle > 0){ //处理 -1<angle<1的角度转换
                    sectionIndex = 90;
                }

                if(channelArray[sectionIndex] == 0){
                    channelArray[sectionIndex] = distance;
                }
                else if(distance < channelArray[sectionIndex]){
                    channelArray[sectionIndex] = distance;
                }
                ++it;
            }
            channelArray[0] = 3;
            channelArray[120] = 3;
            channelArray[179] = 3;
            channelVec.clear();
            int channelCount = 0;

            float sDistance = 200;
            float eDistance = 200;

            int section;// 扇区编号从左往右依次为：-90,-89,...-2,-1,1,2,...89,90
            for(int i = 0; i < 180; i++){
                section = i<90?i-90:i-89;
                if(channelArray[i] == 0){

                    channelCount++;

                    if(i == 179 && channelCount >=6){
                        SafeChannel_Stu channel;
                        channel.sDistance = sDistance;
                        channel.eDistance = eDistance;

                        channel.eAngle = section;
                        channel.sAngle = channel.eAngle - channelCount;


                        channelVec.append(channel);
                        qDebug()<< "channel.sAngle = "<<channel.sAngle;
                        qDebug()<< "channel.eAngle = "<<channel.eAngle;
                        qDebug()<< "channelCount = "<<channelCount;
                    }
                }
                else{
                    if(channelCount < 6){
                        sDistance = channelArray[i];//如果未检测到通道，则实时记录通道的相邻障碍距离
                    }
                    else{
                        eDistance = channelArray[i];
                        SafeChannel_Stu channel;
                        channel.sDistance = sDistance;
                        channel.eDistance = eDistance;

                        channel.eAngle = section>0?section-1:section;
                        channel.sAngle = channel.eAngle - channelCount;
                        channelVec.append(channel);
                        qDebug()<< "channel.sAngle = "<<channel.sAngle;
                        qDebug()<< "channel.eAngle = "<<channel.eAngle;
                        qDebug()<< "channelCount = "<<channelCount;
                    }
                    channelCount = 0;
                    sDistance = channelArray[i];
                    eDistance = 200;
                }

            }

            emit sendCloud();
            m_pointList.clear();
        }
        sMutex.unlock();
    }
}

void ConverCloudThread::recvPoints(){
    //emit sendCloud();
    sMutex.lock();
    m_pointList.append(LidarParseThread::Instance()->g_pointList);
    sMutex.unlock();
}
