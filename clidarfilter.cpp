#include "clidarfilter.h"

CLidarFilter::CLidarFilter(QObject *parent)
{

}
void CLidarFilter::setLidarConfig(stuLidarConfig config){
    m_maxCount = config.maxCount;
    m_radius = config.radius;
    m_landscape = config.landscape;
    m_lengthways = config.lengthways;
    m_heightLow = config.heightLow;
    m_heightHigh = config.heightHigh;
}

void CLidarFilter::filter(vector<PointXYZI> &cloudPoint){
    filterCloud_XOutlier(cloudPoint);
    filterCloud_YOutlier(cloudPoint);
    filterCloud_ZOutlier(cloudPoint);
    filterCloud_RadiusOutlier(cloudPoint);
}

void CLidarFilter::filterCloud_XOutlier(vector<PointXYZI> &cloudPoint){
    vector<PointXYZI>::iterator itPt = cloudPoint.begin();
    while (itPt != cloudPoint.end()) {
        if (itPt->x > m_landscape  ||
            itPt->x < -m_landscape)
        {
            itPt = cloudPoint.erase(itPt);
        }
        else {
            ++itPt;
        }
    }
}

void CLidarFilter::filterCloud_YOutlier(vector<PointXYZI> &cloudPoint){
    vector<PointXYZI>::iterator itPt = cloudPoint.begin();
    while (itPt != cloudPoint.end()) {
        if (itPt->y > m_lengthways ||
            itPt->y < 0.0)
        {
            itPt = cloudPoint.erase(itPt);
        }
        else {
            ++itPt;
        }
    }
}

void CLidarFilter::filterCloud_ZOutlier(vector<PointXYZI> &cloudPoint){
    vector<PointXYZI>::iterator itPt = cloudPoint.begin();
    while (itPt != cloudPoint.end()) {
        if (itPt->z > m_heightHigh ||
            itPt->z < m_heightLow)
        {
            itPt = cloudPoint.erase(itPt);
        }
        else {
            ++itPt;
        }
    }
}

void CLidarFilter::filterCloud_RadiusOutlier(vector<PointXYZI> &cloudPoint){

    //计算横向栅格数量
    double tmpValue = m_landscape/m_radius;
    if((int)tmpValue *m_radius < m_landscape ){
        row = ((int)(m_landscape/m_radius) + 1) * 2;
    }
    else{
        row = ((int)(m_landscape/m_radius)) * 2;
    }
    //计算纵向栅格数量
    tmpValue = m_lengthways/m_radius;
    if((int)tmpValue *m_radius < m_lengthways ){
        column = (int)(m_lengthways/m_radius) + 1;
    }
    else{
        column = (int)(m_lengthways/m_radius) + 1;
    }
    //计算竖向栅格数量
    tmpValue = abs(m_heightLow/m_radius);
    if((int)tmpValue *m_radius < abs(m_heightLow)){
        low = (int)abs(m_heightLow/m_radius) + 1;
    }
    else{
        low = (int)abs(m_heightLow/m_radius);
    }

    tmpValue = m_heightHigh/m_radius;
    if((int)tmpValue *m_radius < m_heightHigh){
        high = (int)(m_heightHigh/m_radius) + 1;
    }
    else{
        high = (int)(m_heightHigh/m_radius);
    }
    height = low+high;
    //立方体在空间中的x,y,z轴的索引
    int idRow,idColumn,idHeight;
    //把识别空间分割成 row*column*height个小立方体
    map<int,vector<PointXYZI>> cubeMap;
    map<int,vector<PointXYZI>>::iterator itCude;
    for(PointXYZI pt:cloudPoint){

        tmpValue = pt.x/m_radius;
        if(tmpValue < 0){
            idRow = row -1 +(int)tmpValue;
        }
        else{
            idRow = row + (int)tmpValue;
        }

        idColumn = (int)(pt.y/m_radius);

        tmpValue = pt.z/m_radius;
        if(tmpValue < 0){
            idHeight = low -1 +(int)tmpValue;
        }
        else{
            idHeight = low + (int)tmpValue;
        }

        itCude = cubeMap.find(idRow+row*idColumn+row*column*idHeight);
        if(itCude != cubeMap.end()){
            itCude->second.push_back(pt);
        }
        else{
            vector<PointXYZI> ptVec;
            ptVec.push_back(pt);
            int key = idRow+row*idColumn+row*column*idHeight;
            cubeMap.insert(pair<int,vector<PointXYZI>>(key,ptVec));
        }
    }

    itCude = cubeMap.begin();
    while(itCude != cubeMap.end()){
        if(itCude->second.size() < m_maxCount){

            vector<PointXYZI>::iterator itCurrent = itCude->second.begin();
            while (itCurrent != itCude->second.end()) {
                int idX = itCude->first%(row*column)%row;
                int idY = itCude->first%(row*column)/row;
                int idZ = itCude->first/(row*column);

                //如果当前cube的点数不足临界值，则从相邻的11个cube中查找
                int findCount = findNearbyPoint(idX,idY,idZ,*itCurrent,cubeMap);

                if (findCount < m_maxCount)
                {
                    itCurrent = itCude->second.erase(itCurrent);
                }
                else {
                    ++itCurrent;
                }
            }
        }
        itCude++;
    }

    cloudPoint.clear();
    itCude = cubeMap.begin();
    while(itCude != cubeMap.end()){
        for(PointXYZI pt:itCude->second)
        {
            cloudPoint.push_back(pt);
        }
        itCude++;
    }
}

double CLidarFilter::get3DDistance(PointXYZI p1, PointXYZI p2){
    double distance = sqrt(pow((p1.x-p2.x),2)+pow((p1.y-p2.y),2)+pow((p1.z-p2.z),2));
    return distance;
}

//查找相邻空间的临界点
int CLidarFilter::findNearbyPoint(int idX, int idY, int idZ, PointXYZI curPoint, map<int,vector<PointXYZI>> &cubeMap){
    int findCount = 0;
    //如果当前cube的点数不足临界值，则从相邻的11个cube中查找
    for(int x = (idX-1); x <= (idX+1); x++){
        for(int y = (idY-1); y <= (idY+1); y++){
            for(int z = (idZ-1); z <= (idZ+1); z++){
                int key = x+row*y+row*column*z;
                map<int,vector<PointXYZI>>::iterator it = cubeMap.find(key);
                if(it != cubeMap.end())
                {
                    for(PointXYZI pt:it->second){
                        double distance = get3DDistance(curPoint,pt);
                        if(distance < m_radius){
                            findCount++;
                        }
                    }
                }
            }
        }
    }
    return findCount;
}
