#include "lidarparsethread.h"

#include <math.h>
#include <QDebug>
#include <QFile>
#include <QDateTime>
#include "lidarobject.h"
LidarParseThread* LidarParseThread::m_pThread = NULL;
LidarParseThread* LidarParseThread::Instance(){
    if(m_pThread == nullptr){
        m_pThread = new LidarParseThread();
    }
    return m_pThread;
}

int isABPacket(int distance) {
    int ABflag = 0;
    if ((distance & 32768) != 0) {
        ABflag = 1; // B
    } else {
        ABflag = 0;// A
    }
    return ABflag;
}
int estimateTemperature(float Temper) {
    int temp = (int)floor(Temper + 0.5);
    if (temp < TEMPERATURE_MIN) {
        temp = TEMPERATURE_MIN;
    } else if (temp > TEMPERATURE_MIN + TEMPERATURE_RANGE) {
        temp = TEMPERATURE_MIN + TEMPERATURE_RANGE;
    }

    return temp;
}

LidarParseThread::LidarParseThread(QObject *parent)
{
    pic.column = 0;
    pic.pos_count = 0;

    initStatus();

    readAngle();
    readChannel();
    readCurves();
    readCurves_Rate();

    m_timestamp = QDateTime::currentMSecsSinceEpoch();

}

void LidarParseThread::initStatus(){
    m_timestamp = QDateTime::currentMSecsSinceEpoch();
    cloudPoints.clear();
    m_pointList.clear();
}

void LidarParseThread::run()
{
    float lastAzimuth = -1;
    float addedAzimuth = 0;
    while (true) {
        long tmpTimestamp = QDateTime::currentMSecsSinceEpoch();
        if((tmpTimestamp - m_timestamp) < 100){
            m_bRefuseRecv = true;
            baList.clear();
            continue;
        }
        m_bRefuseRecv = false;
        if(baList.size() > 0){
            QByteArray ba = baList.at(0);
            pic.pos_count = 0;
            pic.column = 0;
            raw_packet_t packet;
            memcpy(&packet,(char*)ba.data(),ba.length());
            for(int block = 0; block < BLOCKS_PER_PACKET; block++){
                float azimuth = packet.blocks[block].rotation_1*256 + packet.blocks[block].rotation_2;

                {
                    //&#x8BB0;&#x5F55;&#x7D2F;&#x8BA1;&#x89D2;&#x5EA6;
                    if(lastAzimuth < 0){
                        lastAzimuth = azimuth;
                    }
                    else{
                        float added = azimuth - lastAzimuth;
                        if(added < 0){
                            added =  36000 - lastAzimuth + azimuth;
                        }
                        addedAzimuth += added;
                        lastAzimuth = azimuth;
                    }
                }

                //&#x5982;&#x679C;&#x6536;&#x96C6;&#x4E86;360&#x5EA6;&#x7684;&#x6570;&#x636E;&#xFF0C;&#x5219;&#x5C06;&#x6570;&#x636E;&#x53D1;&#x7ED9;"&#x4E91;&#x5904;&#x7406;&#x7EBF;&#x7A0B;",&#x5E76;break;
                if(addedAzimuth > 36000){
                    lastAzimuth = -1;
                    addedAzimuth = 0;
                    g_pointList.clear();
                    for(int i = 0; i < m_pointList.length(); i++){
                        g_pointList.append(m_pointList[i]);
                    }
                    qDebug() << "emit sendPoints()";
                    emit sendPoints();
                    initStatus();
                    break;
                }

                float azimuth_diff;
                float azimuth_corrected_f;
                int azimuth_corrected;
                float intensity;
                if (block < (BLOCKS_PER_PACKET - 1))//12
                {
                    int azi1, azi2;
                    azi1 = 256 * packet.blocks[block + 1].rotation_1 + packet.blocks[block + 1].rotation_2;
                    azi2 = 256 * packet.blocks[block].rotation_1 + packet.blocks[block].rotation_2;
                    azimuth_diff = (float) ((36000 + azi1 - azi2) % 36000);

                    if (azimuth_diff <= 0.0 || azimuth_diff > 25.0) {
                        continue;
                    }
                } else {
                    int azi1, azi2;
                    azi1 = 256 * packet.blocks[block].rotation_1 + packet.blocks[block].rotation_2;
                    azi2 = 256 * packet.blocks[block - 1].rotation_1 + packet.blocks[block - 1].rotation_2;
                    azimuth_diff = (float) ((36000 + azi1 - azi2) % 36000);

                    if (azimuth_diff <= 0.0 || azimuth_diff > 25.0) {
                        continue;
                    }
                }
                //qDebug() << "cur direction:" + QString::number(azimuth/100);

                union two_bytes tmp_flag;
                tmp_flag.bytes[1] = packet.blocks[block].data[0];
                tmp_flag.bytes[0] = packet.blocks[block].data[1];
                int ABflag = isABPacket(tmp_flag.uint);

                int k = 0;
                int index;
                for (int dsr = 0; dsr < RS32_SCANS_PER_FIRING * RS32_FIRINGS_PER_BLOCK; dsr++, k += RAW_SCAN_SIZE)//16   3
                {
                    if (ABflag == 1 && dsr < 16) {
                        index = k + 48;
                    } else if (ABflag == 1 && dsr >= 16) {
                        index = k - 48;
                    } else {
                        index = k;
                    }

                    int point_count = pic.column * SCANS_PER_BLOCK + dsr;
                    int dsr_temp;
                    if (dsr >= 16) { dsr_temp = dsr - 16; }
                    else { dsr_temp = dsr; }
                    azimuth_corrected_f = azimuth + (azimuth_diff * ((dsr_temp * RS32_DSR_TOFFSET)) / RS32_BLOCK_TDURATION);
                    azimuth_corrected = correctAzimuth(azimuth_corrected_f, dsr);

                    pic.azimuthforeachP[pic.pos_count] = azimuth_corrected;

                    union two_bytes tmp;
                    tmp.bytes[1] = packet.blocks[block].data[index];
                    tmp.bytes[0] = packet.blocks[block].data[index + 1];

                    int ab_flag_in_block = isABPacket(tmp.uint);

                    int distance = tmp.uint - ab_flag_in_block * 32768;

                    // read intensity
                    intensity = (float) packet.blocks[block].data[index + 2];
                    intensity = calibrateIntensity(intensity, dsr, distance);

                    float distance2 = pixelToDistance(distance, dsr);
                    distance2 = distance2 * DISTANCE_RESOLUTION;

                    pic.distance[point_count] = distance2/2;
                    pic.intensity[point_count] = intensity;

                    pic.pos_count++;
                    //qDebug()<<"azimuth:"<<QString::number(azimuth_corrected/100) << "  cur dustance:" + QString::number(distance2);
                }

                pic.column++;
            }

            for (int block_num = 0; block_num < pic.column; block_num++) {

                vector<PointXYZI> vPos(RS32_SCANS_PER_FIRING);

                for (int dsr = 0; dsr < RS32_SCANS_PER_FIRING * RS32_FIRINGS_PER_BLOCK; dsr++) {
                    int point_count = block_num * SCANS_PER_BLOCK + dsr;
                    float dis = pic.distance[point_count];
                    float arg_horiz = pic.azimuthforeachP[point_count] / 18000 * M_PI;
                    float intensity = pic.intensity[point_count];
                    float arg_vert = VERT_ANGLE[dsr];
                    PointXYZI point;
                    if (dis > DISTANCE_MAX || dis < DISTANCE_MIN)  //invalid data
                    {
                        point.x = NAN;
                        point.y = NAN;
                        point.z = NAN;
                        point.intensity = 0;
                    } else {
                        point.x = dis * cos(arg_vert) * sin(arg_horiz); //&#x5DE6;&#x53F3;
                        point.y = dis * cos(arg_vert) * cos(arg_horiz); //&#x8DDD;&#x79BB;
                        point.z = dis * sin(arg_vert); //&#x9AD8;
                        point.intensity = intensity;
                    }

                    m_pointList.append(point);
                }


                //cloudPoints.append(vPos);
            }
            baList.remove(0);
        }
    }
}

int LidarParseThread::correctAzimuth(float azimuth_f, int passageway) {
    int azimuth;
    if (azimuth_f > 0.0 && azimuth_f < 3000.0) {
        azimuth_f = azimuth_f + HORI_ANGLE[passageway] + 36000.0f;
    } else {
        azimuth_f = azimuth_f + HORI_ANGLE[passageway];
    }
    azimuth = (int)azimuth_f;
    azimuth %= 36000;

    return azimuth;
}
//&#x6821;&#x51C6;&#x53CD;&#x5C04;&#x5F3A;&#x5EA6;&#x503C;
float LidarParseThread::calibrateIntensity(float intensity, int calIdx, int distance)
{
    int algDist;
    int sDist;
    int uplimitDist;
    float realPwr;
    float refPwr;
    float tempInten;
    float distance_f;
    float endOfSection1;

    int temp = estimateTemperature(temper);

    realPwr = std::max( (float)( intensity / (1+(temp-TEMPERATURE_MIN)/24.0f) ), 1.0f );
    // realPwr = intensity;

    // transform the one byte intensity value to two byte
    if ((int) realPwr < 126)
        realPwr = realPwr * 4.0f;
    else if ((int) realPwr >= 126 && (int) realPwr < 226)
        realPwr = (realPwr - 125.0f) * 16.0f + 500.0f;
    else
        realPwr = (realPwr - 225.0f) * 256.0f + 2100.0f;

    int indexTemper = estimateTemperature(temper) - TEMPERATURE_MIN;
    uplimitDist = g_ChannelNum[calIdx][indexTemper] + 20000;
    //limit sDist
    sDist = (distance > g_ChannelNum[calIdx][indexTemper]) ? distance : g_ChannelNum[calIdx][indexTemper];
    sDist = (sDist < uplimitDist) ? sDist : uplimitDist;
    //minus the static offset (this data is For the intensity cal useage only)
    algDist = sDist - g_ChannelNum[calIdx][indexTemper];

    // calculate intensity ref curves
    float refPwr_temp = 0.0f;
    int order = 3;
    endOfSection1 = 500.0f;
    distance_f = (float)algDist;
    if(distance_f <= endOfSection1)
    {
      refPwr_temp = aIntensityCal[0][calIdx] * exp(aIntensityCal[1][calIdx] -
      aIntensityCal[2][calIdx] * distance_f/100.0f) + aIntensityCal[3][calIdx];
    //   printf("a-calIdx=%d,distance_f=%f,refPwr=%f\n",calIdx,distance_f,refPwr_temp);
    }
    else
    {
      for(int i = 0; i < order; i++)
      {
        refPwr_temp +=aIntensityCal[i+4][calIdx]*(pow(distance_f/100.0f,order-1-i));
      }
      // printf("b-calIdx=%d,distance_f=%f,refPwr=%f\n",calIdx,distance_f,refPwr_temp);
    }

    refPwr = std::max(std::min(refPwr_temp,500.0f),4.0f);

    tempInten = (51* refPwr) / realPwr;
    if(numOfLasers == 32){
        tempInten = tempInten * CurvesRate[calIdx];
    }
    tempInten = (int) tempInten > 255 ? 255.0f : tempInten;
    return tempInten;
}

float LidarParseThread::pixelToDistance(int pixelValue, int passageway) {
    float DistanceValue;
    int indexTemper = estimateTemperature(temper) - TEMPERATURE_MIN;
    if (pixelValue <= g_ChannelNum[passageway][indexTemper]) {
        DistanceValue = 0.0;
    } else {
        DistanceValue = (float) (pixelValue - g_ChannelNum[passageway][indexTemper]);
    }
    return DistanceValue;
}

void LidarParseThread::readAngle(){
    QFile file("angle.csv");
    if(!file.open(QIODevice::ReadOnly | QIODevice::Text))
    {
       qDebug() << "no file";
       return;
    }

    QTextStream in(&file);
    int loop =0;
    while(!in.atEnd())
    {
        QString fileLine = in.readLine();
        QStringList list = fileLine.split(",", QString::SkipEmptyParts);
        VERT_ANGLE[loop] = list.at(0).toFloat();
        VERT_ANGLE[loop] = VERT_ANGLE[loop] / 180 * M_PI;

        HORI_ANGLE[loop] = list.at(1).toFloat();
        HORI_ANGLE[loop] = HORI_ANGLE[loop] * 100;
        loop++;
    }
    file.close();
}
void LidarParseThread::readCurves(){
    QString path = "curves.csv";
    FILE *f_inten = fopen("curves.csv", "r");
    int loopi = 0;
    int loopj = 0;

    if (!f_inten) {
        qDebug()<<"curves.csv" << " does not exist";
    }
    else
    {
        while (!feof(f_inten)) {
            float a[32];
            loopi++;
            if (loopi > 7)
                break;
            fscanf(f_inten,
                   "%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f\n",
                   &a[0], &a[1], &a[2], &a[3], &a[4], &a[5], &a[6], &a[7], &a[8], &a[9], &a[10], &a[11], &a[12],
                   &a[13],
                   &a[14], &a[15], &a[16], &a[17], &a[18], &a[19], &a[20], &a[21], &a[22], &a[23], &a[24],
                   &a[25], &a[26], &a[27],
                   &a[28], &a[29], &a[30], &a[31]);

            for (loopj = 0; loopj < numOfLasers; loopj++) {
                aIntensityCal[loopi - 1][loopj] = a[loopj];
            }
        }
        fclose(f_inten);
    }
}
void LidarParseThread::readChannel(){
    FILE *f_channel = fopen("ChannelNum.csv", "r");
    if (!f_channel) {
        qDebug()<<"ChannelNum does not exist";
    }
    else {
        int loopl = 0;
        int loopm = 0;
        int c[51];
        int tempMode = 1;
        while (!feof(f_channel)) {
            fscanf(f_channel,
                   "%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d\n",
                   &c[0], &c[1], &c[2], &c[3], &c[4], &c[5], &c[6], &c[7], &c[8], &c[9], &c[10], &c[11], &c[12],
                   &c[13], &c[14], &c[15],
                   &c[16], &c[17], &c[18], &c[19], &c[20], &c[21], &c[22], &c[23], &c[24], &c[25], &c[26], &c[27],
                   &c[28], &c[29], &c[30],
                   &c[31], &c[32], &c[33], &c[34], &c[35], &c[36], &c[37], &c[38], &c[39], &c[40],
                   &c[41], &c[42], &c[43], &c[44], &c[45], &c[46], &c[47], &c[48], &c[49], &c[50]);

            if (c[1] < 100 || c[1] > 3000)
            {
                tempMode = 0;
            }
            for (loopl = 0; loopl < TEMPERATURE_RANGE+1; loopl++) {
                g_ChannelNum[loopm][loopl] = c[tempMode * loopl];
            }
            loopm++;
            if (loopm > (numOfLasers - 1)) {
                break;
            }
        }
        fclose(f_channel);
    }
}
void LidarParseThread::readCurves_Rate(){
    FILE *f_curvesRate = fopen("CurveRate.csv", "r");
    if(!f_curvesRate)
    {
        qDebug()<<"CurveRate does not exist";
    }
    else
    {
        int loopk = 0;
        while(!feof(f_curvesRate))
        {
            fscanf(f_curvesRate, "%f\n", &CurvesRate[loopk]);
            loopk++;
            if(loopk > (numOfLasers-1)) break;
        }
        fclose(f_curvesRate);
    }
}
