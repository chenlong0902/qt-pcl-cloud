#ifndef LIDARDEF_H
#define LIDARDEF_H
#include <QObject>
#define M_PI		3.14159265358979323846
#define GRID_X_CENTER 50
#define CHANNEL_GRID_X  6
#define VDISTANCE  2
static const ushort UPPER_BANK = 0xeeff;
static const int TEMPERATURE_MIN = 31;
static const int SIZE_BLOCK = 100;
static const int RAW_SCAN_SIZE = 3;
static const int SCANS_PER_BLOCK = 32;
static const int BLOCK_DATA_SIZE = (SCANS_PER_BLOCK * RAW_SCAN_SIZE); //96

static const int PACKET_SIZE = 1248;
static const int BLOCKS_PER_PACKET = 12;
static const int PACKET_STATUS_SIZE = 4;

static const char packetHead[8] = {(char)0x55, (char)0xAA, (char)0x05, (char)0x0A, (char)0x5A, (char)0xA5, (char)0x50, (char)0xA0}; //包头
static const char packetTail[2] = {(char)0x00, (char)0xFF};
static const int RS32_SCANS_PER_FIRING = 32;
static const int TEMPERATURE_RANGE = 50;

static const int numOfLasers = 32;
static const int RS32_FIRINGS_PER_BLOCK = 1;
static const float RS32_DSR_TOFFSET = 3.0f;

static const float RS32_BLOCK_TDURATION = 50.0f;

static const float DISTANCE_MAX = 200.0f;        /**< meters */
static const float DISTANCE_MIN = 0.2f;        /**< meters */
static const float DISTANCE_RESOLUTION = 0.01f; /**< meters */
static const float DISTANCE_MAX_UNITS = (DISTANCE_MAX
                                         / DISTANCE_RESOLUTION + 1.0f);
using namespace std;


typedef struct SafeChannel {
    int sAngle;
    int eAngle;
    float sDistance;
    float eDistance;

    SafeChannel(){
        sDistance = 200;
        eDistance = 200;
    }
} SafeChannel_Stu;


typedef struct raw_block {
    uint16_t header;        ///< UPPER_BANK or LOWER_BANK
    uint8_t rotation_1;
    uint8_t rotation_2;     ///combine rotation1 and rotation2 together to get 0-35999, divide by 100 to get degrees
    uint8_t data[BLOCK_DATA_SIZE]; //96
} raw_block_t;

typedef struct raw_packet {
    raw_block_t blocks[BLOCKS_PER_PACKET];
} raw_packet_t;

union two_bytes {
    uint16_t uint;
    uint8_t bytes[2];
};

typedef struct rslidarPic {
    int column;
    int pos_count;
    float distance[32*12];
    float intensity[32*12];
    float azimuthforeachP[32*12];

} rslidarPic_t;

struct PointXYZI{
    double x;
    double y;
    double z;
    int intensity;
};


//Lidar表数据
struct stuLidarConfig{
    int maxCount;//滤波临界值（点数量）
    int safeSector;//安全扇区角度
    double radius;//滤波半径
    double landscape; //横向范围（左右各有landscape距离）
    double lengthways;//纵向（正前方）
    double heightLow;//激光雷达水平线以下
    double heightHigh;//激光雷达水平线以上

};
#endif // LIDARDEF_H
