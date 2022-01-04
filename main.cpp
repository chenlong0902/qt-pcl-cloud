#include <QCoreApplication>
#include "cpcl.h"
#include <QDebug>
using namespace std;


int main(int argc, char *argv[])
{
    QCoreApplication a(argc, argv);

    CPcl pcl;
    pcl.init();
    qDebug()<<"pcl.init()";
    pcl.startTimer();
    qDebug()<<"pcl.startTimer()";

    return a.exec();
}
