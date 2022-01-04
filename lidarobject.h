#ifndef LIDAROBJECT_H
#define LIDAROBJECT_H

#include <QObject>
#include <iostream>
#include <vector>
#include <QDebug>
#include <QThread>

#include <QUdpSocket>
#include <QList>
#include "LidarDef.h"
using namespace std;


class LidarObject : public QObject
{
    Q_OBJECT
public:
    explicit LidarObject(QObject *parent = nullptr);
    QByteArray m_ba;
    QUdpSocket *m_Socket;
    void connectLidar();



signals:
    void sendPoints(QByteArray ba);
public slots:
    void receive();

};

#endif // LIDAROBJECT_H
