#include "lidarobject.h"
#include "LidarDef.h"
#include "clidarchannalthread.h"

LidarObject::LidarObject(QObject *parent) : QObject(parent)
{


    CLidarChannalThread::Instance().start();


    m_Socket = new QUdpSocket(this);
    m_Socket->bind(QHostAddress::Any, 6699, QUdpSocket::ShareAddress);
    connect(m_Socket, SIGNAL(readyRead()), this, SLOT(receive()));


}


void LidarObject::connectLidar(){


}

void LidarObject::receive(){

    while(m_Socket->hasPendingDatagrams())
    {
        m_ba.resize(m_Socket->pendingDatagramSize());
        m_Socket->readDatagram(m_ba.data(), m_ba.size());
        if(CLidarChannalThread::Instance().IsResuseRecv())
        {
            continue;
        }
        int size = m_ba.length();
        if(m_ba.at(0) == packetHead[0] &&
            m_ba.at(1) == packetHead[1] &&
            m_ba.at(2) == packetHead[2] &&
            m_ba.at(3) == packetHead[3] &&
            m_ba.at(4) == packetHead[4] &&
            m_ba.at(5) == packetHead[5] &&
            m_ba.at(6) == packetHead[6] &&
            m_ba.at(7) ==packetHead[7] &&
            m_ba.at(size-2) == packetTail[0] &&
            m_ba.at(size-1) == packetTail[1])
        {
            QByteArray tmpba = m_ba.mid(42,size-42-6);
            CLidarChannalThread::Instance().addData(tmpba);
        }

    }

}



