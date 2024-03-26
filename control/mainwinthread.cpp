#include "mainwinthread.h"
#include<QDebug>

mainwinthread::mainwinthread()
{
    this->motor = MotorCtr::getInstance();
}

void mainwinthread::getMotorNowPosition(float &position, int nAxis)
{
    this->motor->getNowPosition(position, nAxis);
}

bool mainwinthread::getMotorConnect()
{
    return this->motor->getConnect();
}


void mainwinthread::startRun(float dis)
{
    this->distance = dis;
    this->start();
}

//void mainwinthread::getFocusValue(Mat image,float& value)
//{
//    this->motor->getFocusValue(image,value);
//}

//Mat mainwinthread::QImage2Mat(const QImage &image)
//{
//    Mimg = this->motor->QImage2Mat(image);
//    return Mimg;
//}

void mainwinthread::run()
{
    float nowPosition = 0 ;
    this->motor->getNowPosition(nowPosition,0);
    qDebug()<<nowPosition;
    float dis = abs(nowPosition) - distance;
    qDebug()<<dis;
    bool blog = RUNREVERSAL;
    int m_nAxis = 0;
    if(dis>0)
    {
        blog = RUNFRONT;
        qDebug()<<"正转";
    }
    qDebug()<<"反转";
    this->motor->runMotor(RUNMOTION, blog, abs(dis), m_nAxis);
}
