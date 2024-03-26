#include "motorthread.h"
#include "myDefine.h"

MotorThread::MotorThread()
{
    this->motor = MotorCtr::getInstance();
    this->stopFlage = 0;
}
//停止线程
void MotorThread::Stop()
{
    this->stopFlage = 1;
}

void MotorThread::run()
{
    while (true) {
        this->motor->positionBorderDeal();
        if(this->stopFlage)
        {
            break;
        }
        msleep(100);
    }
    this->stopFlage = 0;
}

