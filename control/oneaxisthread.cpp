#include "oneaxisthread.h"
#include <QMessageBox>


OneAxisThread* OneAxisThread::myself = nullptr;

OneAxisThread *OneAxisThread::getInstance()
{
    if(OneAxisThread::myself == nullptr)
    {
        OneAxisThread::myself = new OneAxisThread();
    }
    return OneAxisThread::myself;
}

float OneAxisThread::getMotorSpeed()
{
    return this->motor->getSpeed();
}

bool OneAxisThread::getMotorConnect()
{
    return this->motor->getConnect();
}

void OneAxisThread::getMotorNowPosition(float &position, int nAxis)
{
    this->motor->getNowPosition(position, nAxis);
}

void OneAxisThread::setMotorSpeed(float speed)
{
    this->motor->setSpeed(speed);
}

void OneAxisThread::motorAdd(int axis)
{
    //电机运行
    this->motor->runMotor(RUNCONTINUE, RUNFRONT, 0, axis);
}

void OneAxisThread::motorReduce(int axis)
{
    //电机运行
    this->motor->runMotor(RUNCONTINUE, RUNREVERSAL, 0, axis);
}

void OneAxisThread::StopMotor(int axis)
{
    this->motor->stopMotor(axis);
}

void OneAxisThread::setPosition(float position, int model)
{
    this->motor->setRecordPosition_X(position, model);
}

void OneAxisThread::recordPosition(int axis, int model)
{
    float position = 0;
    this->motor->getNowPosition(position, axis);
    this->motor->setRecordPosition_X(position, model);
}

void OneAxisThread::startRun(int axis)
{
    this->m_axis = axis;
    this->threadStop = false;
    this->start();
}

void OneAxisThread::axisPosZero(int axis)
{
    this->motor->setDpos(0, axis);
}

void OneAxisThread::Stop()
{
    this->threadStop = true;
}

void OneAxisThread::run()
{
    //先将轴运动到起始位置
    float pos = 0, posStart = 0, posStop = 0;
    this->motor->getNowPosition(pos, this->m_axis);
    this->motor->getRecordPosition_X(posStart, 0);
    this->motor->getRecordPosition_X(posStop, 1);
    float dis = pos - posStart;
    bool blog = RUNREVERSAL;
    if(dis<0)
    {
        blog = RUNFRONT;
    }
    this->motor->runMotor(RUNMOTION, blog, abs(dis), this->m_axis);
    this->motor->getNowPosition(pos, this->m_axis);
    while(pos != posStart)
    {
        this->motor->getNowPosition(pos, this->m_axis);
        if(this->threadStop)
        {
            this->motor->stopMotor(this->m_axis);
            return;
        }
    }

    //相机开机
    this->camera->collectData();

    //然后运动到终点位置
    dis = posStop - posStart;
    this->motor->runMotor(RUNMOTION, RUNFRONT, dis, this->m_axis);
    this->motor->getNowPosition(pos, this->m_axis);
    while(pos != posStop)
    {
        this->motor->getNowPosition(pos, this->m_axis);
        if(this->threadStop)
        {
            this->motor->stopMotor(this->m_axis);
            return;
        }
    }

    //扫描完成
    this->camera->overCollectData();
    msleep(500);
    //点云转化
    this->cloud->StartThread(ToPCLCloud::ONEAXIS);
    msleep(500);

    //回到起始点
    this->motor->runMotor(RUNMOTION, RUNREVERSAL, dis, this->m_axis);
    this->motor->getNowPosition(pos, this->m_axis);
    while(pos != posStart)
    {
        this->motor->getNowPosition(pos, this->m_axis);
        if(this->threadStop)
        {
            this->motor->stopMotor(this->m_axis);
            return;
        }
    }
}

OneAxisThread::OneAxisThread()
{
    this->motor = MotorCtr::getInstance();
    this->camera = cameraCtr::getInstance();
    this->cloud = ToPCLCloud::getInstance();
    this->threadStop = false;
    this->m_axis = AXIS_X;
}

