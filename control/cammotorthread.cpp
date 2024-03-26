#include "cammotorthread.h"

//私有实例初始化
camMotorThread*camMotorThread:: myself = nullptr;

camMotorThread::camMotorThread()
{
    this->motor = MotorCtr::getInstance();
}

camMotorThread *camMotorThread::getInstance()
{
    if(camMotorThread::myself == nullptr)
    {
        camMotorThread::myself = new camMotorThread();
    }
    return camMotorThread::myself;
}

void camMotorThread::setcamMotorPara(float m_acc2, float m_dec2, float m_lspeed2, float m_speed2, float m_sramp2, float m_units2, float m_step2, int m_mode2)
{
    this->motor->setcamPara(m_acc2, m_dec2, m_lspeed2, m_speed2, m_sramp2,
                          m_units2, m_step2, m_mode2);
}

void camMotorThread::setcamMotorMode(int mode2)
{
    this->motor->setcamMode(mode2);
}

void camMotorThread::getcamMotorMode(int &mode2)
{
    this->motor->getcamMode(mode2);
}

void camMotorThread::runcamMotor(int mode2, bool bLogic2, float step2, int nAxis2)
{
    this->motor->runcamMotor(mode2, bLogic2, step2, nAxis2);
}

void camMotorThread::stopcamMotor(int nAxis2)
{
    this->motor->stopcamMotor(nAxis2);
}

void camMotorThread::axiscamPosZero(int axis2)
{
     this->motor->setcamDpos(0, axis2);
}

