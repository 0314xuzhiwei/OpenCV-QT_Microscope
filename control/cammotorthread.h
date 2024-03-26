#ifndef CAMMOTORTHREAD_H
#define CAMMOTORTHREAD_H

#include <QObject>
#include <QThread>
#include"../model/motorctr.h"

class camMotorThread : public QThread
{
public:
     //camMotorThread();#include(D:\0314\qt\trap\ProgrammingLog\Microscope_1_8\program\opencv_env.pri)
     //~ camMotorThread();
    static camMotorThread* getInstance();
    void setcamMotorPara(float m_acc2, float m_dec2, float m_lspeed2, float m_speed2, float m_sramp2,
                 float m_units2, float m_step2, int m_mode2);     //设置电机参数
    void setcamMotorMode(int mode2);
    void getcamMotorMode(int& mode2);
    void runcamMotor(int mode2, bool bLogic2, float step2, int nAxis2);       //控制电机运动
    void stopcamMotor(int nAxis2);
    void axiscamPosZero(int axis);
protected:

private:
    camMotorThread();
    MotorCtr* motor;
    static camMotorThread*myself;

};

#endif // CAMMOTORTHREAD_H
