#ifndef ONEAXISTHREAD_H
#define ONEAXISTHREAD_H

#include <QObject>
#include <QThread>
#include "../model/motorctr.h"
#include "../model/cameractr.h"
#include "./topclcloud.h"


class OneAxisThread : public QThread
{
public:
    static OneAxisThread* getInstance();
    bool getMotorConnect();   //查看是否连接运动控制卡
    void getMotorNowPosition(float& position, int nAxis);  //获取当前轴位置
    float getMotorSpeed();
    void setMotorSpeed(float speed);
    void motorAdd(int axis);
    void motorReduce(int axis);
    void StopMotor(int axis);
    void setPosition(float position, int model);
    void recordPosition(int axis, int model);
    void startRun(int axis);
    void axisPosZero(int axis);
    void Stop();

protected:
    void run() override;

private:
    static OneAxisThread* myself;
    OneAxisThread();
    MotorCtr* motor;    //电机控制
    cameraCtr* camera;  //相机控制
    ToPCLCloud* cloud;  //点云控制
    bool threadStop;
    int m_axis;
};

#endif // ONEAXISTHREAD_H
