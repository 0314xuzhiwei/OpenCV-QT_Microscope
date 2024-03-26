#ifndef GLUETHREAD_H
#define GLUETHREAD_H

#include <QObject>
#include <QThread>
#include "./model/motorctr.h"
#include "myDefine.h"

class GlueThread : public QThread
{
    Q_OBJECT
public:
    GlueThread();
    void Stop();
    void setSpeed(float speed);
    void getSpeed(float& speed);
    void motorAdd(int axis);
    void motorReduce(int axis);
    void StopMotor(int axis);
    void getMotorConnect(bool& status);
    void getPosition(float& position, int axis);
    int showWorkspace();
    void setOpenGlueTime(int time);
    void setCloseGlueDis(float dis);
    void resetMotor_XYZ();

protected slots:
    void setStartP(AXIS_P p);
    void setStopP(AXIS_P p);
    void setPlist(QList<GLUEPOINT*>* plist);

protected:
    void run();

private:
    MotorCtr* motor;
    AXIS_P* p_start; //工作起始点
    AXIS_P* p_stop;  //工作终点
    QList<GLUEPOINT*>* pList;    //需要点胶的所有线段
    int stopFlage;
    int openTime;   //胶阀开启时间
    int runMode;    //运行模式
    float m_speed;  //滴胶速度
    float m_units;  //脉冲当量
    float closeDis; //胶阀关闭距离
};

#endif // GLUETHREAD_H
