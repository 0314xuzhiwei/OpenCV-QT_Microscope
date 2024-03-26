#ifndef MAINWINTHREAD_H
#define MAINWINTHREAD_H

#include <QObject>
#include<QThread>
#include "../model/motorctr.h"
#include<opencv2/opencv.hpp>

class mainwinthread : public QThread
{
public:
    mainwinthread();
    void getMotorNowPosition(float& position, int nAxis);  //获取当前轴位置
    bool getMotorConnect();   //查看是否连接运动控制卡
    void startRun(float dis);
    //void getFocusValue(Mat image, float& value);
    //Mat QImage2Mat(const QImage& image);
protected:
    void run();
private:
    MotorCtr* motor;    //电机控制
    float distance;
    //Mat Mimg;
};

#endif // MAINWINTHREAD_H
