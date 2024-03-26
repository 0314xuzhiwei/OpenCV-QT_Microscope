#ifndef CAMERATHREAD_H
#define CAMERATHREAD_H

#include <QObject>
#include <QThread>
#include "../model/motorctr.h"
#include "../model/cameractr.h"

class cameraThread : public QThread
{
    Q_OBJECT
public:
    cameraThread();
    void openCamera(WId showWidget,int Width,int Height);
    void saveImage();
    void Entropy(Mat img, double& value);
    void getImage(Mat& image);
    void LapFun(Mat image,double& value);
    void textSetImage(Mat& image);
    void startRun();
protected:
    void run();
private:
    cameraCtr* cameractr;  //相机模型层
    MotorCtr* motor;

    double startvalue[2];
    vector<float> pos;
    vector<double> value;
    Mat image;
    int status;
    int NUM;

};

#endif // CAMERATHREAD_H
