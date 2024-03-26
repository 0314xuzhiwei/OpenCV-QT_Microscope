#include "camerathread.h"
#include<QDebug>
#include<QPixmap>

cameraThread::cameraThread()
{
    this->cameractr = cameraCtr::getInstance();
    this->motor = MotorCtr::getInstance();
    status = 0;
    NUM = 0;
}

void cameraThread::openCamera(WId showWidget,int Width,int Height)
{
    this->cameractr->openCamera(showWidget,Width,Height);
}

void cameraThread::saveImage()
{
    this->cameractr->saveImage();
}

void cameraThread::Entropy(Mat img, double &value)
{
    this->cameractr->Entropy(img,value);
}

void cameraThread::getImage(Mat& image)
{
    this->cameractr->getImage(image);
}

void cameraThread::LapFun(Mat image, double &value)
{
    this->cameractr->LapFun(image,value);
}

void cameraThread::textSetImage(Mat& image)
{
    Mat imgGray,imgThre;
    double value = 0;
    cvtColor(image, imgGray, COLOR_BGR2GRAY);
    threshold(imgGray, imgThre, 0, 255, THRESH_OTSU);  //大津法二值化
    this->cameractr->Entropy(imgThre,value);
    string str2 = QString("x%1.bmp").arg(NUM++).toStdString();
    imwrite(str2,image);
    stringstream meanValueStream;
    string meanValueString;
    meanValueStream << value;
    meanValueStream >> meanValueString;
    meanValueString = "value: " + meanValueString;
    putText(image, meanValueString, Point(20, 50), FONT_HERSHEY_COMPLEX, 0.8, Scalar(255, 255, 25), 2);
    string str = QString("%1.bmp").arg(NUM++).toStdString();
    imwrite(str,image);
}

void cameraThread::startRun()
{
    this->start();
}


void cameraThread::run()
{
    int num = 0;
    float nowPosition = 0;
    double val;
    bool blog = RUNFRONT;
    this->cameractr->getImage(image);
    //this->textSetImage(image);
    cvtColor(image, image, COLOR_BGR2GRAY);
    threshold(image, image, 0, 255, THRESH_OTSU);  //大津法二值化
    this->cameractr->Entropy(image,startvalue[0]);
    //this->cameractr->LapFun(image,startvalue[0]);
    this->motor->getNowPosition(nowPosition,0);
    qDebug()<<"pos:"<<nowPosition<<","<<"0:"<<startvalue[0];
    value.push_back(startvalue[0]);
    pos.push_back(nowPosition);
    Sleep(1000);
    this->motor->runMotor(RUNMOTION,RUNFRONT,50,0);
    this->motor->getStatus(status,0);
    while(status != -1)
    {
        this->motor->getStatus(status,0);
        //qDebug()<<status;
        if(status == -1)
        {
            Sleep(2000);
            this->cameractr->getImage(image);
            //this->textSetImage(image);
            cvtColor(image, image, COLOR_BGR2GRAY);
            threshold(image, image, 0, 255, THRESH_OTSU);  //大津法二值化
            this->cameractr->Entropy(image,startvalue[1]);
            //this->cameractr->LapFun(image,startvalue[1]);
            this->motor->getNowPosition(nowPosition,0);
            qDebug()<<"pos:"<<nowPosition<<","<<"1:"<<startvalue[1];
            value.push_back(startvalue[1]);
            pos.push_back(nowPosition);
            if(startvalue[1] > startvalue[0])
            {
                blog = RUNREVERSAL;
            }
            qDebug()<<"voer";
        }
    }
    qDebug()<<"status:"<<status;
    while (num<5 || status != -1)
    {
        this->motor->getStatus(status,0);
        //qDebug()<<status;
        if(status == -1)
        {
            Sleep(1000);
            this->motor->getNowPosition(nowPosition,0);
            pos.push_back(nowPosition);
            this->cameractr->getImage(image);
            //this->textSetImage(image);
            cvtColor(image, image, COLOR_BGR2GRAY);
            threshold(image, image, 0, 255, THRESH_OTSU);  //大津法二值化
            this->cameractr->Entropy(image,val);
            //this->cameractr->LapFun(image,val);
            value.push_back(val);
            qDebug()<<"pos:"<<nowPosition<<","<<"value:"<<val;
            this->motor->runMotor(RUNMOTION,blog,35,0);
            num++;
        }
        //this->motor->getStatus(status,0);
    }
    qDebug()<<"num:"<<num;
    qDebug()<<"status2:"<<status;
    float Nposition = 0;


    qDebug()<<"status3:"<<status;
    int minPosition = min_element(value.begin(),value.end()) - value.begin();
    qDebug()<<"minPosition"<<minPosition;
    float correctPosition = 0;
    correctPosition = pos[minPosition];
    qDebug()<<"correctPosition:"<<correctPosition;
    float dis = 0;
    this->motor->getStatus(status,0);
    qDebug()<<"status4:"<<status;
    while(status != -1)
    {
        this->motor->getStatus(status,0);
        if(status == -1)
        {
            this->motor->getNowPosition(Nposition,0);
            qDebug()<<"Nposition:"<<Nposition;
            dis = Nposition - correctPosition;
            qDebug()<<"dis:"<<dis;
            bool blog2 = RUNREVERSAL;
            if(dis<0)
            {
                blog2 = RUNFRONT;
            }
            qDebug()<<"in";
            this->motor->runMotor(RUNMOTION, blog2, abs(dis), 0);
            this->motor->getStatus(status,0);
            qDebug()<<"status5:"<<status;
        }
    }

    qDebug()<<"status6:"<<status;
    value.clear();
    pos.clear();
}

