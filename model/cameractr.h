#ifndef CAMERACTR_H
#define CAMERACTR_H

#include<iostream>
#include "myDefine.h"
#include<opencv2/opencv.hpp>
#include<QImage>
#include"./include/tisgrabber.h"
#include"windows.h"

using namespace std;
using namespace cv;

class cameraCtr
{
public:
    static cameraCtr* getInstance();

    void openCamera(WId showWidget,int Width,int Height);
    void saveImage();
    void Entropy(Mat img,double& value);
    void getImage(Mat& image);
    Mat QImage2Mat(const QImage& image);
    void LapFun(Mat image,double& value);
private:
    cameraCtr();
    static cameraCtr* myself;
    HGRABBER                    hGrabber; //相机实时抓取器
    long						iWidth;
    long						iHeight;
    int							iBitsPerPixel;
    COLORFORMAT					ColorFormat;
    int num;  //保存图片
};

#endif // CAMERACTR_H
