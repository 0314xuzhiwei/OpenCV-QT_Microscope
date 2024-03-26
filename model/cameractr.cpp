#include "cameractr.h"
#include<QDebug>

cameraCtr *cameraCtr::myself = nullptr;
cameraCtr *cameraCtr::getInstance()
{
    if(cameraCtr::myself == nullptr)
    {
        cameraCtr::myself = new cameraCtr();
    }
    return cameraCtr::myself;
}
cameraCtr::cameraCtr()
{
    hGrabber = NULL;
    num = 0;
}

void cameraCtr::openCamera(WId showWidget, int Width, int Height)
{
    //TODO: Add your control notification handler code here
   if( hGrabber )
   {
       IC_ReleaseGrabber(&hGrabber);
   }

   char* ch=QString("device.xml").toLocal8Bit().data();
   hGrabber = IC_LoadDeviceStateFromFile( hGrabber, ch );
   if ( !IC_IsDevValid( hGrabber ) )
   {
       hGrabber = IC_ShowDeviceSelectionDialog(NULL);
   }
   // If we have selected a valid device, save it to the file "device.xml", so
   // the application can load it automatically when it is started the next time.
   if( IC_IsDevValid( hGrabber ) )
   {

       //ScreenToClient(rtImageWnd);

       IC_SetHWnd( hGrabber, showWidget ); //设置实时显示的窗口句柄

       IC_SaveDeviceStateToFile( hGrabber, ch );
       // Now display the device's name in the caption bar of the application.
       //SetWindowText( "My " + QString( IC_GetDeviceName( hGrabber ) ) );
       IC_StartLive( hGrabber, 1 );
       IC_GetImageDescription( hGrabber, &iWidth, &iHeight, &iBitsPerPixel, &ColorFormat );
       IC_SetDefaultWindowPosition ( hGrabber, 0 );
       IC_SetWindowPosition ( hGrabber, 0, 0, Width, Height );

   }
}

void cameraCtr::saveImage()
{
    Mat mimg;
        IC_SnapImage ( hGrabber, INFINITE );
        //IC_GetImagePtr( hGrabber );//获取图像数据指针


        unsigned char *data;
        data = IC_GetImagePtr( hGrabber );//获取图像数据指针;
        QImage img((unsigned char*)data,iWidth,iHeight,iWidth*3,QImage::Format_RGB888);
        QString qstr = QString("./picture/%1.jpg").arg(num++);
        img.save(qstr,"jpg",100);
//        mimg = QImage2Mat(img);
//        cvtColor(mimg, mimg, COLOR_BGR2GRAY);
//        threshold(mimg, mimg, 0, 255, THRESH_OTSU);  //大津法二值化
//        imshow("1",mimg);
}



void cameraCtr::getImage(Mat& image)
{
    IC_SnapImage ( hGrabber, INFINITE );
    unsigned char *data;
    data = IC_GetImagePtr( hGrabber );//获取图像数据指针;
    QImage img((unsigned char*)data,iWidth,iHeight,iWidth*3,QImage::Format_RGB888);
    image = QImage2Mat(img);
//    cvtColor(image, image, COLOR_BGR2GRAY);
//    threshold(image, image, 0, 255, THRESH_OTSU);  //大津法二值化
}

Mat cameraCtr::QImage2Mat(const QImage &image)
{
    switch(image.format())
        {
            // 8-bit, 4 channel
            case QImage::Format_ARGB32:
                break;
            case QImage::Format_ARGB32_Premultiplied:
                {
                    cv::Mat mat(image.height(), image.width(),
                                CV_8UC4,
                                (void*)image.constBits(),
                                image.bytesPerLine());
                    return mat.clone();
                }

            // 8-bit, 3 channel
            case QImage::Format_RGB32:
                {
                    cv::Mat mat(image.height(),image.width(),
                                CV_8UC4,
                                (void*)image.constBits(),
                                image.bytesPerLine());

                     // drop the all-white alpha channel
                    cv::cvtColor(mat, mat, cv::COLOR_BGRA2BGR);
                    return mat.clone();
                }
            case QImage::Format_RGB888:
                {
                    QImage swapped = image.rgbSwapped();
                    cv::Mat mat(swapped.height(), swapped.width(),
                                CV_8UC3,
                                (void*)image.constBits(),
                                image.bytesPerLine());
                    return mat.clone();
                }

            // 8-bit, 1 channel
            case QImage::Format_Indexed8:
                {
                    cv::Mat mat(image.height(),image.width(),
                                CV_8UC1,
                                (void*)image.constBits(),
                                image.bytesPerLine());
                    return mat.clone();
                }

            // wrong
            default:
                //qDebug() << "ERROR: QImage could not be converted to Mat.";
                break;
        }
        return cv::Mat();

}

void cameraCtr::Entropy(Mat img, double& value)
{
    double temp[256] = { 0.0 };

        // 计算每个像素的累积值
        for (int m = 0; m < img.rows; m++)
        {
            const uchar* t = img.ptr<uchar>(m);
            for (int n = 0; n < img.cols; n++)
            {
                int i = t[n];
                temp[i] = temp[i] + 1;
            }
        }

        // 计算每个像素的概率
        for (int i = 0; i < 256; i++)
        {
            temp[i] = temp[i] / (img.rows*img.cols);
        }

        double result = 0;
        // 计算图像信息熵
        for (int i = 0; i < 256; i++)
        {
            if (temp[i] == 0.0)
                result = result;
            else
                result = result - temp[i] * (log(temp[i]) / log(2.0));
        }
        value = result;
}

void cameraCtr::LapFun(Mat image, double& value)
{
    Mat imgLap;
    Laplacian(image, imgLap, 2,5,1,0,CV_16U);
    //Laplacian(image, imgLap, CV_16S, 3, 1, 0, BORDER_DEFAULT);

    //图像的平均灰度
    value = mean(imgLap)[0];
}
