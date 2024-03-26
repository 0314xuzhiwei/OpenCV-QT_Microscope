#ifndef YOLOC_H
#define YOLOC_H

#include <QFileDialog>
#include <QFile>
#include <opencv2/opencv.hpp>
#include <opencv2/dnn.hpp>
#include <QWidget>
#include <QTimer>
#include <QImage>
#include <QPixmap>
#include <QDateTime>
#include <QMutex>
#include <QMutexLocker>
#include <QMimeDatabase>
#include <iostream>
#include <yolov5.h>
#include <chrono>


//#pragma comment(lib,"D:/0314/opencv411/opencv411/build/include/x64/vc16/lib/opencv_world411.lib")
//#pragma comment(lib,"D:/0314/opencv411/opencv411/build/include/x64/vc16/lib/opencv_img_hash411d.lib")
//opencvdnn环境配置
//#ifdef QT_NO_DEBUG
//#pragma comment(lib,"D:/0314/opencv452/install/x64/vc15/lib/opencv_core452.lib")
//#pragma comment(lib,"D:/0314/opencv452/install/x64/vc15/lib/opencv_imgcodecs452.lib")
//#pragma comment(lib,"D:/0314/opencv452/install/x64/vc15/lib/opencv_imgproc452.lib")
//#pragma comment(lib,"D:/0314/opencv452/install/x64/vc15/lib/opencv_imgcodecs452.lib")
//#pragma comment(lib,"D:/0314/opencv452/install/x64/vc15/lib/opencv_video452.lib")
//#pragma comment(lib,"D:/0314/opencv452/install/x64/vc15/lib/opencv_videoio452.lib")
//#pragma comment(lib,"D:/0314/opencv452/install/x64/vc15/lib/opencv_objdetect452.lib")
//#pragma comment(lib,"D:/0314/opencv452/install/x64/vc15/lib/opencv_shape452.lib")
//#pragma comment(lib,"D:/0314/opencv452/install/x64/vc15/lib/opencv_dnn452.lib")
//#pragma comment(lib,"D:/0314/opencv452/install/x64/vc15/lib/opencv_dnn_objdetect452.lib")
//#endif
//#include <QWidget>

namespace Ui {
class yoloC;
}

class yoloC : public QWidget
{
    Q_OBJECT

public:
    explicit yoloC(QWidget *parent = nullptr);
    void Init();
    ~yoloC();
private slots:
    void readFrame(); //自定义信号处理函数

    void on_openfile_clicked();

    void on_loadfile_clicked();

    void on_startdetect_clicked();

    void on_stopdetect_clicked();

    void on_comboBox_activated(const QString &arg1);

private:

    QTimer *timer;
    cv::VideoCapture *capture;

    YOLOV5 *yolov5;
    NetConfig conf;
    NetConfig *yolo_nets;
    std::vector<cv::Rect> bboxes;
    int IsDetect_ok = 0;

private:
    Ui::yoloC *ui;
};

#endif // YOLOC_H
