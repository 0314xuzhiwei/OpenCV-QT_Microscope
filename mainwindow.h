#ifndef MAINWINDOW_H
#define MAINWINDOW_H
#pragma execution_character_set("utf-8")

#include <QMainWindow>
#include"./control/mainwinthread.h"
#include"./control/camerathread.h"
#include"./control/controlall.h"
#include"./control/cammotorthread.h"
#include"opencv2/opencv.hpp"
#include<QTimer>
#include <QtSerialPort/QSerialPort>         // 提供访问串口的功能
#include <QtSerialPort/QSerialPortInfo>     // 提供系统中存在的串口信息
#include"showimage.h"
#include"yoloc.h"
//#include"yolov5c.h"


using namespace cv;

QT_BEGIN_NAMESPACE
namespace Ui { class MainWindow; }
QT_END_NAMESPACE

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    MainWindow(QWidget *parent = nullptr);
    ~MainWindow();

protected:
void initialization();

public slots:
void upDate();
void frontMove();
void reversalMove();
void camfrontMove();
void camreversalMove();
void  SerialPortInit();
void RefreshSerialPort(int index);
void DataReceived();

private slots:

    void on_openBtn_clicked();

    void on_saveBtn_clicked();

    void on_connectIP_clicked();

    void on_breakIP_clicked();

    void on_frontBut1_clicked();

    void on_reversalBut1_clicked();

    void on_setOrigin1_clicked();

    void on_rebackOrigin1_clicked();

    void on_stopBut1_clicked();

    void on_motionBut1_clicked();

    void on_continueBut1_clicked();

    void on_pressfrontBut1_pressed();

    void on_pressreversalBut1_pressed();

    void on_pressfrontBut1_released();

    void on_pressreversalBut1_released();

    void on_frontBut2_clicked();

    void on_reversalBut2_clicked();

    void on_stopBut2_clicked();

    void on_pressfrontBut2_pressed();

    void on_pressfrontBut2_released();

    void on_pressreversalBut2_pressed();

    void on_pressreversalBut2_released();

    void on_motionBut2_clicked();

    void on_continueBut2_clicked();

    void on_OpenSerialButton_2_clicked();

    void on_pushButton_6_clicked();

    void on_autoFocous_clicked();



    void on_aotoPicture_clicked();

    void on_advancePreview_clicked();



    void on_yolov5Btn_clicked();

private:
    Ui::MainWindow *ui;

    QTimer *update;
    QTimer*frontmove;
    QTimer*reversalmove;
    QTimer*camfrontmove;
    QTimer*camreversalmove;

    mainwinthread* mainwinTool;//自动变倍线程
    camMotorThread*camMotorTool; //第二线程控制层
    ControlAll* control;       //第一线程控制层
    cameraThread* cameraTool;  //相机线程
    showImage*     image;
    yoloC*         yoloc;
    //yolov5C*        yolov5c;



    bool flags;
    int multiple;   //放大倍率

    double startvalue[2];
    vector<float> pos;
    vector<double> value;

    //*******motor控制层******//
    int press;
    int	m_nAxis=0;        //轴类型
    int m_nAxis1=1;
    int m_mode;     //轴模式
    int m_mode1;
    //ui->showimage->width()
    //ui->showimage->width()


    //******串口设置**********//
    // 串口变量
    QSerialPort     *serial;                            // 定义全局的串口对象（第三步）
    QTimer     *iotimer;                                // 设置定时器对象
    // 参数配置
    QStringList     baudList;                           //波特率
    QStringList     parityList;                         //校验位
    QStringList     dataBitsList;                       //数据位
    QStringList     stopBitsList;                       //停止位
    QStringList     flowControlList;                    //控制流

};
#endif // MAINWINDOW_H
