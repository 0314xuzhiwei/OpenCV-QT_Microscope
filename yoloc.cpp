#include "yoloc.h"
#include "ui_yoloc.h"
//#include<QStatusBar>

yoloC::yoloC(QWidget *parent) :
    QWidget(parent),
    ui(new Ui::yoloC)
{
    ui->setupUi(this);
    setWindowTitle(QStringLiteral("YoloV5目标检测软件"));
    this->setStyleSheet("#yoloC{background-color: qlineargradient(spread:pad, x1:0, y1:0, x2:1, y2:0, stop:0 rgba(187, 155, 204, 200),"
                        " stop:1 rgba(252, 200, 238, 210));}");


    yolo_nets=new NetConfig(); //为NetConfig开辟空间
    timer = new QTimer(this);
    timer->setInterval(33);
    connect(timer,SIGNAL(timeout()),this,SLOT(readFrame()));
    ui->startdetect->setEnabled(false);
    ui->stopdetect->setEnabled(false);
    //Init();
}

yoloC::~yoloC()
{
    delete ui;
    capture->release();
    delete capture;
    delete [] yolo_nets;
    delete yolov5;
    delete ui;
}

//模型预处理
void yoloC::Init()
{
    capture = new cv::VideoCapture();
    yolo_nets = new NetConfig[4]{
                                {0.5, 0.5, 0.5, "yolov5s"},
                                {0.6, 0.6, 0.6, "yolov5m"},
                                {0.65, 0.65, 0.65, "yolov5l"},
                                {0.75, 0.75, 0.75, "yolov5x"}
                            };
    conf = yolo_nets[0];
    yolov5 = new YOLOV5();
    yolov5->Initialization(conf);
            ui->textEditlog->append(QStringLiteral("默认模型类别：yolov5s args: %1 %2 %3")
                                    .arg(conf.nmsThreshold)
                                    .arg(conf.objThreshold)
                                    .arg(conf.confThreshold));
}

//读取模型
void yoloC::readFrame()
{
    //读取图像
    qDebug()<<125;
    cv::Mat frame;
    capture->read(frame);
    if (frame.empty()) return;

    qDebug()<<126;
    //显示每桢图像的时间
    auto start = std::chrono::steady_clock::now();//开始时刻
    yolov5->detect(frame);
    auto end = std::chrono::steady_clock::now();//结束时刻
    std::chrono::duration<double, std::milli> elapsed = end - start;//所需时间
    ui->textEditlog->append(QString("cost_time: %1 ms").arg(elapsed.count()));

    double t0 = static_cast<double>(cv::getTickCount());
    yolov5->detect(frame);
    double t1 = static_cast<double>(cv::getTickCount());
    ui->textEditlog->append(QStringLiteral("cost_time: %1 ").arg((t1 - t0) / cv::getTickFrequency()));
    qDebug()<<127;
    cv::cvtColor(frame, frame, cv::COLOR_BGR2RGB);
    QImage rawImage = QImage((uchar*)(frame.data),frame.cols,frame.rows,frame.step,QImage::Format_RGB888);
    ui->label->setPixmap(QPixmap::fromImage(rawImage));
}

//打开文件
void yoloC::on_openfile_clicked()
{
    QString filename = QFileDialog::getOpenFileName(this,QStringLiteral("打开文件"),".","*.mp4 *.avi;;*.png *.jpg *.jpeg *.bmp");
    if(!QFile::exists(filename)){
        return;
    }
    //ui->statusbar->showMessage(filename);

    QMimeDatabase db;
    QMimeType mime = db.mimeTypeForFile(filename);
    if (mime.name().startsWith("image/")) {
        cv::Mat src = cv::imread(filename.toLatin1().data());
        if(src.empty()){
            //ui->statusbar->showMessage("图像不存在！");
            return;
        }
        cv::Mat temp;
        if(src.channels()==4)
            cv::cvtColor(src,temp,cv::COLOR_BGRA2RGB);
        else if (src.channels()==3)
            cv::cvtColor(src,temp,cv::COLOR_BGR2RGB);
        else
            cv::cvtColor(src,temp,cv::COLOR_GRAY2RGB);

        auto start = std::chrono::steady_clock::now();
        yolov5->detect(temp);
        auto end = std::chrono::steady_clock::now();
        std::chrono::duration<double, std::milli> elapsed = end - start;
        ui->textEditlog->append(QString("cost_time: %1 ms").arg(elapsed.count()));
        QImage img = QImage((uchar*)(temp.data),temp.cols,temp.rows,temp.step,QImage::Format_RGB888);
        ui->label->setPixmap(QPixmap::fromImage(img));
        ui->label->resize(ui->label->pixmap()->size());
        filename.clear();
    }else if (mime.name().startsWith("video/")) {
        capture->open(filename.toLatin1().data());
        if (!capture->isOpened()){
            ui->textEditlog->append("fail to open MP4!");
            return;
        }
        IsDetect_ok +=1;
        if (IsDetect_ok ==2)
            ui->startdetect->setEnabled(true);
        ui->textEditlog->append(QString::fromUtf8("Open video: %1 succesfully!").arg(filename));

        //获取整个帧数QStringLiteral
        long totalFrame = capture->get(cv::CAP_PROP_FRAME_COUNT);
        ui->textEditlog->append(QStringLiteral("整个视频共 %1 帧").arg(totalFrame));
        ui->label->resize(QSize(capture->get(cv::CAP_PROP_FRAME_WIDTH), capture->get(cv::CAP_PROP_FRAME_HEIGHT)));

        //设置开始帧()
        long frameToStart = 0;
        capture->set(cv::CAP_PROP_POS_FRAMES, frameToStart);
        ui->textEditlog->append(QStringLiteral("从第 %1 帧开始读").arg(frameToStart));

        //获取帧率
        double rate = capture->get(cv::CAP_PROP_FPS);
        ui->textEditlog->append(QStringLiteral("帧率为: %1 ").arg(rate));
    }
}

//加载模型
void yoloC::on_loadfile_clicked()
{
    QString onnxFile = QFileDialog::getOpenFileName(this,QStringLiteral("选择模型"),".","*.onnx");
    if(!QFile::exists(onnxFile)){
        return;
    }
    //ui->statusbar->showMessage(onnxFile);
    if (!yolov5->loadModel(onnxFile.toLatin1().data())){
        ui->textEditlog->append(QStringLiteral("加载模型失败！"));
        return;
    }
    IsDetect_ok +=1;
    ui->textEditlog->append(QString::fromUtf8("Open onnxFile: %1 succesfully!").arg(onnxFile));
    if (IsDetect_ok ==2)
        ui->startdetect->setEnabled(true);
}

//开始检测
void yoloC::on_startdetect_clicked()
{
    timer->start();
    ui->startdetect->setEnabled(false);
    ui->stopdetect->setEnabled(true);
    ui->openfile->setEnabled(false);
    ui->loadfile->setEnabled(false);
    ui->comboBox->setEnabled(false);
    ui->textEditlog->append(QStringLiteral("================\n"
                                           "    开始检测\n"
                                           "================\n"));
}

//停止检测
void yoloC::on_stopdetect_clicked()
{
    ui->startdetect->setEnabled(true);
    ui->stopdetect->setEnabled(false);
    ui->openfile->setEnabled(true);
    ui->loadfile->setEnabled(true);
    ui->comboBox->setEnabled(true);
    timer->stop();
    ui->textEditlog->append(QStringLiteral("================\n"
                                           "    停止检测\n"
                                           "================\n"));
}

//选择迁移模型类型
void yoloC::on_comboBox_activated(const QString &arg1)
{
    if (arg1.contains("s")){
        conf = yolo_nets[0];
    }else if (arg1.contains("m")) {
        conf = yolo_nets[1];
    }else if (arg1.contains("l")) {
        conf = yolo_nets[2];
    }else if (arg1.contains("x")) {
        conf = yolo_nets[3];}
    yolov5->Initialization(conf);
    ui->textEditlog->append(QStringLiteral("使用模型类别：%1 args: %2 %3 %4")
                            .arg(arg1)
                            .arg(conf.nmsThreshold)
                            .arg(conf.objThreshold)
                            .arg(conf.confThreshold));
}

