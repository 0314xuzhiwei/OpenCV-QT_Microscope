#include "mainwindow.h"
#include "ui_mainwindow.h"
#include<QDebug>
#include<QMessageBox>
#include"windows.h"
#include<QTimer>
#include<opencv2/opencv.hpp>

MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent)
    , ui(new Ui::MainWindow)
{
    ui->setupUi(this);

    this->setStyleSheet("#MainWindow{background-color: qlineargradient(spread:pad, x1:0, y1:0, x2:1, y2:0, stop:0 rgba(251,102,102, 200),"
                        " stop:1 rgba(20,196,188, 210));}");
    this->ui->IPEdit->setInputMask("000.000.000.000;");

    this->initialization();
    this->SerialPortInit();
    //this->image->imgInitialization();
    //初始化控制层
    this->cameraTool = new cameraThread();
    this->mainwinTool = new mainwinthread();
    this->control = ControlAll::getInstance();
    this->camMotorTool = camMotorThread::getInstance();
    image=new showImage();
    this->image->imgInitialization();
    this->image->hide();
    yoloc=new yoloC();
    this->yoloc->Init();
    this->yoloc->hide();

    update=new QTimer(this);
    frontmove=new QTimer(this);
    reversalmove=new QTimer(this);
    camfrontmove=new QTimer(this);
    camreversalmove=new QTimer(this);

    connect(update,SIGNAL(timeout()),this,SLOT(upDate()));

    connect(frontmove,SIGNAL(timeout()),this,SLOT(frontMove()));
    connect(reversalmove,SIGNAL(timeout()),this,SLOT(reversalMove()));
    connect(camfrontmove,SIGNAL(timeout()),this,SLOT(camfrontMove()));
    connect(camreversalmove,SIGNAL(timeout()),this,SLOT(camreversalMove()));
    update->start(20);

    flags = false;
}

MainWindow::~MainWindow()
{
    delete ui;
    //delete this->yoloc->ui;
}

//打开相机
void MainWindow::on_openBtn_clicked()
{
    this->cameraTool->openCamera(ui->showimage->winId(),ui->showimage->width(),ui->showimage->height());
    flags = true;
}

//保存图片
void MainWindow::on_saveBtn_clicked()
{
    // TODO: Add your control notification handler code here
    if ( !flags )
    {
         QMessageBox::warning(this, "提示","未连相机! ",QMessageBox::Ok|QMessageBox::Cancel,QMessageBox::Ok);
         qDebug()<<"未连相机!";
        return;
    }
    this->cameraTool->saveImage();

//    ui->label->setPixmap(QPixmap::fromImage(img.scaled(ui->label->size())));
}

//连接IP
void MainWindow::on_connectIP_clicked()
{

    QString ip = this->ui->IPEdit->text();

        int res = this->control->connectMotor(ip);
        if(res == 0)
        {
            QMessageBox::information(this, "提示","连接成功! ",QMessageBox::Ok|QMessageBox::No);
            this->control->setPress(0);
            return;
        }
        //连接失败
        QMessageBox::warning(this, "提示","连接失败!",QMessageBox::Ok|QMessageBox::Cancel,QMessageBox::Ok);

//          this->ui->connectIP->setEnabled(false);
//          this->ui->breakIP->setEnabled(true);
}

//断开连接
void MainWindow::on_breakIP_clicked()
{

    int res = this->control->disconnectMotor();
    if(res == 0)
    {
        //断开成功
       // this->control->setPress(0);
        QMessageBox::information(this,"提示","断开成功",QMessageBox::Ok,QMessageBox::Ok);
        this->control->setPress(1);
        return;
    }
    //断开失败
    QMessageBox::warning(this,"提示","未连接运动控制卡,请检查运动控制卡是否连接",QMessageBox::Ok|QMessageBox::Cancel,QMessageBox::Ok);
//    this->ui->connectIP->setEnabled(true);
//    this->ui->breakIP->setEnabled(false);
//    qDebug()<<"点击";

}

//UI初始化
void MainWindow::initialization()
{
    qDebug()<<"111";
    //**********公共对象初始化*********//
    this->ui->IPEdit->setText("192.168.0.11");

    //**********第一线程对象初始化*********//
    float	m_acc1 = 2000;          //加速度
    float	m_dec1 = 2000;          //减速度
    float	m_lspeed1 = 0;       //起始速度
    float	m_speed1 = 50;        //电机速度
    float	m_sramp1 = 10;        //S曲线时间
    float	m_units1 = 10;        //脉冲当量
    float	m_step1 = 10;         //寸动运行的步长
    //float   m_position1=0;
    float   m_nowposition1=0;
    m_nAxis = 0;  //选择X轴,单轴
    m_mode = 0;  //持续运动
    m_mode1=0;
    press = 1;

    //设置到模型层
//   this->control->setMotorPara1(m_acc1, m_dec1, m_lspeed1, m_speed1, m_sramp1, m_units1,
//                                m_step1, m_mode);

    //速度改下
    this->ui->speedEdit1->setText(QString("%1").arg(m_speed1));
    //脉冲当量改下
    this->ui->pulseEdit1->setText(QString("%1").arg(m_units1));
    this->ui->srampEdit1->setText(QString("%1").arg(m_sramp1));
    this->ui->ISpeedEdit1->setText(QString("%1").arg(m_lspeed1));
    this->ui->accSpeedEdit1->setText(QString("%1").arg(m_acc1));
    this->ui->decSpeedEdit1->setText(QString("%1").arg(m_dec1));
    this->ui->movedistanceEdit1->setText(QString("%1").arg(m_step1));
    this->ui->nowdistanceEdit1->setText(QString("%1").arg(m_nowposition1));

    qDebug()<<"222";

          //**********第二线程初始化******//
    float	m_acc2 = 2000;          //加速度
    float	m_dec2 = 2000;          //减速度
    float	m_lspeed2 = 0;       //起始速度
    float	m_speed2 = 50;        //电机速度
    float	m_sramp2 = 10;        //S曲线时间
    float	m_units2 = 10;        //脉冲当量
    float	m_step2 = 10;         //寸动运行的步长
    //float   m_position2=0;
    float   m_nowposition2=0;

    //设置到模型层
  /* this->control->setMotorPara1(m_acc2, m_dec2, m_lspeed2, m_speed2, m_sramp2, m_units2,
                                m_step2, m_mode);*/
    //速度改下
    this->ui->speedEdit2->setText(QString("%1").arg(m_speed2));
    //脉冲当量改下
    this->ui->pulseEdit2->setText(QString("%1").arg(m_units2));
    this->ui->srampEdit2->setText(QString("%1").arg(m_sramp2));
    this->ui->ISpeedEdit2->setText(QString("%1").arg(m_lspeed2));
    this->ui->accSpeedEdit2->setText(QString("%1").arg(m_acc2));
    this->ui->decSpeedEdit2->setText(QString("%1").arg(m_dec2));
    this->ui->movedistanceEdit2->setText(QString("%1").arg(m_step2));
    this->ui->nowdistanceEdit2->setText(QString("%1").arg(m_nowposition2));

    qDebug()<<"333";
    if(m_mode == 0&&m_mode1==0)
    {
        //持续模式
        this->ui->continueBut1->setChecked(true);
         this->ui->continueBut2->setChecked(true);
        this->ui->motionBut1->setChecked(false);
        this->ui->motionBut2->setChecked(false);
    }
    else
    {
        //持续模式
        this->ui->continueBut1->setChecked(false);
         this->ui->continueBut2->setChecked(false);
        this->ui->motionBut1->setChecked(true);
        this->ui->motionBut2->setChecked(true);
    }  
}

void MainWindow::on_motionBut1_clicked()
{
    this->m_mode = 1;
    this->control->setMotorMode(m_mode);
}

void MainWindow::on_continueBut1_clicked()
{
    this->m_mode = 0;
    this->control->setMotorMode(m_mode);
}

void MainWindow::upDate()
{
   // qDebug()<<"111";
   //    m_nAxis = 0;  //选择线路0，X轴单轴
   //    m_nAxis1=1;   //选择线路1，x轴单轴
   // press = 1;
   // m_mode= 0;  //持续运动
    float	m_acc1 ;          //加速度
    float	m_dec1 ;          //减速度
    float	m_lspeed1;       //起始速度
    float	m_speed1;        //电机速度
    float	m_sramp1 ;        //S曲线时间
    float	m_units1 ;        //脉冲当量
    float	m_step1 ;         //寸动运行的步长
    float   m_position1;
    float   m_nowposition1;

    float	m_acc2 ;          //加速度
    float	m_dec2 ;          //减速度
    float	m_lspeed2;       //起始速度
    float	m_speed2;        //电机速度
    float	m_sramp2 ;        //S曲线时间
    float	m_units2 ;        //脉冲当量
    float	m_step2 ;         //寸动运行的步长
    float   m_position2;
    float   m_nowposition2;

    //qDebug()<<"123";
    //速度改下
    m_speed1= this->ui->speedEdit1->text().toInt();//速度改下
    //脉冲当量改下
    m_units1=this->ui->pulseEdit1->text().toInt();
    m_sramp1=this->ui->srampEdit1->text().toInt();
    m_lspeed1= this->ui->ISpeedEdit1->text().toInt();
    m_acc1=this->ui->accSpeedEdit1->text().toInt();
    m_dec1=this->ui->decSpeedEdit1->text().toInt();
    m_step1=this->ui->movedistanceEdit1->text().toInt();
    m_position1=this->ui->movedistanceEdit1->text().toInt();
    m_nowposition1=this->ui->nowdistanceEdit1->text().toInt();


    //速度改下
     m_speed2= this->ui->speedEdit2->text().toInt();
    //脉冲当量改下
    m_units2=this->ui->pulseEdit2->text().toInt();
    m_sramp2=this->ui->srampEdit2->text().toInt();
    m_lspeed2= this->ui->ISpeedEdit2->text().toInt();
    m_acc2=this->ui->accSpeedEdit2->text().toInt();
    m_dec2=this->ui->decSpeedEdit2->text().toInt();
    m_step2=this->ui->movedistanceEdit1->text().toInt();
    m_position2=this->ui->movedistanceEdit2->text().toInt();
    m_nowposition2=this->ui->nowdistanceEdit2->text().toInt();

  /* this->control->setMotorPara1(m_acc2, m_dec2, m_lspeed2, m_speed2, m_sramp2, m_units2,
                                  m_step2, m_mode);*/

   bool motorconnet=this->control->getMotorConnect();
   if(motorconnet)
   {
   this->control->setMotorPara1( m_acc1, m_dec1, m_lspeed1,  m_speed1,  m_sramp1,
                                 m_units1,  m_step1, m_mode);
   this->camMotorTool->setcamMotorPara( m_acc2, m_dec2, m_lspeed2,  m_speed2,  m_sramp2,
                                     m_units2,  m_step2, m_mode);
   }
   this->control->getPress(press);

    if(press)
    {
        ui->breakIP->setEnabled(false);
        ui->connectIP->setEnabled(true);
    }
    else
    {
        ui->breakIP->setEnabled(true);
        ui->connectIP->setEnabled(false);
    }
    //*获取当前第一运动轴的位置
    float nowPosition1 = 0;
    if(motorconnet)
    {
        this->control->getMotorNowPosition(nowPosition1,this->m_nAxis);
    }

    float Position = nowPosition1 / 50;
    QString pos = QString("%1").arg(Position);
    this->ui->movedistanceEdit1->setText(pos);

    //获取当前第二运动轴的位置
    float nowPosition2 = 0;
    if(motorconnet)
    {
        this->control->getMotorNowPosition(nowPosition1,this->m_nAxis1);
    }
    float Position2 = nowPosition2 / 50;
    QString pos2= QString("%1").arg(Position2);
    this->ui->movedistanceEdit2->setText(pos);
}


//****************主线程运动控制********************//

//电机正转
void MainWindow::on_frontBut1_clicked()
{
    //获取是否连接运动控制卡
    bool motorConnect = this->control->getMotorConnect();
    if(!motorConnect)
    {
        QMessageBox::warning(this, "提示","未连接运动控制卡,请检查连接! ",QMessageBox::Ok|QMessageBox::Cancel,QMessageBox::Ok);
        return;
    }
    //获取电机运行模式
    int mode = 0;
    this->control->getMotorMode(mode);
    float step = this->ui->movedistanceEdit1->text().toFloat();
    this->control->runMotor(mode, true, step, this->m_nAxis);
}

//电机反转
void MainWindow::on_reversalBut1_clicked()
{
    //获取是否连接运动控制卡
    bool motorConnect = this->control->getMotorConnect();
    if(!motorConnect)
    {
        QMessageBox::warning(this, "提示","未连接运动控制卡,请检查连接! ",QMessageBox::Ok|QMessageBox::Cancel,QMessageBox::Ok);
        return;
    }
    //获取电机运行模式
   int mode = 0;
    this->control->getMotorMode(mode);
    float step = this->ui->movedistanceEdit1->text().toFloat();
    this->control->runMotor(mode, false, step, this->m_nAxis);
}


//设置原点
void MainWindow::on_setOrigin1_clicked()
{
    //获取是否连接运动控制卡
    bool connectmove = this->control->getMotorConnect();
    if(!connectmove)
    {
        QMessageBox::warning(this, "提示","未连接运动控制卡! ",QMessageBox::Ok|QMessageBox::Cancel,QMessageBox::Ok);
        return;
    }
    //将当前轴清零
    this->control->axisPosZero(this->m_nAxis);
    this->control->recordPosition(this->m_nAxis,0);
}
//返回原点
void MainWindow::on_rebackOrigin1_clicked()
{

    //获取是否连接运动控制卡
    bool connectmove = this->control->getMotorConnect();
    if(!connectmove)
    {
        QMessageBox::warning(this, "提示","未连接运动控制卡! ",QMessageBox::Ok|QMessageBox::Cancel,QMessageBox::Ok);
        return;
    }
     this->control->returnZero();
}

//电机停止
void MainWindow::on_stopBut1_clicked()
{
    //获取是否连接运动控制卡
    bool motorConnect = this->control->getMotorConnect();
    if(!motorConnect)
    {
        QMessageBox::warning(this, "提示","未连接运动控制卡! ",QMessageBox::Ok|QMessageBox::Cancel,QMessageBox::Ok);
        return;
    }
    this->control->stopMotor(this->m_nAxis);
    qDebug()<<"电机已经停止运动";
    //qDebug()<<"轴"<<m_nAxis;
}

//长按正转
void MainWindow::on_pressfrontBut1_pressed()
{
    //获取是否连接运动控制卡
    bool connectmove = this->control->getMotorConnect();
    if(!connectmove)
    {
        QMessageBox::warning(this, "提示","未连接运动控制卡! ",QMessageBox::Ok|QMessageBox::Cancel,QMessageBox::Ok);
        return;
    }
   frontmove->start(20);
}
void MainWindow::frontMove()
{
    //设置电机持续运动
    int mode = 0;
    float step = this->ui->movedistanceEdit1->text().toFloat();
    this->control->runMotor(mode, true, step, this->m_nAxis);
}
void MainWindow::on_pressfrontBut1_released()
{
    frontmove->stop();
    this->control->stopMotor(m_nAxis);
}
//长按反转
void MainWindow::on_pressreversalBut1_pressed()
{
    //获取是否连接运动控制卡
    bool connectmove = this->control->getMotorConnect();
    if(!connectmove)
    {
        QMessageBox::warning(this, "提示","未连接运动控制卡! ",QMessageBox::Ok|QMessageBox::Cancel,QMessageBox::Ok);
        return;
    }
   reversalmove->start(20);
}

void MainWindow::reversalMove()
{
    //设置电机持续运动
    int mode = 0;
    float step = this->ui->movedistanceEdit1->text().toFloat();
    this->control->runMotor(mode, false, step, this->m_nAxis);
}



void MainWindow::on_pressreversalBut1_released()
{
    reversalmove->stop();
    this->control->stopMotor(m_nAxis);
}

//*************************副线程运动控制**********************//

void MainWindow::camfrontMove()
{
    //设置电机持续运动
    int mode = 0;
    float step = this->ui->movedistanceEdit1->text().toFloat();
    this->camMotorTool->runcamMotor(mode, true, step, this->m_nAxis1);

}
void MainWindow::camreversalMove()
{
    //设置电机持续运动
    int mode = 0;
    float step = this->ui->movedistanceEdit1->text().toFloat();
    this->camMotorTool->runcamMotor(mode, false, step, this->m_nAxis1);
}
//变倍正转
void MainWindow::on_frontBut2_clicked()
{
    //获取是否连接运动控制卡
    bool motorStatus = this->control->getMotorConnect();
    if(!motorStatus)
    {
        QMessageBox::warning(this, "提示","未连接运动控制卡! ",QMessageBox::Ok|QMessageBox::Cancel,QMessageBox::Ok);
        return;
    }
    //获取电机运行模式
    int mode = 0;
    this->camMotorTool->getcamMotorMode(mode);
    float step = this->ui->movedistanceEdit2->text().toFloat();
    this->camMotorTool->runcamMotor(mode, true, step, this->m_nAxis1);

}
//变倍反转
void MainWindow::on_reversalBut2_clicked()
{
    //获取是否连接运动控制卡
    bool motorStatus = this->control->getMotorConnect();
    if(!motorStatus)
    {
        QMessageBox::warning(this, "提示","未连接运动控制卡! ",QMessageBox::Ok|QMessageBox::Cancel,QMessageBox::Ok);
        return;
    }
    //获取电机运行模式
    int mode = 0;
    this->camMotorTool->getcamMotorMode(mode);
    float step = this->ui->movedistanceEdit2->text().toFloat();
    this->camMotorTool->runcamMotor(mode, false, step, this->m_nAxis1);
}
//变倍停止
void MainWindow::on_stopBut2_clicked()
{
    //获取是否连接运动控制卡
    bool motorStatus = this->control->getMotorConnect();
    if(!motorStatus)
    {
        QMessageBox::warning(this, "提示","未连接运动控制卡! ",QMessageBox::Ok|QMessageBox::Cancel,QMessageBox::Ok);
        return;
    }
    this->camMotorTool->stopcamMotor(this->m_nAxis1);
    qDebug()<<"电机停止";
    qDebug()<<"轴 "<<m_nAxis1;
}
//变倍长按正转
void MainWindow::on_pressfrontBut2_pressed()
{
    //获取是否连接运动控制卡
    bool motorStatus = this->control->getMotorConnect();
    if(!motorStatus)
    {
        QMessageBox::warning(this, "提示","未连接运动控制卡! ",QMessageBox::Ok|QMessageBox::Cancel,QMessageBox::Ok);
        return;
    }
    this->camfrontmove->start(20);
}

void MainWindow::on_pressfrontBut2_released()
{
 this->camfrontmove->stop();
 this->camMotorTool->stopcamMotor(this->m_nAxis1);
}

//变倍长按反转
void MainWindow::on_pressreversalBut2_pressed()
{
    //获取是否连接运动控制卡
    bool motorStatus = this->control->getMotorConnect();
    if(!motorStatus)
    {
        QMessageBox::warning(this, "提示","未连接运动控制卡! ",QMessageBox::Ok|QMessageBox::Cancel,QMessageBox::Ok);
        return;
    }
 this->camreversalmove->start(20);
}

void MainWindow::on_pressreversalBut2_released()
{
 this->camreversalmove->stop();
 this->camMotorTool->stopcamMotor(this->m_nAxis1);
}

void MainWindow::on_motionBut2_clicked()
{
    this->m_mode1 = 1;
    this->camMotorTool->setcamMotorMode(m_mode1);
}

void MainWindow::on_continueBut2_clicked()
{
    this->m_mode1 = 0;
    this->camMotorTool->setcamMotorMode(m_mode1);
}

void MainWindow::SerialPortInit()
{
    this->serial = new QSerialPort;                       //申请内存,并设置父对象

  // 获取计算机中有效的端口号，然后将端口号的名称给端口选择控件
  /*foreach(const QSerialPortInfo &info,QSerialPortInfo::availablePorts())
  {
      serial->setPort(info);                      // 在对象中设置串口
      if(serial->open(QIODevice::ReadWrite))      // 以读写方式打开串口
      {
          ui->PortBox_2->addItem(info.portName());  // 添加计算机中的端口
          serial->close();                        // 关闭
      }
      else
      {
        qDebug()<<" 正在打开串口，请稍后 !";
      }
  }*/
  // 参数配置
  // 波特率，波特率默认选择9600 ，禁止用户点击
  serial->setBaudRate(QSerialPort::Baud9600);
  serial->setParity(QSerialPort::NoParity); // 校验，校验默认选择
  serial->setDataBits(QSerialPort::Data8);// 数据位，数据位默认选择8位
  serial->setStopBits(QSerialPort::OneStop);// 停止位，停止位默认选择1位
  serial->setFlowControl(QSerialPort::NoFlowControl);  // 控制流，默认选择无
  RefreshSerialPort(0); // 刷新串口
  connect(serial,&QSerialPort::readyRead,this,&MainWindow::DataReceived);         // 接收数据
}

void MainWindow::RefreshSerialPort(int index)
{
    // 刷新串口
    QStringList portNameList;                                        // 存储所有串口名
    if(index != 0)
    {
        serial->setPortName(ui->PortBox_2->currentText());             //设置串口号
    }
    else
    {
        ui->PortBox_2->clear();                                        //关闭串口号
        ui->PortBox_2->addItem("刷新");                                //添加刷新
        foreach(const QSerialPortInfo &info,QSerialPortInfo::availablePorts()) //添加新串口
        {
            portNameList.append(info.portName());
        }
        ui->PortBox_2->addItems(portNameList);
        ui->PortBox_2->setCurrentIndex(7);                             // 当前串口号为COM4
        serial->setPortName(ui->PortBox_2->currentText());             //设置串口号
    }
}

void MainWindow::DataReceived()
{
    //显示串口数据

    //char BUF[5] = {0};                                       // 存储转换类型后的数据
    QByteArray data = serial->readAll();
    // 读取数据
    if(!data.isEmpty())                                 // 接收到数据
    {
        QString str = ui->DataReceived_2->toPlainText();  // 返回纯文本
        str += tr(data);                                // 数据是一行一行传送的，要保存所有数据
        ui->DataReceived_2->clear();                      // 清空之前的数据
        ui->DataReceived_2->setText(str);                  // 将数据放入控件中
    }

}


void MainWindow::on_OpenSerialButton_2_clicked()
{
   //打开串口
    if(serial->isOpen())                                        // 如果串口打开了，先给他关闭
    {
        serial->clear();
        serial->close();
        // 关闭状态，按钮显示“打开串口”
        ui->OpenSerialButton_2->setText("打开串口");

        ui->OpenSerialButton_2->setStyleSheet("color: green;");

        ui->DataReceived_2->clear();

    }
    else                                                        // 如果串口关闭了，先给他打开
    {
        //当前选择的串口名字
        serial->setPortName(ui->PortBox_2->currentText());
        //用ReadWrite 的模式尝试打开串口，无法收发数据时，发出警告
        if(!serial->open(QIODevice::ReadWrite))
        {
            QMessageBox::warning(this,tr("提示"),tr("串口打开失败!"),QMessageBox::Ok);
            return;
         }
        // 打开状态，按钮显示“关闭串口”
        ui->OpenSerialButton_2->setText("关闭串口");
        // 打开状态，禁止用户操作
        ui->OpenSerialButton_2->setStyleSheet("color: red;");
    }
}

void MainWindow::on_pushButton_6_clicked()
{
    //开始变倍

    if ( !flags )
    {
         QMessageBox::warning(this, "提示","未连相机! ",QMessageBox::Ok|QMessageBox::Cancel,QMessageBox::Ok);
         qDebug()<<"未连相机!";
        return;
    }
    //获取是否连接运动控制卡
    bool motorStatus = this->mainwinTool->getMotorConnect();
    if(!motorStatus)
    {
        QMessageBox::warning(this, "提示","未连接运动控制卡! ",QMessageBox::Ok|QMessageBox::Cancel,QMessageBox::Ok);
        return;
    }
    qDebug()<<"2";
    switch (multiple)
    {
    qDebug()<<"3";
    case 0:  //1倍
    {
        float distance = 0;
        this->mainwinTool->startRun(distance);
        break;
    }
    case 1:  //1.5倍
    {
        float distance = 705;
        this->mainwinTool->startRun(distance);
        break;
    }
    case 2:
    {
        float distance = 1017.5;
        this->mainwinTool->startRun(distance);
        break;
    }
    case 3:
    {
        float distance = 1115.5;
        this->mainwinTool->startRun(distance);
        break;
    }
    }

}

void MainWindow::on_autoFocous_clicked()
{
    //自动聚焦
    if ( !flags )
    {
        QMessageBox::warning(this, "提示","未连相机!",QMessageBox::Ok|QMessageBox::Cancel,QMessageBox::Ok);
        return;
    }
    //获取是否连接运动控制卡
    bool motorStatus = this->mainwinTool->getMotorConnect();
    if(!motorStatus)
    {
        QMessageBox::warning(this, "提示","未连接运动控制卡! ",QMessageBox::Ok|QMessageBox::Cancel,QMessageBox::Ok);
        return;
    }
    this->cameraTool->startRun();

}



void MainWindow::on_aotoPicture_clicked()
{
    //连续自动采图

    // TODO: Add your control notification handler code here
    if ( !flags )
    {
         QMessageBox::warning(this, "提示","未连相机! ",QMessageBox::Ok|QMessageBox::Cancel,QMessageBox::Ok);
         qDebug()<<"未连相机!";
        return;
    }
    for(int i=0;i<10;i++)
    {this->cameraTool->saveImage();}

    QMessageBox::warning(this,"提示","已完成采图",QMessageBox::Ok|QMessageBox::Cancel,QMessageBox::Ok);

}

void MainWindow::on_advancePreview_clicked()
{
    //图片预览
    this->image->show();
}


void MainWindow::on_yolov5Btn_clicked()
{
 this->yoloc->show();
}
