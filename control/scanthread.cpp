#include "scanthread.h"
#include <QDebug>
#include <QFile>

ScanThread::ScanThread()
{
    this->camera = cameraCtr::getInstance();
    this->motor = MotorCtr::getInstance();
    this->axisZDis = 0;
    this->scanGapDis = 0;
    this->maxScanCols = 3000;
    this->Gap = 0;
    this->m_speed = 0;
    this->p_start = new AXIS_P;
    this->p_stop = new AXIS_P;
    this->stopFlage = 0;
    memset(this->p_start, 0, sizeof (AXIS_P));
    memset(this->p_stop, 0, sizeof (AXIS_P));
    this->cloudList = new QList<PointCloudT::Ptr>;
    this->keepFile = "";
    this->cloudThread = ToPCLCloud::getInstance();
}

void ScanThread::setPara(float axisZ, float scanGapDis, float scanGap, int maxScanCols, float m_speed)
{
    this->axisZDis = axisZ;
    this->scanGapDis = scanGapDis;
    this->Gap = scanGap;
    this->m_speed = m_speed;
    if(this->maxScanCols != maxScanCols)
    {
        this->maxScanCols = maxScanCols;
    }
}

void ScanThread::setPStart(AXIS_P p)
{
    memset(this->p_start, 0, sizeof(AXIS_P));
    this->p_start->x = p.x + this->Gap;
    if(this->p_start->x > AXISXMAX)
    {
        this->p_start->x = AXISXMAX;
    }
    this->p_start->y = p.y;
    if(this->p_start->y > AXISYMAX)
    {
        this->p_start->y = AXISYMAX;
    }
    this->p_start->z = this->axisZDis;
    if(this->p_start->z > AXISZMAX)
    {
        this->p_start->z = AXISZMAX;
    }
}

void ScanThread::setPStop(AXIS_P p)
{
    memset(this->p_stop, 0, sizeof(AXIS_P));
    this->p_stop->x = p.x + this->Gap;
    if(this->p_stop->x > AXISXMAX)
    {
        this->p_stop->x = AXISXMAX;
    }
    this->p_stop->y = p.y + this->Gap;
    if(this->p_stop->y > AXISYMAX)
    {
        this->p_stop->y = AXISYMAX;
    }
    this->p_stop->z = this->axisZDis;
    if(this->p_stop->z > AXISZMAX)
    {
        this->p_stop->z = AXISZMAX;
    }
}

void ScanThread::setMode(int mode, QString file)
{
    this->threadMode = mode;
    this->stopFlage = 0;
    if(file != "")
    {
        this->keepFile = file;
    }
}

void ScanThread::resetMotor_XYZ()
{
    this->motor->returnZero_XYZ();
}

void ScanThread::Stop()
{
    this->stopFlage = 1;
}


void ScanThread::showWorkspace()
{
    float position[3] = {0};
    //先运动到坐标起始点, 终止点，起始点
    this->motor->moveLine_XY(*this->p_start, this->m_speed * 2);
    this->motor->getNowPosition(position[0], 0);  //x
    this->motor->getNowPosition(position[1], 1);  //y
    while(position[0] != this->p_start->x || position[1] != this->p_start->y)
    {
        this->motor->getNowPosition(position[0], 0);  //x
        this->motor->getNowPosition(position[1], 1);  //y
        if(this->stopFlage == 1)
        {
            return;
        }
    }
    //落Z轴
    this->motor->runMotor(RUNMOTION, RUNFRONT, this->axisZDis, AXIS_Z);
    //qDebug() << "z dis: " << this->p_start->z;
    this->motor->getNowPosition(position[2], 2);  //z
    while(position[2] != this->p_start->z)
    {
        this->motor->getNowPosition(position[2], 2);  //z
        if(this->stopFlage == 1)
        {
            return;
        }
    }
    //运动到终止点
    this->motor->moveLine_XY(*this->p_stop, this->m_speed * 2);
    this->motor->getNowPosition(position[0], 0);  //x
    this->motor->getNowPosition(position[1], 1);  //y
    while(position[0] != this->p_stop->x || position[1] != this->p_stop->y)
    {
        this->motor->getNowPosition(position[0], 0);  //x
        this->motor->getNowPosition(position[1], 1);  //y
        if(this->stopFlage == 1)
        {
            return;
        }
    }
    //运动到起始点
    this->motor->moveLine_XY(*this->p_start, this->m_speed * 2);
    this->motor->getNowPosition(position[0], 0);  //x
    this->motor->getNowPosition(position[1], 1);  //y
    while(position[0] != this->p_start->x || position[1] != this->p_start->y)
    {
        this->motor->getNowPosition(position[0], 0);  //x
        this->motor->getNowPosition(position[1], 1);  //y
        if(this->stopFlage == 1)
        {
            return;
        }
    }
    //上升Z轴
    this->motor->runMotor(RUNMOTION, RUNREVERSAL, this->axisZDis, AXIS_Z);
    this->motor->getNowPosition(position[2], 2);  //z
    while(position[2] != 0)
    {
        this->motor->getNowPosition(position[2], 2);  //z
        if(this->stopFlage == 1)
        {
            return;
        }
    }
}

void ScanThread::scanWorkspace(int mode)
{
    float position[3] = {0};
    //先移动到起始点
    this->motor->moveLine_XY(*this->p_start, this->m_speed);
    this->motor->getNowPosition(position[0], 0);  //x
    this->motor->getNowPosition(position[1], 1);  //y
    while(position[0] != this->p_start->x || position[1] != this->p_start->y)
    {
        this->motor->getNowPosition(position[0], 0);  //x
        this->motor->getNowPosition(position[1], 1);  //y
        if(this->stopFlage == 1)
        {
            return;
        }
    }
    //Z轴下落
    this->motor->runMotor(RUNMOTION, RUNFRONT, this->axisZDis, AXIS_Z);
    this->motor->getNowPosition(position[2], 2);  //z
    while(position[2] != this->axisZDis)
    {
        this->motor->getNowPosition(position[2], 2);  //z
        if(this->stopFlage == 1)
        {
            return;
        }
    }
    //然后开始按照间隔扫描
    AXIS_P point = {0,0,0};
    float work_h = abs(this->p_stop->y - this->p_start->y);
    float len = abs(this->p_stop->x - this->p_start->x);
    float len_f = len / this->scanGapDis;//用于判断是否需要+1
    int len_i = int(len_f);
    if((len_f-len_i) != 0)
    {
        len_i += 1;
    }
    if(this->stopFlage == 1)
    {
        return;
    }
    for(int i=0; i<len_i; i++)
    {
        while(this->cloudThread->getThreadState() != 0)
        {
            msleep(1);
        }
        //开启扫描程序
        if(mode)
        {
            this->camera->collectData();
            msleep(50);    //延迟一下等待相机稳定
        }
        //开始循环扫描
        point.x = this->p_start->x + this->scanGapDis * i;
        if(point.x > AXISXMAX)
        {
            point.x = AXISXMAX;
        }
        point.y = this->p_start->y + work_h;
        if(point.y > AXISYMAX)
        {
            point.y = AXISYMAX;
        }
        point.z = this->axisZDis;
        //开始移动
        this->motor->moveLine_XY(point, this->m_speed);
        //检测坐标
        this->motor->getNowPosition(position[0], 0);  //x
        this->motor->getNowPosition(position[1], 1);  //y
        while(position[0] != point.x || position[1] != point.y)
        {
            this->motor->getNowPosition(position[0], 0);  //x
            this->motor->getNowPosition(position[1], 1);  //y
            if(this->stopFlage == 1)
            {
                return;
            }
        }
        //到位后，移动到下一个起始位置
        //关闭相机扫描
        if(mode)
        {
            this->camera->overCollectData();
            //开启点云保存线程
            this->cloudThread->StartThread(ToPCLCloud::MOREAXIS);
        }
        if(this->stopFlage == 1)
        {
            return;
        }
        if(i == len_i-1)
        {
            break;
        }
        point.x = this->p_start->x + this->scanGapDis * (i+1);
        if(point.x > AXISXMAX)
        {
            point.x = AXISXMAX;
        }
        point.y = this->p_start->y;
        if(point.y > AXISYMAX)
        {
            point.y = AXISYMAX;
        }
        point.z = this->axisZDis;
        //开始移动
        this->motor->moveLine_XY(point, this->m_speed * 2);
        //检测坐标
        this->motor->getNowPosition(position[0], 0);  //x
        this->motor->getNowPosition(position[1], 1);  //y
        while(position[0] != point.x || position[1] != point.y)
        {
            this->motor->getNowPosition(position[0], 0);  //x
            this->motor->getNowPosition(position[1], 1);  //y
            if(this->stopFlage == 1)
            {
                return;
            }
        }
    }
    if(this->stopFlage == 1)
    {
        return;
    }
    //全部运行完成，区域路径扫描结束，需要回到原点
    this->motor->returnZero_XYZ();
}

void ScanThread::run()
{
    float position[3] = {0};
    this->motor->getNowPosition(position[2], 2);  //z
    if(position[2] != 0)
    {
        this->motor->runMotor(RUNMOTION, RUNREVERSAL, position[2], AXIS_Z);
        this->motor->getNowPosition(position[2], 2);  //z
        while(position[2] != 0)
        {
            this->motor->getNowPosition(position[2], 2);  //z
            if(this->stopFlage == 1)
            {
                return;
            }
        }
    }

    switch (this->threadMode) {
    case SCANAREA:
    {
        this->showWorkspace();
        break;
    }
    case SCANPATH:
    {
        //显示扫描路径，不开启扫描
        this->scanWorkspace(0);
        break;
    }
    case SCANBEGIN:
    {
        //进行扫描----清空数据
        if(!cloudList->isEmpty())
        {
            cloudList->clear();
        }
        this->scanWorkspace(1);
        qDebug() << "scanf over" ;
        while(this->cloudThread->getThreadState() != 0)
        {
            msleep(1);
        }
        qDebug() << "getCloud" ;
        this->cloudThread->getCloud(0);
        qDebug() << "getCloud over" ;
        break;
    }
    }
}

QList<PointCloudT::Ptr> *ScanThread::getCloudList() const
{
    return cloudList;
}
