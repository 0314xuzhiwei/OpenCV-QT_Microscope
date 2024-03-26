#include "gluethread.h"
#include "myDefine.h"
#include <QDebug>
#include "./tool/shape.h"
#include <math.h>

GlueThread::GlueThread()
{
    this->motor = MotorCtr::getInstance();
    this->stopFlage = 0;
    this->m_units = 10;  //默认脉冲当量是10
    this->m_speed = 500;    //默认出胶速度为100
    this->pList = new QList<GLUEPOINT*>;
    this->p_stop = new AXIS_P;
    memset(this->p_stop, 0, sizeof (AXIS_P));
    this->p_start = new AXIS_P;
    memset(this->p_start, 0, sizeof (AXIS_P));
    this->runMode = 0;
}

void GlueThread::Stop()
{
    this->stopFlage = 1;
}

void GlueThread::setSpeed(float speed)
{
    this->m_speed = speed;
}

void GlueThread::getSpeed(float &speed)
{
    speed = this->m_speed;
}

void GlueThread::motorAdd(int axis)
{
    //获取控制
    float	m_acc;          //加速度
    float	m_dec;          //减速度
    float	m_lspeed;       //起始速度
    float	m_speed;        //电机速度
    float	m_sramp;        //S曲线时间
    float	m_units;        //脉冲当量
    float	m_step;         //寸动运行的步长
    int     m_mode;
    this->motor->getAllPara(m_acc, m_dec, m_lspeed, m_speed, m_sramp, m_units, m_step, m_mode);
    //设置速度、脉冲当量、持续模式
    this->motor->setPara(m_acc, m_dec, m_lspeed, this->m_speed, m_sramp, this->m_units, m_step, m_mode);
    //电机运行
    this->motor->runMotor(RUNCONTINUE, RUNFRONT, 0, axis);
    //重新设置对应的数据
    this->motor->setPara(m_acc, m_dec, m_lspeed, m_speed, m_sramp, m_units, m_step, m_mode);
}

void GlueThread::motorReduce(int axis)
{
    //获取控制
    float	m_acc;          //加速度
    float	m_dec;          //减速度
    float	m_lspeed;       //起始速度
    float	m_speed;        //电机速度
    float	m_sramp;        //S曲线时间
    float	m_units;        //脉冲当量
    float	m_step;         //寸动运行的步长
    int     m_mode;
    this->motor->getAllPara(m_acc, m_dec, m_lspeed, m_speed, m_sramp, m_units, m_step, m_mode);
    //设置速度、脉冲当量、持续模式
    this->motor->setPara(m_acc, m_dec, m_lspeed, this->m_speed, m_sramp, this->m_units, m_step, m_mode);
    //电机运行
    this->motor->runMotor(RUNCONTINUE, RUNREVERSAL, 0, axis);
    //重新设置对应的数据
    this->motor->setPara(m_acc, m_dec, m_lspeed, m_speed, m_sramp, m_units, m_step, m_mode);
}

void GlueThread::StopMotor(int axis)
{
    //获取控制
    this->motor->stopMotor(axis);
}

void GlueThread::getMotorConnect(bool &status)
{
    status = this->motor->getConnect();
}

void GlueThread::getPosition(float &position, int axis)
{
    this->motor->getNowPosition(position, axis);
}

int GlueThread::showWorkspace()
{
    if(this->p_start->z == 0)
    {
        return -1;
    }
    if(this->p_stop->z == 0)
    {
        return -2;
    }
    this->runMode = 1;
    this->start();
    return 0;
}

void GlueThread::setOpenGlueTime(int time)
{
    this->openTime = time;
}

void GlueThread::setCloseGlueDis(float dis)
{
    this->closeDis = dis;
}

void GlueThread::resetMotor_XYZ()
{
    this->motor->returnZero_XYZ();
}

void GlueThread::setStartP(AXIS_P p)
{
    memset(this->p_start, 0, sizeof (AXIS_P));
    memcpy(this->p_start, &p, sizeof (AXIS_P));
}

void GlueThread::setStopP(AXIS_P p)
{
    memset(this->p_stop, 0, sizeof (AXIS_P));
    memcpy(this->p_stop, &p, sizeof (AXIS_P));
}

void GlueThread::setPlist(QList<GLUEPOINT*>* plist)
{
    this->pList->clear();
    int len = plist->size();
    for(int i=0; i<len; i++)
    {
        this->pList->push_back(plist->at(i));
    }
    //启动线程
    this->stopFlage = 0;
    this->runMode = 0;
    this->start();
}

void GlueThread::run()
{
    float position[3] = {0};
    //观察Z轴是否复位
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
    switch (this->runMode) {
    case 0:
    {
        //运行点胶线程
        qDebug() << "begin to glue!";
        //先运动到坐标起始点, 终止点，起始点
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
        //开始进行直线插补
        int len = this->pList->size();
        for(int i=0; i<len; i++)
        {
            //msleep(WAITETIME);
            switch (this->pList->at(i)->type) {
            case Shape::Line:
            {
                if(this->stopFlage == 1)
                {
                    return;
                }
                //先移动到起始点
                this->motor->moveLine_XY(this->pList->at(i)->p0[0], this->m_speed);
                //msleep(WAITETIME);
                this->motor->getNowPosition(position[0], 0);  //x
                this->motor->getNowPosition(position[1], 1);  //y
                while(position[0] != this->pList->at(i)->p0[0].x || position[1] != this->pList->at(i)->p0[0].y)
                {
                    this->motor->getNowPosition(position[0], 0);  //x
                    this->motor->getNowPosition(position[1], 1);  //y
                    if(this->stopFlage == 1)
                    {
                        return;
                    }
                }
                //先落Z轴
                this->motor->runMotor(RUNMOTION, RUNFRONT, this->pList->at(i)->p0[0].z, AXIS_Z);
                //msleep(WAITETIME);
                this->motor->getNowPosition(position[2], 2);  //z
                while(position[2] != this->pList->at(i)->p0[0].z)
                {
                    this->motor->getNowPosition(position[2], 2);  //z
                    if(this->stopFlage == 1)
                    {
                        return;
                    }
                }
                //然后出胶
                this->motor->OP(GLUE, 1);
                msleep(this->openTime);
                //然后移动到终止点
                if(this->stopFlage == 1)
                {
                    this->motor->OP(GLUE, 0);
                    return;
                }
                this->motor->moveLine_XY(this->pList->at(i)->p0[1], this->m_speed);
                //msleep(WAITETIME);
                this->motor->getNowPosition(position[0], 0);  //x
                this->motor->getNowPosition(position[1], 1);  //y
                while(position[0] != this->pList->at(i)->p0[1].x || position[1] != this->pList->at(i)->p0[1].y)
                {
                    this->motor->getNowPosition(position[0], 0);  //x
                    this->motor->getNowPosition(position[1], 1);  //y
                    if(this->stopFlage == 1)
                    {
                        this->motor->OP(GLUE, 0);
                        return;
                    }
                    float dis = sqrtl((this->pList->at(i)->p0[1].x - position[0]) * (this->pList->at(i)->p0[1].x - position[0])
                            + (this->pList->at(i)->p0[1].y - position[1]) * (this->pList->at(i)->p0[1].y - position[1]));
                    if(dis <= this->closeDis)
                    {
                        //关闭出胶
                        this->motor->OP(GLUE, 0);
                    }
                    qDebug() << "dis: " << dis;
                }
                this->motor->OP(GLUE, 0);
                //向后走一半的距离
                AXIS_P t_t = {0};
                t_t.x = (this->pList->at(i)->p0[1].x + this->pList->at(i)->p0[0].x) / 2;
                t_t.y = (this->pList->at(i)->p0[1].y + this->pList->at(i)->p0[0].y) / 2;
                QString str_t = QString::number(t_t.x, 'f', 1); //为了保留一位小数
                t_t.x = str_t.toFloat();
                str_t.clear();
                str_t = QString::number(t_t.y, 'f', 1);  //为了保留一位小数
                t_t.y = str_t.toFloat();
                this->motor->moveLine_XY(t_t, this->m_speed);
                //Z轴同时上移
                if(this->stopFlage == 1)
                {
                    return;
                }
                //获取控制
                float	m_acc;          //加速度
                float	m_dec;          //减速度
                float	m_lspeed;       //起始速度
                float	m_speed;        //电机速度
                float	m_sramp;        //S曲线时间
                float	m_units;        //脉冲当量
                float	m_step;         //寸动运行的步长
                int     m_mode;
                this->motor->getAllPara(m_acc, m_dec, m_lspeed, m_speed, m_sramp, m_units, m_step, m_mode);
                //设置速度、脉冲当量、持续模式
                this->motor->setPara(m_acc, m_dec, m_lspeed, this->m_speed/40, m_sramp, this->m_units, m_step, m_mode);
                //电机运行
                this->motor->runMotor(RUNMOTION, RUNREVERSAL, this->pList->at(i)->p0[0].z, AXIS_Z);
                //重新设置对应的数据
                this->motor->setPara(m_acc, m_dec, m_lspeed, m_speed, m_sramp, m_units, m_step, m_mode);
                this->motor->getNowPosition(position[0], AXIS_X);  //x
                this->motor->getNowPosition(position[1], AXIS_Y);  //y
                while(position[0] != t_t.x || position[1] != t_t.y)
                {
                    this->motor->getNowPosition(position[0], AXIS_X);  //x
                    this->motor->getNowPosition(position[1], AXIS_Y);  //y
                    if(this->stopFlage == 1)
                    {
                        return;
                    }
                }
                //msleep(WAITETIME);
                //z轴加速 ---可以减少出现打胶不好情况
                this->motor->stopMotor(AXIS_Z);
                msleep(1000);
                this->motor->getNowPosition(position[2], AXIS_Z);  //z
                this->motor->runMotor(RUNMOTION, RUNREVERSAL, position[2], AXIS_Z);
                this->motor->getNowPosition(position[2], AXIS_Z);  //z
                while(position[2] != 0)
                {
                    this->motor->getNowPosition(position[2], AXIS_Z);  //z
                    if(this->stopFlage == 1)
                    {
                        return;
                    }
                }
                break;
            }
            case Shape::Rect:
            {
                //先移动到第一点
                if(this->stopFlage == 1)
                {
                    return;
                }
                this->motor->moveLine_XY(this->pList->at(i)->p0[0], this->m_speed);
                //msleep(WAITETIME);
                this->motor->getNowPosition(position[0], 0);  //x
                this->motor->getNowPosition(position[1], 1);  //y
                while(position[0] != this->pList->at(i)->p0[0].x || position[1] != this->pList->at(i)->p0[0].y)
                {
                    this->motor->getNowPosition(position[0], 0);  //x
                    this->motor->getNowPosition(position[1], 1);  //y
                    if(this->stopFlage == 1)
                    {
                        return;
                    }
                }
                //先落Z轴
                this->motor->runMotor(RUNMOTION, RUNFRONT, this->pList->at(i)->p0[0].z, AXIS_Z);
                //msleep(WAITETIME);
                this->motor->getNowPosition(position[2], 2);  //z
                while(position[2] != this->pList->at(i)->p0[0].z)
                {
                    this->motor->getNowPosition(position[2], 2);  //z
                    if(this->stopFlage == 1)
                    {
                        return;
                    }
                }
                //然后出胶
                this->motor->OP(GLUE, 1);
                msleep(this->openTime);
                //然后移动到第二个点
                if(this->stopFlage == 1)
                {
                    this->motor->OP(GLUE, 0);
                    return;
                }
                this->motor->moveLine_XY(this->pList->at(i)->p0[1], this->m_speed);
                //msleep(WAITETIME);
                this->motor->getNowPosition(position[0], 0);  //x
                this->motor->getNowPosition(position[1], 1);  //y
                while(position[0] != this->pList->at(i)->p0[1].x || position[1] != this->pList->at(i)->p0[1].y)
                {
                    this->motor->getNowPosition(position[0], 0);  //x
                    this->motor->getNowPosition(position[1], 1);  //y
                    if(this->stopFlage == 1)
                    {
                        this->motor->OP(GLUE, 0);
                        return;
                    }
                }
                //然后移动到第三个点
                this->motor->moveLine_XY(this->pList->at(i)->p0[2], this->m_speed);
                //msleep(WAITETIME);
                this->motor->getNowPosition(position[0], 0);  //x
                this->motor->getNowPosition(position[1], 1);  //y
                while(position[0] != this->pList->at(i)->p0[2].x || position[1] != this->pList->at(i)->p0[2].y)
                {
                    this->motor->getNowPosition(position[0], 0);  //x
                    this->motor->getNowPosition(position[1], 1);  //y
                    if(this->stopFlage == 1)
                    {
                        this->motor->OP(GLUE, 0);
                        return;
                    }
                }
                //然后移动到第四个点
                this->motor->moveLine_XY(this->pList->at(i)->p0[3], this->m_speed);
                //msleep(WAITETIME);
                this->motor->getNowPosition(position[0], 0);  //x
                this->motor->getNowPosition(position[1], 1);  //y
                while(position[0] != this->pList->at(i)->p0[3].x || position[1] != this->pList->at(i)->p0[3].y)
                {
                    this->motor->getNowPosition(position[0], 0);  //x
                    this->motor->getNowPosition(position[1], 1);  //y
                    if(this->stopFlage == 1)
                    {
                        this->motor->OP(GLUE, 0);
                        return;
                    }
                }

                //msleep(2000);
                //在回到第一个点
                this->motor->moveLine_XY(this->pList->at(i)->p0[0], this->m_speed);
                //msleep(WAITETIME);
                this->motor->getNowPosition(position[0], 0);  //x
                this->motor->getNowPosition(position[1], 1);  //y
                while(position[0] != this->pList->at(i)->p0[0].x || position[1] != this->pList->at(i)->p0[0].y)
                {
                    this->motor->getNowPosition(position[0], 0);  //x
                    this->motor->getNowPosition(position[1], 1);  //y
                    if(this->stopFlage == 1)
                    {
                        this->motor->OP(GLUE, 0);
                        return;
                    }
                    float dis = sqrtl((this->pList->at(i)->p0[0].x - position[0]) * (this->pList->at(i)->p0[0].x - position[0])
                            + (this->pList->at(i)->p0[0].y - position[1]) * (this->pList->at(i)->p0[0].y - position[1]));
                    if(dis <= this->closeDis)
                    {
                        //关闭出胶
                        this->motor->OP(GLUE, 0);
                    }
                    qDebug() << "dis: " << dis;
                }
                //关闭出胶
                this->motor->OP(GLUE, 0);
                //向后走一半的距离
                AXIS_P t_t = {0};
                t_t.x = (this->pList->at(i)->p0[3].x + this->pList->at(i)->p0[0].x) / 2;
                t_t.y = (this->pList->at(i)->p0[3].y + this->pList->at(i)->p0[0].y) / 2;
                QString str_t = QString::number(t_t.x, 'f', 1); //为了保留一位小数
                t_t.x = str_t.toFloat();
                str_t.clear();
                str_t = QString::number(t_t.y, 'f', 1);  //为了保留一位小数
                t_t.y = str_t.toFloat();
                this->motor->moveLine_XY(t_t, this->m_speed);
                //Z轴同时上移
                //获取控制
                float	m_acc;          //加速度
                float	m_dec;          //减速度
                float	m_lspeed;       //起始速度
                float	m_speed;        //电机速度
                float	m_sramp;        //S曲线时间
                float	m_units;        //脉冲当量
                float	m_step;         //寸动运行的步长
                int     m_mode;
                this->motor->getAllPara(m_acc, m_dec, m_lspeed, m_speed, m_sramp, m_units, m_step, m_mode);
                //设置速度、脉冲当量、持续模式
                this->motor->setPara(m_acc, m_dec, m_lspeed, this->m_speed/40, m_sramp, this->m_units, m_step, m_mode);
                //电机运行
                this->motor->runMotor(RUNMOTION, RUNREVERSAL, this->pList->at(i)->p0[0].z, AXIS_Z);
                //重新设置对应的数据
                this->motor->setPara(m_acc, m_dec, m_lspeed, m_speed, m_sramp, m_units, m_step, m_mode);
                this->motor->getNowPosition(position[0], 0);  //x
                this->motor->getNowPosition(position[1], 1);  //y
                while(position[0] != t_t.x || position[1] != t_t.y)
                {
                    this->motor->getNowPosition(position[0], 0);  //x
                    this->motor->getNowPosition(position[1], 1);  //y
                    if(this->stopFlage == 1)
                    {
                        return;
                    }
                }
                //z轴加速---可以减少出现打胶不好情况
                this->motor->stopMotor(AXIS_Z);
                msleep(1000);
                this->motor->getNowPosition(position[2], 2);  //z
                this->motor->runMotor(RUNMOTION, RUNREVERSAL, position[2], AXIS_Z);
                //msleep(WAITETIME);
                this->motor->getNowPosition(position[2], 2);  //z
                while(position[2] != 0)
                {
                    this->motor->getNowPosition(position[2], 2);  //z
                    if(this->stopFlage == 1)
                    {
                        return;
                    }
                }
                break;
            }
            }
        }
        //全部点胶结束后复位
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
        break;
    }
    case 1:
    {
        //先运动到坐标起始点, 终止点，起始点
        this->motor->moveLine_XY(*this->p_start, this->m_speed);
        this->motor->getNowPosition(position[0], 0);  //x
        this->motor->getNowPosition(position[1], 1);  //y
        while(position[0] != this->p_start->x || position[1] != this->p_start->y)
        {
            this->motor->getNowPosition(position[0], 0);  //x
            this->motor->getNowPosition(position[1], 1);  //y
        }

        //落Z轴
        this->motor->runMotor(RUNMOTION, RUNFRONT, this->p_start->z, AXIS_Z);
        //msleep(1000);
        this->motor->getNowPosition(position[2], 2);  //z
        while(position[2] != this->p_start->z)
        {
            this->motor->getNowPosition(position[2], 2);  //z
        }
        //运动到终止点
        this->motor->moveLine_XY(*this->p_stop, this->m_speed);
        //msleep(WAITETIME);
        this->motor->getNowPosition(position[0], 0);  //x
        this->motor->getNowPosition(position[1], 1);  //y
        while(position[0] != this->p_stop->x || position[1] != this->p_stop->y)
        {
            this->motor->getNowPosition(position[0], 0);  //x
            this->motor->getNowPosition(position[1], 1);  //y
        }
        //运动到起始点
        this->motor->moveLine_XY(*this->p_start, this->m_speed);
        this->motor->getNowPosition(position[0], 0);  //x
        this->motor->getNowPosition(position[1], 1);  //y
        while(position[0] != this->p_start->x || position[1] != this->p_start->y)
        {
            this->motor->getNowPosition(position[0], 0);  //x
            this->motor->getNowPosition(position[1], 1);  //y
        }
        //msleep(1000);
        //上升Z轴
        this->motor->runMotor(RUNMOTION, RUNREVERSAL, this->p_start->z, AXIS_Z);
        break;
    }
    default:
        break;
    }
}
