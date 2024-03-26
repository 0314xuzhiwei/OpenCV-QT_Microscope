#ifndef MULTIPLECTR_H
#define MULTIPLECTR_H

#include "zauxdll2.h"
#include <iostream>
#include <QString>
#include<QImage>
#include "myDefine.h"


using namespace std;


class multiplectr
{
public:
    static multiplectr* getInstance();
    //void setPara(float m_acc, float	m_dec, float m_lspeed, float m_speed, float m_sramp,
                 //float m_units, float m_step, int m_mode);     //设置电机参数
    void setMode(int mode);  //设置当前轴模式
    void getMode(int& mode);         //获取当前模式
    void runMotor(int mode, bool bLogic, float step, int m_nAxis);   //控制电机运动
private:
    multiplectr();
    static multiplectr* myself;

    float	m_acc;          //加速度
    float	m_dec;          //减速度
    float	m_lspeed;       //起始速度
    float	m_speed;        //电机速度
    float	m_sramp;        //S曲线时间
    float	m_units;        //脉冲当量
    float	m_step;         //寸动运行的步长
    int		m_mode;         //模式选择
};

#endif // MULTIPLECTR_H
