#include "multiplectr.h"

#include <iostream>
#include <QDebug>
#include "myDefine.h"


multiplectr *multiplectr::myself = nullptr;

multiplectr *multiplectr::getInstance()
{
        if(multiplectr::myself == nullptr)
        {
            multiplectr::myself = new multiplectr();
        }
        return multiplectr::myself;
}

multiplectr::multiplectr()
{

}

//void multiplectr::setPara(float m_acc, float m_dec, float m_lspeed, float m_speed, float m_sramp, float m_units, float m_step, int m_mode)
//{
//    this->m_acc = m_acc;
//    this->m_dec = m_dec;
//    this->m_lspeed = m_lspeed;
//    this->m_speed = m_speed;
//    this->m_sramp = m_sramp;
//    this->m_units = m_units;
//    this->m_step = m_step;
//    this->m_mode = m_mode;
//}

void multiplectr::setMode(int mode)
{
    this->m_mode = mode;
}

void multiplectr::getMode(int &mode)
{
    mode = this->m_mode;
}

void multiplectr::runMotor(int mode, bool bLogic, float step, int m_nAxis)
{
    //判断当前轴状态--是否处于运动状态  1
    int status = 0;
    ZAux_Direct_GetIfIdle(g_handle, m_nAxis,&status);

    if (status == 0) //已经在运动中
    {
        return;
    }
    //设定轴类型 1-脉冲轴类型
    ZAux_Direct_SetAtype(g_handle, m_nAxis, 1);

    //设定脉冲模式及逻辑方向（脉冲+方向）
    ZAux_Direct_SetInvertStep(g_handle, m_nAxis, 0);

    //设置脉冲当量	1表示以一个脉冲为单位 ，设置为1MM的脉冲个数，这度量单位为MM
    ZAux_Direct_SetUnits(g_handle, m_nAxis, m_units);

    //设定速度，加减速
    ZAux_Direct_SetLspeed(g_handle, m_nAxis, m_lspeed);
    ZAux_Direct_SetSpeed(g_handle, m_nAxis, m_speed);
    ZAux_Direct_SetAccel(g_handle, m_nAxis, m_acc);
    ZAux_Direct_SetDecel(g_handle, m_nAxis, m_dec);

    //设定S曲线时间 设置为0表示梯形加减速
    ZAux_Direct_SetSramp(g_handle, m_nAxis, m_sramp);

    //边界处理
    if(this->stopFlage[m_nAxis] == 2 && bLogic)
    {
        return;
    }
    else if(this->stopFlage[m_nAxis] == 1 && !bLogic)
    {
        return;
    }

    if(mode == 0 )
    {//持续驱动(速度模式)
        ZAux_Direct_Single_Vmove(g_handle, m_nAxis, bLogic?1:-1);
    }
    else
    {//寸动(位置模式)
        ZAux_Direct_Single_Move(g_handle, m_nAxis, step*(bLogic?1:-1));
    }
}


