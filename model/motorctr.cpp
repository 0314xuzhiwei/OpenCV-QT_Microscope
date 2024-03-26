#include "motorctr.h"
#include <iostream>
#include <QDebug>
#include "myDefine.h"

using namespace std;

MotorCtr *MotorCtr::myself = nullptr;

MotorCtr *MotorCtr::getInstance()
{
    if(MotorCtr::myself == nullptr)
    {
        MotorCtr::myself = new MotorCtr();
    }
    return MotorCtr::myself;
}

MotorCtr::MotorCtr()
{
    this->myPosition[0] = -10000.0;
    this->myPosition[1] = -10000.0;
    this->status = -1;
    g_handle = nullptr;
    memset(this->stopFlage, 0, 3 * sizeof (int));

    //边界
    this->maxPositionXYZ[0] = AXISXMAX;
    this->maxPositionXYZ[1] = AXISYMAX;
    this->maxPositionXYZ[2] = AXISZMAX;

    memset(this->minPositionXYZ, 0, sizeof (float) * 3);
}
/*
 * 函数名称：int MotorCtr::connect(QString ip)
 * 函数参数：ip-IP地址
 * 函数作用：连接运动控制卡
 * 返回值：0-连接成功  -1-连接失败
 */
int MotorCtr::Connect(QString ip)
{
    if(this->g_handle != nullptr)
    {
        ZAux_Close(g_handle);
        g_handle = nullptr;
    }
    //验证ip地址
    if(ip.length() <= 3)
    {
        return -1;
    }
    //开始建立连接
    int iresult = ZAux_OpenEth((char*)(ip.toStdString().c_str()), &g_handle);
    if(ERR_SUCCESS != iresult)
    {
        g_handle = nullptr;
        return -1;
    }
    //连接成功
    return 0;
}
/*
 * 函数名称：int MotorCtr::disconnect()
 * 函数参数：void
 * 函数作用：断开运动控制卡
 * 返回值：0-断开成功  -1-断开失败，未连接运动控制卡
 */
int MotorCtr::disconnect()
{
    //断开连接
    if(nullptr != g_handle)
    {
        ZAux_Close(g_handle);
        return 0;
    }
    return -1;
}
/*
 * 函数名称：void MotorCtr::getRecordPosition(float *position)
 * 函数参数：position-当前记录下的位置
 * 函数作用：获取当前记录下的位置
 * 返回值：void
 */

void MotorCtr::getRecordPosition_X(float& position, int index)
{
    position = this->myPosition[index];
}
/*
 * 函数名称：void MotorCtr::setRecordPosition(float *position)
 * 函数参数：position-当前记录下的位置
 * 函数作用：设置当前记录下的位置
 * 返回值：void
 */
void MotorCtr::setRecordPosition_X(float position, int index)
{
    this->myPosition[index] = position;
}

void MotorCtr::getNowPosition(float &position, int m_nAxis)
{
    if(g_handle)
    {
        ZAux_Direct_GetDpos( g_handle,m_nAxis,&position);
    }
}

void MotorCtr::getStatus(int &status, int m_nAxis)
{
    if(g_handle)
    {
        ZAux_Direct_GetIfIdle(g_handle, m_nAxis,&this->status);
    }
    status = this->status;
}

bool MotorCtr::getConnect()
{
    return this->g_handle==nullptr?false:true;
}

void MotorCtr::setMode(int mode)
{
    this->m_mode = mode;
}

void MotorCtr::getMode(int &mode)
{
    mode = this->m_mode;
}

float MotorCtr::getStep()
{
    return this->m_step;
}

void MotorCtr::getAutobLogic(int &bLogic)
{
    if(this->myPosition[0] == this->myPosition[1])
    {
        //起始点等于终止点
        bLogic = -1;
    }
    else if(this->myPosition[0] > this->myPosition[1])
    {
        //起始点大于终止点---正转
        bLogic = 0;
    }
    else
    {
        //起始点小于终止点---反转
        bLogic = 1;
    }
}

void MotorCtr::runMotor(int mode, bool bLogic, float step, int m_nAxis)
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

void MotorCtr::stopMotor(int m_nAxis)
{
    ZAux_Direct_Single_Cancel(g_handle,m_nAxis,2);
}

int MotorCtr::resetMotor(int m_nAxis)
{
    if(this->myPosition[0] == -10000)
    {
        return -1;
    }
    float pos = 0;
    this->getNowPosition(pos, m_nAxis);
    if(this->myPosition[0] == pos)
    {
        return 0;
    }
    this->PointToPoint(this->myPosition[0], m_nAxis);
    return 0;
}

int MotorCtr::PointToPoint(float point, int m_nAxis)
{
    float pos = 0;
    bool b_logic = false;
    this->getNowPosition(pos, m_nAxis);
    if(pos == point)
    {
        return 0;
    }
    else if(pos < point)
    {
        //正转
        b_logic = true;
    }
    else
    {
        //反转
        b_logic = false;
    }
    //qDebug()<< "选择模式: " << b_logic;
    //以持续模式运行
    this->runMotor(0, b_logic, 0, m_nAxis);
    if(b_logic == true)
    {
        //正转
        this->getNowPosition(pos, m_nAxis);
        while (pos<point) {
            this->getNowPosition(pos, m_nAxis);
            //持续给运动信号，防止卡死
            int status = 0;
            this->getStatus(status, m_nAxis);
            if(status == -1)     //如果当前不转的，重新给信号
            {
                if(this->stopFlage[m_nAxis])
                {
                    break;
                }
                this->runMotor(0, b_logic, 0, m_nAxis);
            }
        }
    }
    else
    {
        //反转
        this->getNowPosition(pos, m_nAxis);
        while (pos>point) {
            this->getNowPosition(pos, m_nAxis);
            //持续给运动信号，防止卡死
            int status = 0;
            this->getStatus(status, m_nAxis);
            if(status == -1)    //如果当前不转的，重新给信号
            {
                if(this->stopFlage[m_nAxis] == 1)
                {
                    break;
                }
                this->runMotor(0, b_logic, 0, m_nAxis);
            }
        }
    }
    this->stopMotor(m_nAxis);
    //qDebug()<< "stopMotor";
    return 0;
}

void MotorCtr::getAllPara(float &m_acc, float &m_dec, float &m_lspeed, float &m_speed, float &m_sramp,
                          float &m_units, float &m_step, int &m_mode)
{
    m_acc = this->m_acc;
    m_dec = this->m_dec;
    m_lspeed = this->m_lspeed;
    m_speed = this->m_speed;
    m_sramp = this->m_sramp;
    m_units = this->m_units;
    m_step = this->m_step;
    m_mode = this->m_mode;
    //m_nAxis = this->m_nAxis;
}

void MotorCtr::setPara(float m_acc, float m_dec, float m_lspeed, float m_speed, float m_sramp,
                       float m_units, float m_step, int m_mode)
{
    this->m_acc = m_acc;
    this->m_dec = m_dec;
    this->m_lspeed = m_lspeed;
    this->m_speed = m_speed;
    this->m_sramp = m_sramp;
    this->m_units = m_units;
    this->m_step = m_step;
    this->m_mode = m_mode;
    //this->m_nAxis = m_nAxis;
}

void MotorCtr::OP(int num, int value)
{
    ZAux_Direct_SetOp(g_handle, num, value);
}

int MotorCtr::readIO(int num)
{
    uint32 res = 0;
    ZAux_Direct_GetIn(g_handle, num, &res);
    return int(res);
}

void MotorCtr::positionBorderDeal()
{
    //用于检测三条轴是否会超过边界，如果超过就停止运动
    //检测X轴 Y轴 Z轴
    int nAxis = 0;
    for(nAxis = 0; nAxis<3; nAxis++)
    {
        float position = 0;
        this->getNowPosition(position, nAxis);
        if(position < this->minPositionXYZ[nAxis])
        {
            //停止轴运动
            if(this->stopFlage[nAxis] == 0)
            {
                this->stopMotor(nAxis);
                //对应停止标志变化
                this->stopFlage[nAxis] = 1;
            }
            if(position < this->minPositionXYZ[nAxis])
            {
                //要重新回原点
                this->returnZero(nAxis, 8+nAxis, AXISREVERSE);
            }
        }
        else if(position > this->maxPositionXYZ[nAxis])
        {
            //停止轴运动
            if(this->stopFlage[nAxis] == 0)
            {
                this->stopMotor(nAxis);
                //对应停止标志变化
                this->stopFlage[nAxis] = 2;
            }
        }
        else
        {
            //对应停止标志变化
            this->stopFlage[nAxis] = 0;
        }
    }
}

void MotorCtr::returnZero_XYZ()
{
    this->returnZero(AXIS_X, ZERO_X, AXISREVERSE);
    this->returnZero(AXIS_Y, ZERO_Y, AXISREVERSE);
    this->returnZero(AXIS_Z, ZERO_Z, AXISREVERSE);
}

int MotorCtr::returnZero(int m_nAxis, int m_datumin, int m_datummode)
{
    if(!this->g_handle)
    {
        return -1;
    }
    //开始做回零运动
    //参数设定
    int status;
    this->getStatus(status, m_nAxis);
    if(status == 0)
    {
        //说明轴在运动
        //this->stopMotor(m_nAxis);
        return 1;
    }
    //设定轴类型 7-   脉冲轴类型 +	编码器Z信号		不用EZ回零也可以设置为1

    ZAux_Direct_SetAtype(g_handle, m_nAxis, 7);

    //设定脉冲模式及逻辑方向（脉冲+方向）
    ZAux_Direct_SetInvertStep(g_handle, m_nAxis, 0);

    //设置脉冲当量	1表示以一个脉冲为单位 ，设置为1MM的脉冲个数，这度量单位为MM
    ZAux_Direct_SetUnits(g_handle, m_nAxis, m_units);

    //设定速度，加减速
    ZAux_Direct_SetLspeed(g_handle, m_nAxis, m_lspeed);
    ZAux_Direct_SetSpeed(g_handle, m_nAxis, m_speed);
    ZAux_Direct_SetAccel(g_handle, m_nAxis, m_acc);
    ZAux_Direct_SetDecel(g_handle, m_nAxis, m_dec);
    float m_creep = 10;
    ZAux_Direct_SetCreep(g_handle, m_nAxis, m_creep);

    //设定对应轴的原点输入口信号
    ZAux_Direct_SetDatumIn(g_handle, m_nAxis, m_datumin);

    //回零运动
    ZAux_Direct_Single_Datum(g_handle, m_nAxis, m_datummode);
    return 0;
}

void MotorCtr::moveLine_XY(AXIS_P p, float speed)
{
    int axislist[2] = {0,1};					//运动BASE轴列表

    float poslist[2] = {p.x,p.y};	//运动列表
    //基本参数设置
    for(int i=0; i<2; i++)
    {
        ZAux_Direct_SetAtype(g_handle,i,1);				//轴类型  脉冲轴
        ZAux_Direct_SetUnits(g_handle,i,10);				//脉冲当量 1 脉冲为单位
        ZAux_Direct_SetLspeed(g_handle, i, m_lspeed);
    }
    //选择参与运动的轴，第一个轴为主轴，插补参数全用主轴参数

    ZAux_Direct_SetSpeed(g_handle,axislist[0],speed);				//速度	UNITS / S
    ZAux_Direct_SetAccel(g_handle,axislist[0],m_acc);				//加速度
    ZAux_Direct_SetDecel(g_handle,axislist[0],m_dec);				//减速度

    //XY直线插补
    ZAux_Direct_MoveAbs(g_handle,2,axislist,poslist);
}

void MotorCtr::moveCircular_XY(AXIS_P mid, AXIS_P p_end, float speed)
{
    int axislist[2] = {0,1};					//运动BASE轴列表

    float poslist[2] = {p_end.x,p_end.y};	//运动列表
    float midlist[2] = {mid.x, mid.y};	//圆弧中间点
    //选择参与运动的轴，第一个轴为主轴，插补参数全用主轴参数

    ZAux_Direct_SetSpeed(g_handle,axislist[0],speed);				//速度	UNITS / S
    ZAux_Direct_SetAccel(g_handle,axislist[0],m_acc);				//加速度
    ZAux_Direct_SetDecel(g_handle,axislist[0],m_dec);				//减速度

    //XY 3点圆弧
    ZAux_Direct_MoveCirc2Abs(g_handle,2,axislist,midlist[0],midlist[1],poslist[0],poslist[1]);
}

float MotorCtr::getSpeed()
{
    return this->m_speed;
}

void MotorCtr::setSpeed(float speed)
{
    this->m_speed = speed;
}

void MotorCtr::setDpos(float pos, int m_nAxis)
{
    ZAux_Direct_SetDpos(g_handle,m_nAxis,pos);        //设置零点
}

void MotorCtr::setPress(int press)
{
    this->press = press;
}

void MotorCtr::getPress(int &press)
{
    press = this->press;
}

void MotorCtr::setcamPara(float m_acc2, float m_dec2, float m_lspeed2, float m_speed2, float m_sramp2, float m_units2, float m_step2, int m_mode2)
{
    this->m_acc2 = m_acc2;
    this->m_dec2 = m_dec2;
    this->m_lspeed2 = m_lspeed2;
    this->m_speed2 = m_speed2;
    this->m_sramp2 = m_sramp2;
    this->m_units2 = m_units2;
    this->m_step2 = m_step2;
    this->m_mode2 = m_mode2;
}

void MotorCtr::setcamMode(int mode2)
{
    this->m_mode2 = mode2;
}

void MotorCtr::getcamMode(int &mode2)
{
    mode2 = this->m_mode2;
}

void MotorCtr::runcamMotor(int mode2, bool bLogic2, float step2, int m_nAxis2)
{
    //判断当前轴状态--是否处于运动状态  1
    int status = 0;
    ZAux_Direct_GetIfIdle(g_handle, m_nAxis2,&status);

    if (status == 0) //已经在运动中
    {
        return;
    }
    //设定轴类型 1-脉冲轴类型
    ZAux_Direct_SetAtype(g_handle, m_nAxis2, 1);

    //设定脉冲模式及逻辑方向（脉冲+方向）
    ZAux_Direct_SetInvertStep(g_handle, m_nAxis2, 0);

    //设置脉冲当量	1表示以一个脉冲为单位 ，设置为1MM的脉冲个数，这度量单位为MM
    ZAux_Direct_SetUnits(g_handle, m_nAxis2, m_units2);

    //设定速度，加减速
    ZAux_Direct_SetLspeed(g_handle, m_nAxis2, m_lspeed2);
    ZAux_Direct_SetSpeed(g_handle, m_nAxis2, m_speed2);
    ZAux_Direct_SetAccel(g_handle, m_nAxis2, m_acc2);
    ZAux_Direct_SetDecel(g_handle, m_nAxis2, m_dec2);

    //设定S曲线时间 设置为0表示梯形加减速
    ZAux_Direct_SetSramp(g_handle, m_nAxis2, m_sramp2);

    //边界处理
    if(this->stopFlage[m_nAxis2] == 2 && bLogic2)
    {
        return;
    }
    else if(this->stopFlage[m_nAxis2] == 1 && !bLogic2)
    {
        return;
    }

    if(mode2 == 0 )
    {//持续驱动(速度模式)
        ZAux_Direct_Single_Vmove(g_handle, m_nAxis2, bLogic2?1:-1);
    }
    else
    {//寸动(位置模式)
        ZAux_Direct_Single_Move(g_handle, m_nAxis2, step2*(bLogic2?1:-1));
    }
}

void MotorCtr::stopcamMotor(int m_nAxis2)
{
    ZAux_Direct_Single_Cancel(g_handle,m_nAxis2,2);
}

void MotorCtr::setcamDpos(float pos2, int m_nAxis2)
{
    ZAux_Direct_SetDpos(g_handle,m_nAxis2,pos2);
}



