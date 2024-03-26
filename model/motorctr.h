#ifndef MOTORCTR_H
#define MOTORCTR_H


#include "zauxdll2.h"
#include <iostream>
#include <QString>
#include<QImage>
#include "myDefine.h"


using namespace std;

class MotorCtr
{
public:
    static MotorCtr* getInstance();
    void getRecordPosition_X(float& position, int index);  //获取记录位置
    void setRecordPosition_X(float position, int index);   //设置位置
    int Connect(QString ip);   //连接运动控制卡
    int disconnect();   //断开连接
    void getNowPosition(float& position, int m_nAxis);       //获取当前位置
    void getStatus(int& status, int m_nAxis);           //获取当前电机状态
    bool getConnect();          //查看当前是否连接运动控制卡
    void getMode(int& mode);         //获取当前模式
    void setMode(int mode);     //设置当前模式
    float getStep();      //获取寸动步长
    void getAutobLogic(int& bLogic);      //获取自动运行是正转还是反转
    void runMotor(int mode, bool bLogic, float step, int m_nAxis);   //控制电机运动
    void stopMotor(int m_nAxis);    //电机停止运行
    int resetMotor(int m_nAxis);   //复位电机
    int PointToPoint(float point, int m_nAxis);  //从一点运动到另一点
    void getAllPara(float& m_acc, float& m_dec, float& m_lspeed, float& m_speed, float& m_sramp,
                    float& m_units, float& m_step, int& m_mode);  //获取电机参数
    void setPara(float m_acc, float	m_dec, float m_lspeed, float m_speed, float m_sramp,
                 float m_units, float m_step, int m_mode);     //设置电机参数
    void OP(int num, int value);    //io操作
    int readIO(int num);    //读取io
    void positionBorderDeal();  //检测三轴是否会越界
    void returnZero_XYZ();   //三个轴同时会零点
    int returnZero(int m_nAxis, int m_datumin, int m_datummode);    //回零操作
    void moveLine_XY(AXIS_P p, float speed);    //插补直线运动
    void moveCircular_XY(AXIS_P mid, AXIS_P p_end, float speed);    //圆弧插补
    float getSpeed();
    void setSpeed(float speed);
    void setDpos(float pos, int m_nAxis);
    void setPress(int press);
    void getPress(int& press);

    //电动变倍控制路线
    void setcamPara(float m_acc2, float	m_dec2, float m_lspeed2, float m_speed2, float m_sramp2,
                 float m_units2, float m_step2, int m_mode2);     //设置电机参数
    void setcamMode(int mode2);  //设置当前轴模式
    void getcamMode(int& mode2);         //获取当前模式
    void runcamMotor(int mode2, bool bLogic2, float step2, int m_nAxis2);   //控制电机运动
    void stopcamMotor(int m_nAxis2);    //电机停止运行
    void setcamDpos(float pos2, int m_nAxis2);



private:
    MotorCtr();
    static MotorCtr* myself;
    ZMC_HANDLE  g_handle;	//控制器链接句柄
    bool	m_bLogic;       //电机运动方向
    float	m_acc;          //加速度
    float	m_dec;          //减速度
    float	m_lspeed;       //起始速度
    float	m_speed;        //电机速度
    float	m_sramp;        //S曲线时间
    float	m_units;        //脉冲当量
    float	m_step;         //寸动运行的步长
    int		m_mode;         //模式选择
    //int		m_nAxis;        //轴类型
    int     status;         //电机的运行状态
    int     stopFlage[3];   //3轴停止标志    0-X 1-Y 2-Z
    float   myPosition[2];    //轴的起始位置和终止位置
    float   maxPositionXYZ[3];  //3个轴的最远位置  0-X 1-Y 2-Z
    float   minPositionXYZ[3];  //3个轴的最近位置  0-X 1-Y 2-Z

    bool press;  //按钮按下判断

    float	m_acc2;          //加速度
    float	m_dec2;          //减速度
    float	m_lspeed2;       //起始速度
    float	m_speed2;        //电机速度
    float	m_sramp2;        //S曲线时间
    float	m_units2;        //脉冲当量
    float	m_step2;         //寸动运行的步长
    int		m_mode2;         //模式选择

};

#endif // MOTORCTR_H
