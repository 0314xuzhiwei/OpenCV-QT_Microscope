#ifndef CONTROLALL_H
#define CONTROLALL_H

//#include "../model/cameractr.h"
#include "../model/motorctr.h"



class ControlAll
{
public:
    static ControlAll* getInstance();
    ~ControlAll();
    //************************运动控制卡相关函数***************************************
    void getMotorPara(float& m_acc, float& m_dec, float& m_lspeed, float& m_speed, float& m_sramp,
                    float& m_units, float& m_step, int& m_mode);  //获取电机参数
    void setMotorPara1(float m_acc, float	m_dec, float m_lspeed, float m_speed, float m_sramp,
                 float m_units, float m_step, int m_mode);     //设置电机参数*/
    /*void setMotorPara2(float m_acc, float	m_dec, float m_lspeed, float m_speed, float m_sramp,
                 float m_units, float m_step, int m_mode);*/     //设置电机参数*/

    int connectMotor(QString ip);  //连接运动控制卡
    int disconnectMotor();      //断开运动控制卡
   // void setMotornAxis(int nAxis);  //设置运动轴
    void getMotorStatus(int& status, int nAxis);   //获取轴状态
    bool getMotorConnect();   //查看是否连接运动控制卡
    void getMotorNowPosition(float& position, int nAxis);  //获取当前轴位置
    void setMotorMode(int mode);        //设置轴的模式
    void runMotor(int mode, bool bLogic, float step, int nAxis);       //控制电机运动
    void getMotorMode(int& mode);    //获取电机运行模式
    void stopMotor(int nAxis);       //停止电机运行
    void getMotorRecordPosition_X(float& position, int index);  //获取记录的x坐标点
    void setMotorRecordPosition_X(float position, int index); //设置记录的x坐标点
    void glueOP(int value);   //控制胶阀
    int readMotorIO(int num);   //读取对应io数据
    int motorReturnZero(int m_nAxis, int m_datumin, int m_datummode);
    void axisPosZero(int axis);
    void recordPosition(int axis, int model);
    void returnZero();  //自定义回零点
    void setPress(int press);
    void getPress(int& press);

    //*********************************************************************************
//    //**************************************3D相机相关函数**********************************
//    int connectCamera(QString ip);  //连接3D相机
//    int disconnectCamera();     //断开连接3D相机
//    void getCameraPictureType(int& type);
//    void setCameraPictureType(int type);
//    int cameraCollectData();
//    int cameraCollectDataOver();

//变倍控制层
public:
    //void cam

private:
    ControlAll();   //构造私有化
    static ControlAll* myself;
    //cameraCtr* camera;
    MotorCtr* motor;

};

#endif // CONTROLALL_H
