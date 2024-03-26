#include "controlall.h"
#include "myDefine.h"
#include<QDebug>


ControlAll* ControlAll::myself = nullptr;

ControlAll *ControlAll::getInstance()
{
    if(ControlAll::myself == nullptr)
    {
        ControlAll::myself = new ControlAll();
    }
    return ControlAll::myself;
}
/*
 * 函数名称：void ControlAll::getMotorPara(float m_acc, float m_dec, float m_lspeed, float m_speed, float m_sramp,
                              float m_units, float m_step, int m_mode)
 * 函数参数：对应的电机参数
 * 函数作用：获取电机的参数
 * 返回值：void
 */
void ControlAll::getMotorPara(float &m_acc, float &m_dec, float &m_lspeed, float &m_speed, float &m_sramp,
                              float &m_units, float &m_step, int &m_mode)
{
    this->motor->getAllPara(m_acc, m_dec, m_lspeed, m_speed, m_sramp, m_units,
                            m_step, m_mode);
}
/*
 * 函数名称：void ControlAll::setMotorPara(float m_acc, float m_dec, float m_lspeed, float m_speed, float m_sramp,
                              float m_units, float m_step, int m_mode)
 * 函数参数：对应的电机参数
 * 函数作用：设置电机的参数
 * 返回值：void
 */
void ControlAll::setMotorPara1(float m_acc, float m_dec, float m_lspeed, float m_speed, float m_sramp,
                              float m_units, float m_step, int m_mode)
{
    this->motor->setPara(m_acc, m_dec, m_lspeed, m_speed, m_sramp,
                         m_units, m_step, m_mode);
}

/*void ControlAll::setMotorPara2(float m_acc, float m_dec, float m_lspeed, float m_speed, float m_sramp,
                              float m_units, float m_step, int m_mode)
{
    this->motor->setcamPara(m_acc, m_dec, m_lspeed, m_speed, m_sramp,
                         m_units, m_step, m_mode);
}*/

/*
 * 函数名称：int ControlAll::connectMotor(QString ip)
 * 函数参数：ip-IP地址
 * 函数作用：连接运动控制卡
 * 返回值：0-连接成功  -1-连接失败
 */
int ControlAll::connectMotor(QString ip)
{
    int res = this->motor->Connect(ip);
    return res;
}
/*
 * 函数名称：int ControlAll::disconnectMotor()
 * 函数参数：void
 * 函数作用：断开运动控制卡
 * 返回值：0-断开成功  -1-断开失败
 */
int ControlAll::disconnectMotor()
{
    int res = this->motor->disconnect();
    return res;
}
/*
 * 函数名称：void ControlAll::setMotornAxis(int nAxis)
 * 函数参数：nAxis-对应设置的轴类型
 * 函数作用：设置当前运动的轴
 * 返回值：void
 */
//void ControlAll::setMotornAxis(int nAxis)
//{
//    this->motor->setnAxis(nAxis);
//}
/*
 * 函数名称：void ControlAll::getMotorStatus(int &status)
 * 函数参数：status-获取到的轴状态
 * 函数作用：获取轴状态
 * 返回值：void
 */
void ControlAll::getMotorStatus(int &status, int nAxis)
{
    this->motor->getStatus(status, nAxis);
}
/*
 * 函数名称：bool ControlAll::getMotorConnect()
 * 函数参数：void
 * 函数作用：获取轴运行状态
 * 返回值：true-运动中  false-停止运动
 */
bool ControlAll::getMotorConnect()
{
    return this->motor->getConnect();
}
/*
 * 函数名称：void ControlAll::getMotorNowPosition(float &position)
 * 函数参数：position-获取到的轴位置
 * 函数作用：获取轴当前位置
 * 返回值：void
 */
void ControlAll::getMotorNowPosition(float &position, int nAxis)
{
    this->motor->getNowPosition(position, nAxis);
}
/*
 * 函数名称：void ControlAll::setMotorMode(int mode)
 * 函数参数：mode-设置轴的模式
 * 函数作用：设置轴的模式
 * 返回值：void
 */
void ControlAll::setMotorMode(int mode)
{
    this->motor->setMode(mode);
}
/*
 * 函数名称：void ControlAll::runMotor(int mode, bool bLogic, float step)
 * 函数参数：mode-设置轴的模式  bLogic-正反转  step-运动步长
 * 函数作用：控制电机运动
 * 返回值：void
 */
void ControlAll::runMotor(int mode, bool bLogic, float step, int nAxis)
{
    this->motor->runMotor(mode, bLogic, step, nAxis);
}
/*
 * 函数名称：void ControlAll::getMotorMode(int &mode)
 * 函数参数：mode-轴的模式
 * 函数作用：获取轴的模式
 * 返回值：void
 */
void ControlAll::getMotorMode(int &mode)
{
    this->motor->getMode(mode);
}
/*
 * 函数名称：void ControlAll::stopMotor()
 * 函数参数：void
 * 函数作用：停止电机运动
 * 返回值：void
 */
void ControlAll::stopMotor(int nAxis)
{
    this->motor->stopMotor(nAxis);
}
/*
 * 函数名称：void ControlAll::getMotorRecordPosition_X(float &position, int index)
 * 函数参数：position-位置信息  index-选择获取的目标
 * 函数作用：获取记录的x轴位置信息
 * 返回值：void
 */
void ControlAll::getMotorRecordPosition_X(float& position, int index)
{
    this->motor->getRecordPosition_X(position, index);
}
/*
 * 函数名称：void ControlAll::setMotorRecordPosition_X(float position, int index)
 * 函数参数：position-位置信息  index-选择获取的目标
 * 函数作用：设置记录的x轴位置信息
 * 返回值：void
 */
void ControlAll::setMotorRecordPosition_X(float position, int index)
{
    this->motor->setRecordPosition_X(position, index);
}
/*
 * 函数名称：void ControlAll::glueOP(int value)
 * 函数参数：value-控制值
 * 函数作用：胶阀控制
 * 返回值：void
 */
void ControlAll::glueOP(int value)
{
    this->motor->OP(GLUE, value);
}

int ControlAll::readMotorIO(int num)
{
    return this->motor->readIO(num);
}
/*
 * 函数名称：int ControlAll::motorReturnZero(int m_nAxis, int m_datumin, int m_datummode)
 * 函数参数：m_nAxis-控制轴  m_datumin-原点输入口信号  m_datummode-回零模式
 * 函数作用：胶阀控制
 * 返回值：0-回零完成  -1-未连接控制卡  1-电机处于运动状态
 */
int ControlAll::motorReturnZero(int m_nAxis, int m_datumin, int m_datummode)
{
    return this->motor->returnZero(m_nAxis, m_datumin, m_datummode);
}
/*
 * 函数名称：void ControlAll::axisPosZero(int axis)
 * 函数参数：nAxis-控制轴
 * 函数作用：对应轴坐标置零
 * 返回值：void
 */
void ControlAll::axisPosZero(int axis)
{
    this->motor->setDpos(0, axis);
}

void ControlAll::recordPosition(int axis, int model)
{
    float position = 0;
    this->motor->getNowPosition(position, axis);
    this->motor->setRecordPosition_X(position, model);
}

void ControlAll::returnZero()
{
    float posStart = 0,pos = 0;
    int m_nAxis =  0;
    this->motor->getNowPosition(pos,m_nAxis);
    this->motor->setRecordPosition_X(posStart,0);
    float dis = pos - posStart;
    bool blog = RUNREVERSAL;
    if(dis<0)
    {
        blog = RUNFRONT;
    }
    this->motor->runMotor(RUNMOTION, blog, abs(dis), m_nAxis);
}

void ControlAll::setPress(int press)
{
    this->motor->setPress(press);
}

void ControlAll::getPress(int &press)
{
    this->motor->getPress(press);
}



///*
// * 函数名称：void ControlAll::connectCamera(QString ip)
// * 函数参数：ip-IP地址
// * 函数作用：连接3D相机
// * 返回值：-1-连接成功  0-连接失败
// */
//int ControlAll::connectCamera(QString ip)
//{
//    return this->camera->Connect(ip);
//}
///*
// * 函数名称：void ControlAll::disconnectCamera()
// * 函数参数：void
// * 函数作用：断开3D相机
// * 返回值：-1-断开失败  0-断开成功
// */
//int ControlAll::disconnectCamera()
//{
//    return this->camera->disconnect();
//}

//void ControlAll::getCameraPictureType(int &type)
//{
//    this->camera->getPictureType(type);
//}

//void ControlAll::setCameraPictureType(int type)
//{
//    this->camera->setPictureType(type);
//}

//int ControlAll::cameraCollectData()
//{
//    return this->camera->collectData();
//}

//int ControlAll::cameraCollectDataOver()
//{
//    return this->camera->overCollectData();
//}

ControlAll::ControlAll()
{
    //初始化两个控制对象
    this->motor = MotorCtr::getInstance();
    //this->camera = cameraCtr::getInstance();
}
