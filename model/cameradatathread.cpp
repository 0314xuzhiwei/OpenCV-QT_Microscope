#include "cameradatathread.h"
#include "SR7Link.h"
#include <QMessageBox>
#include <QDebug>
#include <QApplication>
#include <QDateTime>
#include <QElapsedTimer>

CameraDataThread::CameraDataThread()
{
    this->initValue();
    this->cameraOnLine = false;
}

CameraDataThread::~CameraDataThread()
{
    if(cameraOnLine)
    {
        if(bReceiveData)
        {
            this->exitReceiveThread();
        }
        this->deleteDataMemory();
    }
}

void CameraDataThread::initValue()
{
    bReceiveData = false;
    //bCamBonline = false;
    mDeviceId = 0;  //只有一个相机所以直接写0就好
    mBatchWidth = 0;
    mProfileData = NULL;
    mIntensityData = NULL;
    mEncoderData = NULL;
    //mDispH = 25000;
    mDelayTime = 10;
    m_BatchPoint_CurNo = 0;
    mXinterVal = 0;
    mYinterVal = 0.1;
}

void CameraDataThread::setCameraOnLine(bool value)
{
    this->cameraOnLine = value;
}

void CameraDataThread::setXinterVal(double x)
{
    this->mXinterVal = x;
}

void CameraDataThread::setYinterVal(double y)
{
    this->mYinterVal = y;
}

void CameraDataThread::setDeviceID(int id)
{
    this->mDeviceId = id;
}

void CameraDataThread::setBatchWidth(int width)
{
    this->mBatchWidth = width;
}

void CameraDataThread::setDelayTime(int delay)
{
    this->mDelayTime = delay;
}

void CameraDataThread::setReceiveFlage(bool value)
{
    this->bReceiveData = value;
}

void CameraDataThread::getXinterVal(double& x)
{
    x = this->mXinterVal;
}

void CameraDataThread::getYinterVal(double &y)
{
    y = this->mYinterVal;
}

void CameraDataThread::getBatchWidth(int &width)
{
    width = this->mBatchWidth;
}

int *CameraDataThread::getProfileData()
{
    //获取轮廓数据
    if(!this->cameraOnLine)
    {
        return nullptr;
    }
    return this->mProfileData;
}

unsigned char *CameraDataThread::getIntensityData()
{
    //获取亮度数据
    if(!this->cameraOnLine)
    {
        return nullptr;
    }
    return this->mIntensityData;
}

unsigned int *CameraDataThread::getEncoderData()
{
    //获取编码数据
    if(!this->cameraOnLine)
    {
        return nullptr;
    }
    return this->mEncoderData;
}

long long CameraDataThread::getBatchCols()
{
    if(!this->cameraOnLine)
    {
        return 0;
    }
    return this->m_BatchPoint_CurNo;
}

int CameraDataThread::StartMeasure(int iTimeout)
{
    int reT = SR7IF_StartMeasure(this->mDeviceId, iTimeout);
    return reT;
}

void CameraDataThread::DataMemoryInit(int Height, int mProW)
{
    if(!this->cameraOnLine)
    {
        return;
    }
    //初始化数据
    try
    {
        //int MAXWIDTH = mProW;
        int MAXECODER = 1;
        int mDataC = Height * mProW;
        this->mProfileData = new int[mDataC];
        this->mIntensityData = new unsigned char[mDataC];
        this->mEncoderData = new unsigned int[Height * MAXECODER];
        for(int i = 0; i < mDataC; i ++)
        {
            mProfileData[i] = -10000 * 100000;
        }
        memset(mIntensityData, 0, sizeof(unsigned char) * mDataC);
        memset(mEncoderData, 0, sizeof(unsigned int) * Height * MAXECODER);
    }
    catch (...)
    {
        qDebug() << "InfiniteLoop: 数据内存申请失败!";
        /*QMessageBox::warning(nullptr, "InfiniteLoop",
                             "数据内存申请失败! ",
                             QMessageBox::Ok|QMessageBox::No,
                             QMessageBox::Ok);*/
        exit(0);
    }
}

void CameraDataThread::deleteDataMemory()
{
    if(this->cameraOnLine)
    {
        if(this->mProfileData)
        {
            delete this->mProfileData;
            this->mProfileData = NULL;
        }
        if(this->mIntensityData)
        {
            delete this->mIntensityData;
            this->mIntensityData = NULL;
        }
        if(this->mEncoderData)
        {
            delete this->mEncoderData;
            this->mEncoderData = NULL;
        }
    }
}

void CameraDataThread::startReceiveThread(int mBatchW, double mXpitch, int iTimeout)
{
    if(cameraOnLine)
    {
        this->mBatchWidth = mBatchW;
        this->mXinterVal = mXpitch;
        this->bReceiveData = true;
        //开始批处理
        int reT = this->StartMeasure(iTimeout);
        if(reT < 0)
        {
            qDebug()<< reT;
            return;
        }
        this->start();
    }
}

void CameraDataThread::exitReceiveThread()
{
    if(cameraOnLine)
    {
        this->bReceiveData = false;
        if(this->isRunning())
        {
            this->quit();
            while(this->isRunning())  //等待线程退出完成
            {
                qApp->processEvents();     //界面刷新
            }
        }
    }
}

void CameraDataThread::run()
{
    if(!this->cameraOnLine)
    {
        return;
    }
    /******数据初始化*****/
    m_BatchPoint_CurNo = 0; //每次开始当前行数都要重新计数
    if(mBatchWidth != 0)
    {
        //this->DataMemoryInit(this->m_BatchPoint_CurNo);
        deleteDataMemory();
    }
    //数据缓存中间变量
    int* TmpBatchPoint = NULL;
    unsigned char* TmpGrayData = NULL;
    unsigned int* FrameLoss = NULL;
    long long *FrameId  = NULL;
    unsigned int* Encoder = NULL;
    int mTmpH = 1000;   //行数应设置为小于显示的高度mDispH
    try
    {
        TmpBatchPoint = new int[mTmpH * mBatchWidth];           //当前批次高度数据缓存
        TmpGrayData = new unsigned char[mTmpH * mBatchWidth];   //当前批次灰度数据缓存
        FrameLoss =  new unsigned int [mTmpH * 1];      //批处理过快掉帧数量数据缓存
        FrameId = new long long[mTmpH];                         //帧编号数据缓存
        Encoder = new unsigned int [mTmpH * 1];         //编码器数据缓存
    }
    catch (...)
    {
        //停止批处理
        SR7IF_StopMeasure(mDeviceId);
        qDebug() << "Data Thread: 数据内存申请失败! ";
        /*QMessageBox::information(nullptr, "Data Thread",
                                         "数据内存申请失败! ",
                                         QMessageBox::Ok|QMessageBox::No);*/
        return;
    }
    //long long BatchPoint_CurNo = 0;     //当前批处理编号
    long long OverFlowStartId = 0;      //溢出起始帧号
    int FrameLossID = 0;                //丢帧数
    int EncoderID = 0;                  //编码器值

    //循环接收
    bool bError = false;
    do
    {
        /* 接收数据---当前批次高度数据、灰度数据、编码器数据、帧编号、掉帧数量数据 */
        int m_curBatchPoint = SR7IF_GetBatchRollData(mDeviceId,
                                                     NULL,
                                                     TmpBatchPoint,
                                                     TmpGrayData,
                                                     Encoder,
                                                     FrameId,
                                                     FrameLoss,
                                                     500);
        if(m_curBatchPoint < 0)
        {
            if(m_curBatchPoint == SR7IF_ERROR_MODE)
            {
                qDebug() << "当前为非循环模式";
                bReceiveData = false;
                bError = true;
                qDebug() << "CAMERA: 当前为非循环模式";
                /*QMessageBox::information(nullptr, "CAMERA",
                                                 "当前为非循环模式",
                                                 QMessageBox::Ok|QMessageBox::No);*/
                break;
            }
            else if(m_curBatchPoint == SR7IF_NORMAL_STOP)
            {
                bReceiveData = false;
                bError = true;
                qDebug() << "CAMERA: 外部IO或其他因素导致批处理正常停止";
                /*QMessageBox::information(nullptr, "CAMERA",
                                                 "外部IO或其他因素导致批处理正常停止",
                                                 QMessageBox::Ok|QMessageBox::No);*/
                break;
            }
            else
            {
                qDebug() << "CAMERA: CAMERA ERR";
                /*QMessageBox::information(nullptr, "CAMERA",
                                                 "CAMERA ERR",
                                                 QMessageBox::Ok|QMessageBox::No);*/
                continue;
            }
        }
        if(m_curBatchPoint == 0)
        {
            continue;
        }
        //数据正常采集
        //上一次最后一行对应的帧等信息
        int TmpID = m_curBatchPoint - 1;
        OverFlowStartId = FrameId[TmpID];
        FrameLossID = FrameLoss[TmpID];
        EncoderID = Encoder[TmpID];

        //数据拷贝显示
        int TmpN = m_curBatchPoint * mBatchWidth;  //此次获取的数据大小
        int dataNum_last = m_BatchPoint_CurNo;    //获取原本数据的行数大小
        m_BatchPoint_CurNo += m_curBatchPoint;    //获取的数据总行数
        long long dataNum = m_BatchPoint_CurNo * mBatchWidth * 2;  //防止数据越界---*2是因为有一个值默认是3200  mBatchWidth-是1600
        //先是申请一段可以容纳前后数据的空间大小
        if(this->mProfileData == nullptr && this->mIntensityData == nullptr && this->mEncoderData==nullptr)
        {
            this->DataMemoryInit(this->m_BatchPoint_CurNo);
        }
        else
        {
            //先保存上次的数据
            int* data_p = this->mProfileData;
            unsigned char* data_i = this->mIntensityData;
            unsigned int* data_e = this->mEncoderData;
            this->mProfileData = nullptr;
            this->mIntensityData = nullptr;
            this->mEncoderData = nullptr;
            this->DataMemoryInit(this->m_BatchPoint_CurNo);
            //后是先将之前的数据进行拷贝
            if(mProfileData)  /* 高度 */
            {
                memcpy(mProfileData, data_p, sizeof (int) * dataNum_last * mBatchWidth);
                //将原先的数据内存清除
                delete data_p;
            }
            if(mIntensityData)  /* 灰度 */
            {
                memcpy(mIntensityData, data_i, sizeof (unsigned char) * dataNum_last * mBatchWidth);
                //将原先的数据清除
                delete data_i;
            }
            if(mEncoderData) /* 编码器 */
            {
                memcpy(mEncoderData, data_e, sizeof (unsigned int) * dataNum_last);
                delete data_e;
            }
        }

        //后将新的数据添加到末尾
        /* 高度 */
        memcpy(&mProfileData[dataNum_last * mBatchWidth], TmpBatchPoint, sizeof (int) * TmpN);
        /* 灰度 */
        memcpy(&mIntensityData[dataNum_last * mBatchWidth], TmpGrayData, sizeof (unsigned char) * TmpN);
        /* 编码器 */
        memcpy(&mEncoderData[dataNum_last], Encoder, sizeof (unsigned int) * m_curBatchPoint);

        //显示
        emit SignalDataShow(mBatchWidth, m_BatchPoint_CurNo);
        //根据需要加个延迟显示
        msleep(mDelayTime);
//        QElapsedTimer timer;
//        timer.start();
//        while(timer.elapsed() < mDelayTime)
//        {
//            qApp->processEvents();
//        }

    }while(bReceiveData);

    if(bError)  //因错误导致循环退出
    {
        //停止批处理
        SR7IF_StopMeasure(mDeviceId);
    }

    /* 内存释放 */
    if(TmpBatchPoint)
    {
        delete[] TmpBatchPoint;
        TmpBatchPoint = nullptr;
    }
    if(TmpGrayData)
    {
        delete[] TmpGrayData;
        TmpGrayData = nullptr;
    }
    if(Encoder)
    {
        delete[] Encoder;
        Encoder = nullptr;
    }
    if(FrameLoss)
    {
        delete[] FrameLoss;
        FrameLoss = nullptr;
    }
    if(FrameId)
    {
        delete[] FrameId;
        FrameId = nullptr;
    }
}
