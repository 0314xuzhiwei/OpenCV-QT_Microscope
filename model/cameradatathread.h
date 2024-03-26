#ifndef CAMERADATATHREAD_H
#define CAMERADATATHREAD_H

#include <QObject>
#include <QThread>

class CameraDataThread : public QThread
{
    Q_OBJECT
public:
    CameraDataThread();
    ~CameraDataThread();
    void initValue();
    void setCameraOnLine(bool value);
    void setXinterVal(double x);
    void setYinterVal(double y);
    void setDeviceID(int id);
    void setBatchWidth(int width);
    void setDelayTime(int delay);
    void setReceiveFlage(bool value);
    void getXinterVal(double& x);
    void getYinterVal(double& y);
    void getBatchWidth(int& width);
    int* getProfileData();
    unsigned char* getIntensityData();
    unsigned int* getEncoderData();
    long long getBatchCols();
    ///
    /// \brief DataMemoryInit          数据内存初始化
    /// \param mProW                   轮廓宽度
    ///
    void DataMemoryInit(int Height, int mProW = 3200);
    ///
    /// \brief deleteDataMemory         数据内存释放
    ///
    void deleteDataMemory();
    ///
    /// \brief startReceiveThread   启动接收数据线程
    ///
    void startReceiveThread(int mBatchW, double mXpitch, int iTimeout);
    ///
    /// \brief exitReceiveThread    退出接收数据线程
    ///
    void exitReceiveThread();

protected:
    void run();
    int StartMeasure(int iTimeout);

signals:
    void SignalDataShow(int _width,
                        int _height);       //数据显示信号

private:
    double mXinterVal;              //X间距
    double mYinterVal;              //Y间距
    bool bReceiveData;              //接收数据标志
    int mDeviceId;                  //设备ID号
    int mBatchWidth;                //轮廓宽度
    bool cameraOnLine;              //相机是否在线
    //bool bCamBonline;               //相机B在线标志
    //int mDispH;                     //当前显示范围
    int* mProfileData;              //批处理数据缓存
    unsigned char* mIntensityData;  //灰度数据缓存
    unsigned int* mEncoderData;     //编码器数据缓存
    int mDelayTime;                 //延迟时间
    long long m_BatchPoint_CurNo;   //当前总行数
};

#endif // CAMERADATATHREAD_H
