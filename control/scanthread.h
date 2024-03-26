#ifndef SCANTHREAD_H
#define SCANTHREAD_H

#include <QObject>
#include <QThread>
#include <QMap>
#include "./model/motorctr.h"
#include "./model/cameractr.h"
#include "myDefine.h"
#include "topclcloud.h"

#include <pcl/common/common.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <vtkRenderWindow.h>
#include <pcl/filters/filter.h>

#include <pcl/io/ply_io.h>
#include <pcl/io/pcd_io.h>

typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> PointCloudT;

class ScanThread : public QThread
{
    Q_OBJECT
public:
    enum Mode {
        SCANAREA,
        SCANPATH,
        SCANBEGIN,
    };
    ScanThread();
    void setPara(float axisZ, float scanGapDis, float scanGap, int maxScanCols,float m_speed);
    void setPStart(AXIS_P p);
    void setPStop(AXIS_P p);
    void setMode(int mode, QString file="");
    void resetMotor_XYZ();
    void Stop();
    QList<PointCloudT::Ptr> *getCloudList() const;

protected:
    void showWorkspace();
    void scanWorkspace(int mode);
    void run();

private:
    cameraCtr* camera;  //相机控制
    MotorCtr* motor;    //电机控制
    AXIS_P* p_start;    //工作起始点
    AXIS_P* p_stop;     //工作终点
    int threadMode;    //线程运行模式
    int stopFlage;      //线程停止
    int maxScanCols;    //
    float axisZDis;     //Z轴下落距离
    float scanGapDis;   //扫描间距
    float Gap;      //相机与胶管间距
    float m_speed;      //扫描速度
    QList<PointCloudT::Ptr>* cloudList;     //点云数据集合
    QString keepFile;   //保存的文件名
    ToPCLCloud* cloudThread;    //点云转化线程
};

#endif // SCANTHREAD_H
