#ifndef PCLDEALTHREAD_H
#define PCLDEALTHREAD_H

#include <QObject>
#include <QThread>
#include "../model/pcltool.h"

class PCLDealThread : public QThread
{
    Q_OBJECT
public:
    PCLDealThread();

    PointCloudT::Ptr getBasicCloud() const;
    void setBasicCloud(const PointCloudT::Ptr &value);

    PointCloudT::Ptr getStandCloud() const;
    void setStandCloud(const PointCloudT::Ptr &value);

    PointCloudT::Ptr getCheckCloud() const;
    void setCheckCloud(const PointCloudT::Ptr &value);

    void showProjectCloud(QString name) const;

    void showOutlier() const;

    void showDiff() const;

    int cloudReduce();

    int cloudOutlierRemoval();

    int projectionCloud();

    int cloudOutlier();

    int cloudDiff();

    int cloudCheck();

protected:
    void run();

private:
    PointCloudT::Ptr basicCloud;            //点云扫描平台
    PointCloudT::Ptr standCloud;            //标准点云
    PointCloudT::Ptr checkCloud;            //检测点云
    PointCloudT::Ptr projectCloudX[2];      //点云投影x 0-标准 1-检测
    PointCloudT::Ptr projectCloudZ[2];      //点云投影z 0-标准 1-检测
    vector<double> outlierX[2];             //点云轮廓x  0-标准 1-检测
    vector<double> outlierZ[2];             //点云轮廓z  0-标准 1-检测
    vector<double> outlierX_Y[2];           //点云轮廓x_Y方向坐标值   0-标准 1-检测
    vector<double> outlierZ_Y[2];           //点云轮廓z_Y方向坐标值   0-标准 1-检测
    vector<double> diffX;                //点云轮廓差分x
    vector<double> diffZ;                //点云轮廓差分z
    vector<double> diffX_Y;              //点云轮廓差分x_Y方向
    vector<double> diffZ_Y;              //点云轮廓差分z_Y方向
};

#endif // PCLDEALTHREAD_H
