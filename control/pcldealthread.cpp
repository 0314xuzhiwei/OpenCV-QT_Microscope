#include "pcldealthread.h"
#include <QDebug>

PCLDealThread::PCLDealThread()
{
    this->basicCloud.reset(new PointCloudT);
    this->standCloud.reset(new PointCloudT);
    this->checkCloud.reset(new PointCloudT);
    this->projectCloudX[0].reset(new PointCloudT);
    this->projectCloudX[1].reset(new PointCloudT);
    this->projectCloudZ[0].reset(new PointCloudT);
    this->projectCloudZ[1].reset(new PointCloudT);
}

void PCLDealThread::run()
{

}

void PCLDealThread::showDiff() const
{
    PCLTool::showPointPlot(diffX, vector<double>(), "diff Outlier", "X", "Y", "X-Axis");
    PCLTool::showPointPlot(diffZ, vector<double>(), "diff Outlier", "X", "Y", "Z-Axis");
    PCLTool::showPointPlot(diffX_Y, vector<double>(), "diff Y Value", "X", "Y", "X-Y-Axis");
    PCLTool::showPointPlot(diffZ_Y, vector<double>(), "diff Y Value", "X", "Y", "Z-Y-Axis");
}

void PCLDealThread::showOutlier() const
{
    PCLTool::showPointPlot(outlierX[0], vector<double>(), "Stand Cloud Outlier", "X", "Y", "X-Axis-outlier");
    PCLTool::showPointPlot(outlierX[1], vector<double>(), "Check Cloud Outlier", "X", "Y", "X-Axis-outlier");
    PCLTool::showPointPlot(outlierZ[0], vector<double>(), "Stand Cloud Outlier", "X", "Y", "Z-Axis-outlier");
    PCLTool::showPointPlot(outlierZ[1], vector<double>(), "Check Cloud Outlier", "X", "Y", "Z-Axis-outlier");
    PCLTool::showPointPlot(outlierX_Y[0], vector<double>(), "Stand Cloud Y Value", "X", "Y", "X-Y-Axis-outlier");
    PCLTool::showPointPlot(outlierX_Y[1], vector<double>(), "Check Cloud Y Value", "X", "Y", "X-Y-Axis-outlier");
    PCLTool::showPointPlot(outlierZ_Y[0], vector<double>(), "Stand Cloud Y Value", "X", "Y", "Z-Y-Axis-outlier");
    PCLTool::showPointPlot(outlierZ_Y[0], vector<double>(), "Check Cloud Y Value", "X", "Y", "Z-Y-Axis-outlier");
}

void PCLDealThread::showProjectCloud(QString name) const
{
    PointCloudT::Ptr cloud1, cloud2;
    cloud1.reset(new PointCloudT);
    cloud2.reset(new PointCloudT);
    for(int i=0; i<projectCloudZ[0]->size(); i++)
    {
        cloud1->push_back(projectCloudZ[0]->points[i]);
    }
    for(int i=0; i<projectCloudX[0]->size(); i++)
    {
        cloud1->push_back(projectCloudX[0]->points[i]);
    }
    for(int i=0; i<projectCloudZ[1]->size(); i++)
    {
        cloud2->push_back(projectCloudZ[1]->points[i]);
    }
    for(int i=0; i<projectCloudX[1]->size(); i++)
    {
        cloud2->push_back(projectCloudX[1]->points[i]);
    }
    PCLTool::showCloud(cloud1, cloud2, name.toStdString());
}

PointCloudT::Ptr PCLDealThread::getCheckCloud() const
{
    PointCloudT::Ptr cloud;
    cloud.reset(new PointCloudT);
    pcl::copyPointCloud(*checkCloud, *cloud);
    return cloud;
}

void PCLDealThread::setCheckCloud(const PointCloudT::Ptr &value)
{
    if(!checkCloud->empty())
    {
        checkCloud->clear();
    }
    pcl::copyPointCloud(*value, *checkCloud);
}

PointCloudT::Ptr PCLDealThread::getStandCloud() const
{
    PointCloudT::Ptr cloud;
    cloud.reset(new PointCloudT);
    pcl::copyPointCloud(*standCloud, *cloud);
    return cloud;
}

void PCLDealThread::setStandCloud(const PointCloudT::Ptr &value)
{
    if(!standCloud->empty())
    {
        standCloud->clear();
    }
    pcl::copyPointCloud(*value, *standCloud);
}

PointCloudT::Ptr PCLDealThread::getBasicCloud() const
{
    PointCloudT::Ptr cloud;
    cloud.reset(new PointCloudT);
    pcl::copyPointCloud(*basicCloud, *cloud);
    return cloud;
}

void PCLDealThread::setBasicCloud(const PointCloudT::Ptr &value)
{
    if(!basicCloud->empty())
    {
        basicCloud->clear();
    }
    pcl::copyPointCloud(*value, *basicCloud);
}

int PCLDealThread::cloudReduce()
{
    if(checkCloud->empty())
    {
        return -1;
    }
    if(basicCloud->empty())
    {
        return -2;
    }
    PointCloudT::Ptr cloud;
    cloud.reset(new PointCloudT);
    PCLTool::cloudReduce(checkCloud, basicCloud, cloud, 0.5);
    checkCloud->clear();
    pcl::copyPointCloud(*cloud, *checkCloud);
    cloud->clear();
    return 0;
}

int PCLDealThread::cloudOutlierRemoval()
{
    if(checkCloud->empty())
    {
        return -1;
    }
    PointCloudT::Ptr cloud;
    cloud.reset(new PointCloudT);
    //PCLTool::cloudOutlierRemovalRadius(checkCloud, cloud, 70, 0.4);
    PCLTool::cloudCondition(checkCloud, cloud,  "z", 0.0, 1.6);
    PCLTool::cloudCondition(cloud, cloud, "x", 7.0, 11.0);
    checkCloud->clear();
    pcl::copyPointCloud(*cloud, *checkCloud);
    cloud->clear();
    return 0;
}

int PCLDealThread::projectionCloud()
{
    if(checkCloud->empty())
    {
        return -1;
    }
    if(standCloud->empty())
    {
        return -2;
    }
    if(!projectCloudX[1]->empty())
    {
        projectCloudX[1]->clear();
        projectCloudZ[1]->clear();
        projectCloudX[0]->clear();
        projectCloudZ[0]->clear();
    }
    PCLTool::projectCloud(checkCloud, projectCloudX[1], PCLTool::CloudX);
    PCLTool::projectCloud(checkCloud, projectCloudZ[1], PCLTool::CloudZ);
    PCLTool::projectCloud(standCloud, projectCloudX[0], PCLTool::CloudX);
    PCLTool::projectCloud(standCloud, projectCloudZ[0], PCLTool::CloudZ);
    return 0;
}

int PCLDealThread::cloudOutlier()
{
    if(projectCloudX[1]->empty() || projectCloudZ[1]->empty())
    {
        return -1;
    }
    if(projectCloudX[0]->empty() || projectCloudZ[0]->empty())
    {
        return -2;
    }
    if(!outlierX[0].empty())
    {
        outlierX[0].clear();
        outlierX_Y[0].clear();
        outlierZ[0].clear();
        outlierZ_Y[0].clear();
        outlierX[1].clear();
        outlierX_Y[1].clear();
        outlierZ[1].clear();
        outlierZ_Y[1].clear();
    }
    //标准胶条
    PCLTool::outlineCloud(projectCloudX[0], outlierX[0], outlierX_Y, PCLTool::CloudX);
    PCLTool::outlineCloud(projectCloudZ[0], outlierZ[0], outlierZ_Y, PCLTool::CloudZ);
    //检测胶条
    PCLTool::outlineCloud(projectCloudX[1], outlierX[1], outlierX_Y+1, PCLTool::CloudX);
    PCLTool::outlineCloud(projectCloudZ[1], outlierZ[1], outlierZ_Y+1, PCLTool::CloudZ);
    return 0;
}

int PCLDealThread::cloudDiff()
{
    if(outlierX[1].empty() || outlierZ[1].empty() || outlierX_Y[1].empty() || outlierZ_Y[1].empty())
    {
        return -1;
    }
    if(outlierX[0].empty() || outlierZ[0].empty() || outlierX_Y[0].empty() || outlierZ_Y[0].empty())
    {
        return -2;
    }
    if(!diffX.empty())
    {
        diffX.clear();
        diffZ.clear();
        diffX_Y.clear();
        diffZ_Y.clear();
    }
    //检测胶条 - 标准胶条
    PCLTool::outlineDiff(outlierX[0], outlierX[1], diffX);
    PCLTool::outlineDiff(outlierZ[0], outlierZ[1], diffZ);
    PCLTool::outlineDiff(outlierX_Y[0], outlierX_Y[1], diffX_Y);
    PCLTool::outlineDiff(outlierZ_Y[0], outlierZ_Y[1], diffZ_Y);
    return 0;
}

int PCLDealThread::cloudCheck()
{
    if(diffX.empty() || diffZ.empty() || diffX_Y.empty() || diffZ_Y.empty())
    {
        return -1;
    }
    bool ret = false;
    //断胶
    ret = PCLTool::decideGlue(diffX_Y, 20, 20, PCLTool::GlueBreak);
    if(ret)
    {
        return PCLTool::GlueBreak;
    }
    ret = PCLTool::decideGlue(diffZ_Y, 20, 20, PCLTool::GlueBreak);
    if(ret)
    {
        return PCLTool::GlueBreak;
    }
    //多胶
    ret = PCLTool::decideGlue(diffX, 1.0, 20, PCLTool::GlueMulti);
    if(ret)
    {
        return PCLTool::GlueMulti;
    }
    ret = PCLTool::decideGlue(diffZ, 1.0, 20, PCLTool::GlueMulti);
    if(ret)
    {
        return PCLTool::GlueMulti;
    }
    //少胶
    ret = PCLTool::decideGlue(diffX, -0.5, 20, PCLTool::GlueLess);
    if(ret)
    {
        return PCLTool::GlueLess;
    }
    ret = PCLTool::decideGlue(diffZ, -0.5, 20, PCLTool::GlueLess);
    if(ret)
    {
        return PCLTool::GlueLess;
    }
    return PCLTool::GlueNone;
}
