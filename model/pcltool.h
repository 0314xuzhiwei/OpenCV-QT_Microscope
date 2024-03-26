#ifndef PCLTOOL_H
#define PCLTOOL_H

#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/io/io.h>                      //I/O相关头文件申明
#include<pcl/visualization/cloud_viewer.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/point_cloud.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/conditional_removal.h>
#include <pcl/visualization/pcl_plotter.h>
#include <pcl/registration/icp.h>
#include<windows.h>
#include <QDebug>

using namespace std;

typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> PointCloudT;

class PCLTool
{
public:
    /// <summary>
    /// PCL文件类型
    /// </summary>
    enum PCLFile
    {
        PCDASCII = 0,
        PCDBINARY,
        PLY,
        BIN,
        TXT
    };
    /// <summary>
    /// 点云坐标系
    /// </summary>
    enum CLOUDP
    {
        CloudX = 0,
        CloudY,
        CloudZ
    };
    /// <summary>
    /// 点胶检测类型
    /// </summary>
    enum GlueType
    {
        GlueNone = 0,
        GlueMulti,
        GlueBreak,
        GlueLess
    };
    PCLTool();
    static void LoadCloud(string fileNameA, string fileNameB, PointCloudT::Ptr &cloud, int FileType, float err = 1.0);
    static void LoadCloud(string path, PointCloudT::Ptr &cloud, int FileType);
    static void cloudReduce(PointCloudT::Ptr cloudA, PointCloudT::Ptr cloudB, PointCloudT::Ptr &cloudOut, float err = 1.0);
    static void showCloud(PointCloudT::Ptr cloud);
    static void showCloud(PointCloudT::Ptr cloud, string name);
    static void showCloud(PointCloudT::Ptr cloud1, PointCloudT::Ptr cloud2, string name);
    static void saveCloud(PointCloudT::Ptr &cloud, string name, int type);
    static void saveCloud(PointCloudT::Ptr &cloud, string path);
    static bool checkDuplicate(PointCloudT::Ptr &cloud, PointT searchPoint);
    static void checkDuplicate(PointCloudT::Ptr &cloud);
    static void projectCloud(PointCloudT::Ptr cloudInput, PointCloudT::Ptr &cloudOut, int direct);
    static void outlineCloud(PointCloudT::Ptr cloud, vector<double> &outline, vector<double> *outY, int direct);
    static void outlineDiff(vector<double> outline, vector<double> &diff);
    static void outlineDiff(vector<double> outline1, vector<double> outline2, vector<double> &diff);
    static bool decideGlue(vector<double> diff, float threshold, int numThrd, int type);
    static void cloudOutlierRemovalRadius(PointCloudT::Ptr cloud, PointCloudT::Ptr &cloudOut, int radius = 20, float radiusSearch = 0.8);
    static void cloudOutlierRemoval(PointCloudT::Ptr cloud, PointCloudT::Ptr &cloudOut, bool negative = false, int meanK = 50, float thresh = 0.5);
    static void cloudCondition(PointCloudT::Ptr cloud, PointCloudT::Ptr &cloudOut, string axis = "z", float min = 0.0, float max = 0.8);
    static void showPointPlot(vector<double> data_y, vector<double> data_x = vector<double>(), string title = "Display Data", string titleX = "X", string titleY = "Y", string name = "Display Line");
    static bool cloudICP(PointCloudT::Ptr cloud, PointCloudT::Ptr cloudOut, PointCloudT::Ptr &cloudAligned, double score = 0.85);
protected:
    static void stringSplit(string str, const char split, vector<string> &outStr);
};

#endif // PCLTOOL_H
