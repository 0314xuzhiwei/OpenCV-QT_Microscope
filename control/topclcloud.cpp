#include "topclcloud.h"
#include <QDebug>
#include <QMessageBox>
#include <iostream>
#include <QFile>

using namespace std;

ToPCLCloud* ToPCLCloud::myself = nullptr;

ToPCLCloud *ToPCLCloud::getInstance()
{
    if(ToPCLCloud::myself == nullptr)
    {
        ToPCLCloud::myself = new ToPCLCloud();
    }
    return ToPCLCloud::myself;
}

void ToPCLCloud::StartThread(int model)
{
    this->m_model = model;
    this->start();
}

void ToPCLCloud::StartThread(int model, QString fileName, pcl::PointCloud<PointT>::Ptr cloud)
{
    this->m_model = model;
    pcl::copyPointCloud(*cloud, *exportCloud);
    this->fileName = fileName;
    this->start();
}

void ToPCLCloud::getCloud(int num)
{
    if(num >= cloudList.size())
    {
        emit sendCloud(m_cloud);
        m_flage = 0;
        return;
    }
    //qDebug() << "size: " << cloudList.size();
    emit sendCloud(cloudList[num]);
    m_flage = 0;
    //qDebug() << "num: " << num;
}

pcl::PointCloud<PointT>::Ptr ToPCLCloud::getMCloud()
{
    PointCloudT::Ptr cloud;
    cloud.reset(new PointCloudT);
    pcl::copyPointCloud(*m_cloud, *cloud);
    return cloud;
}

void ToPCLCloud::run()
{
    static long int cloudSizeLaster = 0;
    this->threadState = 1;
    switch (m_model) {
    case ONEAXIS:
    {
        //进行点云数据转化
        if(m_flage != 1)
        {
            if(!cloudList.isEmpty())
            {
                cloudList.clear();
            }
            if(!m_cloud->empty())
            {
                m_cloud->clear();
            }
        }

        //获取对应的宽高数据等
        int width = 0;  //宽度
        int height = 0;  //高度
        double xpitch = 0;    //x间距
        double ypitch = 0.1;   //y间距
        double x=0, y=0, z=0;  //每隔点云的xyz坐标
        int* heightdata;
        heightdata = this->camera->toCloudData(width, height, xpitch);
        if(heightdata == nullptr || width==0 || height==0 || xpitch==0)
        {
            qDebug() << "camera err!";
            return;
        }
        int count=0;
        for(int i=0; i< width * height; i++)
        {
            z = heightdata[i]  * 0.00001;
            if(z == -10000)
            {
                continue;
            }
            count++;
        }
        //初始化点云数据
        PointCloudT::Ptr cloud;
        cloud.reset(new PointCloudT);
        cloud->width    = count;               //设置点云宽度
        cloud->height   = 1;               //设置点云为无序点
        for(int i=0; i< width * height; i++)
        {
            x = i % width * xpitch;     //x坐标
            y = i / width * ypitch;     //y坐标
            z = heightdata[i]  * 0.00001; //高度  转换成mm为单位
            if(z == -10000)
            {
                continue;
            }
            cloud->push_back(pcl::PointXYZ(x, y, z));
        }
        //发送点云数据
        emit sendCloud(cloud);
        //保存数据中
        cloudList.push_back(cloud);
        qDebug() << "PCL over To send PCL data";
        m_flage = 1;
        break;
    }
    case MOREAXIS:
    {
        if(m_flage != 2)
        {
            if(!cloudList.isEmpty())
            {
                cloudList.clear();
            }
            if(!m_cloud->empty())
            {
                m_cloud->clear();
            }
        }
        PointCloudT::Ptr cloud;
        //m_cloud.reset(new PointCloudT);
        cloud = this->camera->getCloud();
        //机械结构有问题，3D相机没有装好，导致点云图像发生了镜像
        int sizeC = cloud->size();
        for(int i=0; i< sizeC; i++)
        {
            cloud->points[i].x = (16.0 - cloud->points[i].x);
        }
        qDebug() << "获取完成! " << cloud->size();
        this->cloudList.push_back(cloud);
        qDebug() << "添加完成! " << this->cloudList.size();
        //点云拼接算法
        int size = m_cloud->size();
        int max = 0;
        qDebug() << "size: " << size;
        if(size != 0)
        {
            if(cloudSizeLaster > 1600 && (size - cloudSizeLaster)>1700)
            {
                for(int i=cloudSizeLaster; i<cloudSizeLaster+1700; i++)
                {
                    if(max < m_cloud->points[i].x)
                    {
                        max = m_cloud->points[i].x;
                    }
                }
            }
            else
            {
                for(int i=cloudSizeLaster; i<m_cloud->size(); i++)
                {
                    if(max < m_cloud->points[i].x)
                    {
                        max = m_cloud->points[i].x;
                    }
                }
            }
        }
        qDebug() << "max: " << max;
        cloudSizeLaster = size;
        for(int i=0; i<cloud->size(); i++)
        {
            cloud->points[i].x += max;
            m_cloud->push_back(cloud->points[i]);
        }
        m_flage = 2;
        qDebug() << m_cloud->size();
        break;
    }
    case KEEPDATA:
    {
        if(this->fileName.isEmpty())
        {
            qDebug() << "File Name Error";
            return;
        }
        qDebug() << "Begin To Export..." << this->m_cloud->points.size();
        //进行点云数据保存
        QStringList sp = this->fileName.split('.');
        //qDebug() << sp[1];
        if(sp[1] == "pcd")
        {
            pcl::io::savePCDFileBinary(this->fileName.toStdString(), *exportCloud);
        }
        else if(sp[1] == "bin")
        {
            //输出bin文件
            std::ofstream out;
            out.open(this->fileName.toStdString().c_str(), std::ios::out | std::ios::binary);
            int cloudSize = exportCloud->size();
            float ps = cloudSize / 100;
            int count=0, count_t=0;
            for (int i = 0; i < cloudSize; ++i)
            {
                float point_x = exportCloud->points[i].x;
                float point_y = exportCloud->points[i].y;
                float point_z = exportCloud->points[i].z;
                out.write(reinterpret_cast<const char*>(&point_x), sizeof(float));
                out.write(reinterpret_cast<const char*>(&point_y), sizeof(float));
                out.write(reinterpret_cast<const char*>(&point_z), sizeof(float));
                count ++;
                if(count >= ps)
                {
                    count_t ++;
                    char outStr[50] = {0};
                    sprintf(outStr, "Export Schedule: %d/100", count_t);
                    qDebug() << outStr;
                    count = 0;
                }
            }
            out.close();
        }
        else if(sp[1] == "txt")
        {
            //输出txt文件
            QFile file(this->fileName);
            //打开文件
            if(!file.open(QIODevice::WriteOnly | QIODevice::Text | QIODevice::Truncate))
            {
                //打开失败--不保存
                qDebug() << "File Write Error";
                return;
            }
            int cloudSize = exportCloud->size();
            float ps = cloudSize / 100;
            int count=0, count_t=0;
            for (int i = 0; i < cloudSize; ++i)
            {
                float point_x = exportCloud->points[i].x;
                float point_y = exportCloud->points[i].y;
                float point_z = exportCloud->points[i].z;
                char str[50] = {0};
                sprintf(str, "%f %f %f\n", point_x, point_y, point_z);
                file.write(str, strlen(str));
                count ++;
                if(count >= ps)
                {
                    count_t ++;
                    char outStr[50] = {0};
                    sprintf(outStr, "Export Schedule: %d/100", count_t);
                    qDebug() << outStr;
                    count = 0;
                }
            }
            file.close();
            break;
        }
        qDebug() << "Export Success!";
        exportCloud->clear();
    }
    default:
        break;
    }
    this->threadState = 0;
}

ToPCLCloud::ToPCLCloud()
{
    this->camera = cameraCtr::getInstance();
    //点云初始化
    m_cloud.reset(new PointCloudT);
    exportCloud.reset(new PointCloudT);
    //注册PointCloudT::Ptr类型
    qRegisterMetaType<PointCloudT::Ptr>("PointCloudT::Ptr");
    this->m_model = 0;
    this->fileName = "";
    this->threadState = 0;
    m_flage = 0;
}

int ToPCLCloud::getThreadState() const
{
    return threadState;
}

int ToPCLCloud::getCloudListSize()
{
    return cloudList.size();
}
