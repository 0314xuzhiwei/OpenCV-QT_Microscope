#include "pcltool.h"

PCLTool::PCLTool()
{

}


/// <summary>
/// 分割string字符串
/// </summary>
/// <param name="str"></param>		要分割的字符串
/// <param name="split"></param>	分割字符
/// <param name="outStr"></param>	输出分割开的字符
void PCLTool::stringSplit(string str, const char split, vector<string>& outStr)
{
    istringstream iss(str);	// 输入流
    string token;			// 接收缓冲区
    while (getline(iss, token, split))	// 以split为分隔符
    {
        outStr.push_back(token);
    }
}
/// <summary>
/// 加载点云数据，同时进行点云相减
/// </summary>
/// <param name="fileNameA"></param>	点云A
/// <param name="fileNameB"></param>	点云B
/// <param name="cloud"></param>		相减后输出点云（B-A）
/// <param name="model"></param>		文件加载方式
void PCLTool::LoadCloud(string fileNameA, string fileNameB, PointCloudT::Ptr& cloud, int FileType, float err)
{
    DWORD start_time = GetTickCount();
    switch (FileType)
    {
    case PCDASCII:
    case PCDBINARY:		//还存在问题，会没有顺序
    {
        PointCloudT::Ptr cloudA(new PointCloudT);
        PointCloudT::Ptr cloudB(new PointCloudT);
        pcl::io::loadPCDFile(fileNameA, *cloudA);
        qDebug() << "Load CloudA Over";
        pcl::io::loadPCDFile(fileNameB, *cloudB);
        qDebug() << "Load CloudB Over" ;
        //进行点云相减
        int len = cloudA->size();
        float x = 0, y = 0, z = 0;
        if (len > cloudB->size())
        {
            len = cloud->size();
        }
        int pt = len / 100;
        int count = 0, count2 = 0;
        for (int i = 0; i < len; i++)
        {
            x = cloudA->points[i].x;
            y = cloudA->points[i].y;
            float xb = cloudB->points[i].x;
            float yb = cloudB->points[i].y;
            z = cloudB->points[i].z - cloudA->points[i].z;
            count2++;
            if (count2 > pt)
            {
                count++;
                qDebug() << "Load Schedule: " << count << "/100" ;
                count2 = 0;
            }
            if (abs(z) < err)
            {
                continue;
            }
            cloud->push_back(PointT(x, y, z));
        }
        break;
    }
    case TXT:
    {
        std::fstream fileA, fileB;
        fileA.open(fileNameA, std::ios::in);
        fileB.open(fileNameB, std::ios::in);
        float X_A = 0, Y_A = 0, Z_A = 0;
        float X_B = 0, Y_B = 0, Z_B = 0;
        string lineA, lineB; //保存读入的每一行
        int num = 0;
        while (getline(fileA, lineA) && getline(fileB, lineB))
        {
            vector<string> stringListA, stringListB;
            stringSplit(lineA, ' ', stringListA);
            stringSplit(lineB, ' ', stringListB);
            X_A = atof(stringListA[0].c_str());
            Y_A = atof(stringListA[1].c_str());
            Z_A = atof(stringListA[2].c_str());
            X_B = atof(stringListB[0].c_str());
            Y_B = atof(stringListB[1].c_str());
            Z_B = atof(stringListB[2].c_str());
            if (abs(Z_B - Z_A) < err)
            {
                continue;
            }
            //存入相减后的数据
            cloud->push_back(PointT(X_B, Y_B, (Z_B - Z_A)));
            num++;
            qDebug() << "Read Data Len: " << num;
        }
    }
    default:
        break;
    }
    DWORD end_time = GetTickCount();
    std::cout << "载入点云耗时: " << (end_time - start_time) << " ms" << std::endl;
}
/// <summary>
/// 按文件加载点云
/// </summary>
/// <param name="path"></param>		文件路径
/// <param name="cloud"></param>		输出点云
/// <param name="FileType"></param>		文件类型
void PCLTool::LoadCloud(string path, PointCloudT::Ptr& cloud, int FileType)
{
    DWORD start_time = GetTickCount();
    switch (FileType)
    {
    case PCDASCII:
    case PCDBINARY:
    {
        pcl::io::loadPCDFile(path, *cloud);
        break;
    }
    case BIN:
    {
        std::fstream file;
        file.open(path, std::ios::in | std::ios::binary);
        float x = 0, y = 0, z = 0;
        while (file >> x >> y >> z)
        {
            cloud->push_back(PointT(x, y, z));
            qDebug() << x << " " << y << " " << z;
        }
    }
    case TXT:
    {
        std::fstream file;
        file.open(path, std::ios::in);
        string line; //保存读入的每一行
        float x = 0, y = 0, z = 0;
        while (getline(file, line))
        {
            vector<string> stringList;
            stringSplit(line, ' ', stringList);
            x = atof(stringList[0].c_str());
            y = atof(stringList[1].c_str());
            z = atof(stringList[2].c_str());
            cloud->push_back(PointT(x, y, z));
            qDebug() << x << " " << y << " " << z ;
        }
        break;
    }
    default:
        break;
    }
    DWORD end_time = GetTickCount();
    qDebug() << "载入点云耗时: " << (end_time - start_time) << " ms" ;
}
/// <summary>
/// 点云相减  cloudOut = cloudA - cloudB
/// </summary>
/// <param name="cloudA"></param>  输入点云
/// <param name="cloudB"></param>  输入点云
/// <param name="cloudOut"></param> 输出点云
void PCLTool::cloudReduce(PointCloudT::Ptr cloudA, PointCloudT::Ptr cloudB, PointCloudT::Ptr& cloudOut, float err)
{
    int len = cloudA->size();
    int lenB = cloudB->size();
    //点云相减
    if (len > lenB)
    {
        len = lenB;
    }
    cloudOut->height = 1;
    double x = 0, y = 0, z = 0;
    for (int i = 0; i < len; i++)
    {
        z = cloudA->points[i].z - cloudB->points[i].z;
        if (abs(z) < err || z < 0)
        {
            continue;
        }
        x = cloudA->points[i].x;
        y = cloudA->points[i].y;
        z = cloudA->points[i].z - cloudB->points[i].z;
        cloudOut->push_back(PointT(x, y, z));
    }
}
/// <summary>
/// 显示点云数据
/// </summary>
/// <param name="cloud"></param>	输入点云
void PCLTool::showCloud(PointCloudT::Ptr cloud)
{
    pcl::visualization::CloudViewer viewer("cloud viewer");
    viewer.showCloud(cloud);
    while (!viewer.wasStopped());
}
/// <summary>
/// 显示点云数据
/// </summary>
/// <param name="cloud"></param>	输入点云
/// <param name="name"></param>     窗口名称
void PCLTool::showCloud(PointCloudT::Ptr cloud, string name)
{
    pcl::visualization::CloudViewer viewer(name.c_str());
    viewer.showCloud(cloud);
    while (!viewer.wasStopped());
}
/// <summary>
/// 显示点云数据
/// </summary>
/// <param name="cloud1"></param>	输入点云1
/// <param name="cloud2"></param>	输入点云2
/// <param name="name"></param>     窗口名称
void PCLTool::showCloud(PointCloudT::Ptr cloud1, PointCloudT::Ptr cloud2, string name)
{
    boost::shared_ptr< pcl::visualization::PCLVisualizer > viewer(new pcl::visualization::PCLVisualizer(name.c_str()));
    viewer->initCameraParameters();
    //在一个视图里显示两张点云图
    int v1(0);
    viewer->createViewPort(0.0, 0.0, 1.0 / 2.0, 1.0, v1);
    viewer->setBackgroundColor(0, 0, 0, v1);
    //viewer->addText("Radius:0.01", 10, 10, "v1 text", v1);
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> source_color(cloud1, 255, 255, 255);
    viewer->addPointCloud<pcl::PointXYZ>(cloud1, source_color, "cloud1", v1);

    int v2(0);
    viewer->createViewPort(1.0 / 2.0, 0.0, 1.0, 1.0, v2);
    viewer->setBackgroundColor(28, 28, 28, v2);
    //viewer->addText("Radius:0.1", 10, 10, "v2 text", v2);
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> source_color2(cloud2, 0, 0, 255);
    viewer->addPointCloud<pcl::PointXYZ>(cloud2, source_color2, "cloud2", v2);

    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "cloud1");
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "cloud2");

    viewer->addCoordinateSystem(1.0);
    viewer->spin();
}
/// <summary>
/// 保存点云数据
/// </summary>
/// <param name="cloud"></param>	输入点云
/// <param name="name"></param>		文件名称
/// <param name="type"></param>	保存文件类型
void PCLTool::saveCloud(PointCloudT::Ptr& cloud, string name, int type)
{
    string path = "./data/pcdData/";
    switch (type)
    {
    case PCDASCII:
    {
        path += (name + ".pcd");
        pcl::io::savePCDFileASCII(path, *cloud);
        break;
    }
    case PCDBINARY:
    {
        path += (name + ".pcd");
        pcl::io::savePCDFileBinary(path, *cloud);
        break;
    }
    case PLY:
    {
        path += (name + ".ply");
        qDebug() << "暂时不支持 " ;
        //pcl::io::(path, cloud);
        break;
    }
    case BIN:
    {
        path += (name + ".bin");
        std::ofstream out;
        out.open(path, std::ios::out | std::ios::binary);
        int cloudSize = cloud->size();
        float ps = cloudSize / 100;
        int count = 0, count_t = 0;
        for (int i = 0; i < cloudSize; ++i)
        {
            float point_x = cloud->points[i].x;
            float point_y = cloud->points[i].y;
            float point_z = cloud->points[i].z;
            out.write(reinterpret_cast<const char*>(&point_x), sizeof(float));
            out.write(reinterpret_cast<const char*>(&point_y), sizeof(float));
            out.write(reinterpret_cast<const char*>(&point_z), sizeof(float));
            count++;
            if (count >= ps)
            {
                count_t++;
                char outStr[50] = { 0 };
                sprintf_s(outStr, "Export Schedule: %d/100", count_t);
                qDebug() << outStr;
                count = 0;
            }
        }
        out.close();
        break;
    }
    case TXT:
    {
        path += (name + ".txt");
        std::fstream file;
        file.open(path, std::ios::out);
        int cloudSize = cloud->size();
        float ps = cloudSize / 100;
        int count = 0, count_t = 0;
        for (int i = 0; i < cloudSize; ++i)
        {
            float point_x = cloud->points[i].x;
            float point_y = cloud->points[i].y;
            float point_z = cloud->points[i].z;
            char str[50] = { 0 };
            sprintf_s(str, "%f %f %f\n", point_x, point_y, point_z);
            file.write(str, strlen(str));
            count++;
            if (count >= ps)
            {
                count_t++;
                char outStr[50] = { 0 };
                sprintf_s(outStr, "Export Schedule: %d/100", count_t);
                qDebug() << outStr;
                count = 0;
            }
        }
        break;
    }
    default:
        break;
    }
    qDebug() << "Export Success!";
}

void PCLTool::saveCloud(PointCloudT::Ptr &cloud, string path)
{
    int type = PLY;
    vector<string> strVector;
    stringSplit(path, '.', strVector);
    if(strVector[1] == "pcd")
    {
        type = PCDBINARY;
    }
    else if(strVector[1] == "txt")
    {
        type = TXT;
    }
    else if(strVector[1] == "bin")
    {
        type = BIN;
    }
    switch (type)
    {
    case PCDASCII:
    {
        pcl::io::savePCDFileASCII(path, *cloud);
        break;
    }
    case PCDBINARY:
    {
        pcl::io::savePCDFileBinary(path, *cloud);
        break;
    }
    case PLY:
    {
        qDebug() << "暂时不支持 " ;
        break;
    }
    case BIN:
    {
        std::ofstream out;
        out.open(path, std::ios::out | std::ios::binary);
        int cloudSize = cloud->size();
        float ps = cloudSize / 100;
        int count = 0, count_t = 0;
        for (int i = 0; i < cloudSize; ++i)
        {
            float point_x = cloud->points[i].x;
            float point_y = cloud->points[i].y;
            float point_z = cloud->points[i].z;
            out.write(reinterpret_cast<const char*>(&point_x), sizeof(float));
            out.write(reinterpret_cast<const char*>(&point_y), sizeof(float));
            out.write(reinterpret_cast<const char*>(&point_z), sizeof(float));
            count++;
            if (count >= ps)
            {
                count_t++;
                char outStr[50] = { 0 };
                sprintf_s(outStr, "Export Schedule: %d/100", count_t);
                qDebug() << outStr;
                count = 0;
            }
        }
        out.close();
        break;
    }
    case TXT:
    {
        std::fstream file;
        file.open(path, std::ios::out);
        int cloudSize = cloud->size();
        float ps = cloudSize / 100;
        int count = 0, count_t = 0;
        for (int i = 0; i < cloudSize; ++i)
        {
            float point_x = cloud->points[i].x;
            float point_y = cloud->points[i].y;
            float point_z = cloud->points[i].z;
            char str[50] = { 0 };
            sprintf_s(str, "%f %f %f\n", point_x, point_y, point_z);
            file.write(str, strlen(str));
            count++;
            if (count >= ps)
            {
                count_t++;
                char outStr[50] = { 0 };
                sprintf_s(outStr, "Export Schedule: %d/100", count_t);
                qDebug() << outStr;
                count = 0;
            }
        }
        break;
    }
    default:
        break;
    }
    qDebug() << "Export Success!";
}
/// <summary>
/// 对点云进行查重——速度太慢，放弃
/// </summary>
/// <param name="cloud"></param>		查重点云
/// <param name="searchPoint"></param>	查重点
/// <returns></returns>
bool PCLTool::checkDuplicate(PointCloudT::Ptr& cloud, PointT searchPoint)
{
    if (cloud->empty())
    {
        return false;
    }
    pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
    kdtree.setInputCloud(cloud);
    float radius = 0.0001;
    std::vector<int> pointIdxRadiusSearch;
    std::vector<float> pointRadiusSquaredDistance;

    if (kdtree.radiusSearch(searchPoint, radius, pointIdxRadiusSearch, pointRadiusSquaredDistance) > 0)
    {
        return true;
    }
    return false;
}
/// <summary>
/// 对点云进行查重——速度太慢，放弃
/// </summary>
/// <param name="cloud"></param>  输入点云
void PCLTool::checkDuplicate(PointCloudT::Ptr& cloud)
{
    if (cloud->empty())
    {
        return;
    }
    pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
    kdtree.setInputCloud(cloud);
    float radius = 0.0001;
    std::vector<int> pointIdxRadiusSearch;
    std::vector<float> pointRadiusSquaredDistance;
    PointT searchPoint;
    DWORD checkStart = GetTickCount64();
    for (int i = 0; i < cloud->size(); i++)
    {
        searchPoint = cloud->points[i];
        if (kdtree.radiusSearch(searchPoint, radius, pointIdxRadiusSearch, pointRadiusSquaredDistance) > 1)
        {
            qDebug() << "当前处理到: " << i ;
            qDebug() << "清除重复点云数: " << pointIdxRadiusSearch.size() ;
            for (int j = 1; j < pointIdxRadiusSearch.size(); j++)
            {
                PointCloudT::iterator it = cloud->begin() + pointIdxRadiusSearch[j];		//找到重复的点云
                cloud->erase(it);	//删除对应的点云
                for (int k = j+1; k < pointIdxRadiusSearch.size(); k++)
                {
                    pointIdxRadiusSearch[k] -= 1;
                }
            }
            kdtree.setInputCloud(cloud);
        }
    }
    DWORD checkEnd = GetTickCount64();
    qDebug() << "查重耗时: " << checkEnd - checkStart << " ms" ;
}
/// <summary>
///  对点云做投影
/// </summary>
/// <param name="cloudInput"></param>	输入点云
/// <param name="cloudOut"></param>		输出点云
/// <param name="direct"></param>		投影方向
void PCLTool::projectCloud(PointCloudT::Ptr cloudInput, PointCloudT::Ptr& cloudOut, int direct)
{
    cloudOut->width = cloudInput->width;
    cloudOut->height = cloudInput->height;
    int cloudSize = cloudInput->size();
    float x = 0, y = 0, z = 0;
    float ps = cloudSize / 100;
    int count = 0, count_t = 0;
    DWORD start_time = GetTickCount64();
    for (int i = 0; i < cloudSize; i++)
    {

        x = cloudInput->points[i].x;
        y = cloudInput->points[i].y;
        z = cloudInput->points[i].z;
        //进行投影
        if (direct == CloudX) //X方向
        {
            x = 0;
        }
        else if (direct == CloudY)  //Y方向
        {
            y = 0;
        }
        else if (direct == CloudZ)  //Z方向
        {
            z = 0;
        }
        //PointT searchP = PointT(x, y, z);
        //显示进度
        count++;
        if (count >= ps)
        {
            count_t++;
            char outStr[50] = { 0 };
            sprintf_s(outStr, "Projection Schedule: %d/100", count_t);
            qDebug() << outStr ;
            count = 0;
        }
        //保存投影数据
        cloudOut->push_back(PointT(x, y, z));
    }
    DWORD end_time = GetTickCount64();
    qDebug() << "投影耗时: " << end_time - start_time << " ms" << endl;
}
/// <summary>
/// 获取点云轮廓
/// </summary>
/// <param name="cloud"></param>	输入点云
/// <param name="outline"></param>	输出轮廓
/// <param name="outY"></param>		输出Y轴坐标----Y轴投影失效
/// <param name="direct"></param>	轮廓方向
void PCLTool::outlineCloud(PointCloudT::Ptr cloud, vector<double>& outline, vector<double>* outY, int direct)
{
    float row = 0;
    float min = 0, max = 0;
    float y = 0;
    if (direct == CloudX)  //投影x方向的轮廓边缘
    {
        row = cloud->points[0].y;
        min = cloud->points[0].z;
        y = cloud->points[0].y;
    }
    else if (direct == CloudY)  //投影y方向的轮廓边缘
    {
        row = cloud->points[0].x;
        min = cloud->points[0].z;
    }
    else if (direct == CloudZ) //投影z方向的轮廓边缘
    {
        row = cloud->points[0].y;
        min = cloud->points[0].x;
        y = cloud->points[0].y;
    }

    int cloudSize = cloud->size();

    for (int i = 0; i < cloudSize; i++)
    {
        if (direct == CloudX)  //投影x方向的轮廓边缘
        {
            if (cloud->points[i].y == row)
            {
                if (min > cloud->points[i].z)
                {
                    min = cloud->points[i].z;
                    y = cloud->points[i].y;
                    continue;
                }
                if (max < cloud->points[i].z)
                {
                    max = cloud->points[i].z;
                    continue;
                }
            }
            else
            {
                outline.push_back((max - min));
                outY->push_back(y);
                min = cloud->points[i].z;
                row = cloud->points[i].y;
                max = cloud->points[i].z;
                y = row;
            }
        }
        else if (direct == CloudY)  //投影y方向的轮廓边缘
        {
            if (cloud->points[i].x == row)
            {
                if (min > cloud->points[i].z)
                {
                    min = cloud->points[i].z;
                    continue;
                }
                if (max < cloud->points[i].z)
                {
                    max = cloud->points[i].z;
                    continue;
                }
            }
            else
            {
                outline.push_back((max - min));
                min = cloud->points[i].z;
                row = cloud->points[i].x;
                max = cloud->points[i].z;
            }
        }
        else if (direct == CloudZ) //投影z方向的轮廓边缘
        {
            if (cloud->points[i].y == row)
            {
                if (min > cloud->points[i].x)
                {
                    min = cloud->points[i].x;
                    y = cloud->points[i].y;
                    continue;
                }
                if (max < cloud->points[i].x)
                {
                    max = cloud->points[i].x;
                    continue;
                }
            }
            else
            {
                outline.push_back((max - min));
                outY->push_back(y);
                min = cloud->points[i].x;
                row = cloud->points[i].y;
                max = cloud->points[i].x;
                y = row;
            }
        }
    }
}
/// <summary>
/// 对点云进行前后差分
/// </summary>
/// <param name="outline"></param>	点云轮廓
/// <param name="diff"></param>		差分结果
void PCLTool::outlineDiff(vector<double> outline, vector<double>& diff)
{
    int size = outline.size() - 1;
    for (int i = 0; i < size; i++)
    {
        diff.push_back((outline[i + 1] - outline[i]));
    }
}
/// <summary>
/// 两数据差分
/// </summary>
/// <param name="outline1"></param>		被差分数据
/// <param name="outline2"></param>		差分数据
/// <param name="diff"></param>			差分结果
void PCLTool::outlineDiff(vector<double> outline1, vector<double> outline2, vector<double>& diff)
{
    int len = outline1.size();
    if (len > outline2.size())
    {
        len = outline2.size();
    }
    //进行差分 outline2 - outline1
    for (int i = 0; i < len; i++)
    {
        diff.push_back((outline2[i] - outline1[i]));
    }
}
/// <summary>
/// 点胶检测
/// </summary>
/// <param name="diff"></param>			差分数据
/// <param name="threshold"></param>	阈值
/// <param name="numThrd"></param>		数量阈值
/// <param name="type"></param>			检测类型
/// <returns></returns>
bool PCLTool::decideGlue(vector<double> diff, float threshold, int numThrd, int type)
{
    int size = diff.size();
    int count = 0;
    for (int i = 0; i < size; i++)
    {
        if (type == GlueMulti || type == GlueBreak)
        {
            if (diff[i] > threshold)
            {
                count++;
                if (count >= numThrd)
                {
                    return true;
                }
            }
        }
        else if (type == GlueLess)
        {
            if (diff[i] < threshold)
            {
                count++;
                if (count >= numThrd)
                {
                    return true;
                }
            }
        }
    }
    return false;
}
/// <summary>
/// 对点云进行离群点剔除滤波
/// </summary>
/// <param name="cloud"></param>			输入点云
/// <param name="cloudOut"></param>			输出点云
/// <param name="radius"></param>			搜索邻近点数
/// <param name="radiusSearch"></param>		半径
void PCLTool::cloudOutlierRemovalRadius(PointCloudT::Ptr cloud, PointCloudT::Ptr& cloudOut, int radius, float radiusSearch)
{
    DWORD start_time = GetTickCount();

    pcl::RadiusOutlierRemoval<PointT> outrem; //创建滤波器
    outrem.setInputCloud(cloud);       //设置输入点云
    outrem.setRadiusSearch(radiusSearch);       //设置半径为radiusSearch的范围内找临近点
    outrem.setMinNeighborsInRadius(radius); //设置查询点的邻域点集数小于radius的删除
    // apply filter
    outrem.filter(*cloudOut); //执行条件滤波   在半径为0.8 在此半径内必须要有两个邻居点，此点才会保存

    DWORD end_time = GetTickCount();
    qDebug() << "滤波耗时: " << (end_time - start_time) << " ms";
}
/// <summary>
/// 离群点剔除滤波
/// </summary>
/// <param name="cloud"></param>		输入点云
/// <param name="cloudOut"></param>		输出点云
/// <param name="negative"></param>		输出点云类型是滤波结果还是剔除部分
/// <param name="meanK"></param>		查询的邻近点数
/// <param name="thresh"></param>		离群点阈值
void PCLTool::cloudOutlierRemoval(PointCloudT::Ptr cloud, PointCloudT::Ptr& cloudOut,
                        bool negative, int meanK, float thresh)
{
    DWORD start_time = GetTickCount();

    /*创建滤波器，对每个点分析的临近点的个数设置为50 ，并将标准差的倍数设置为1  这意味着如果一
    个点的距离超出了平均距离一个标准差以上，则该点被标记为离群点，并将它移除，存储起来*/
    pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;  //创建滤波器对象
    sor.setInputCloud(cloud);							//设置待滤波的点云
    sor.setMeanK(meanK);								//设置在进行统计时考虑查询点临近点数
    sor.setStddevMulThresh(thresh);						//设置判断是否为离群点的阀值
    sor.setNegative(negative);							//true：滤波结果取反，被过滤的点  false: 获取过滤后的点
    sor.filter(*cloudOut);								//存储

    DWORD end_time = GetTickCount();
    qDebug() << "滤波耗时: " << (end_time - start_time) << " ms";
}
/// <summary>
/// 条件限定滤波器
/// </summary>
/// <param name="cloud"></param>		输入点云
/// <param name="cloudOut"></param>		输出点云
/// <param name="axis"></param>			滤波方向轴
/// <param name="min"></param>			下限值
/// <param name="max"></param>			上限值
void PCLTool::cloudCondition(PointCloudT::Ptr cloud, PointCloudT::Ptr& cloudOut, string axis, float min, float max)
{
    DWORD start_time = GetTickCount();
    //创建条件限定的下的滤波器
    pcl::ConditionAnd<pcl::PointXYZ>::Ptr range_cond(new pcl::ConditionAnd<pcl::PointXYZ>()); //创建条件定义对象
    //添加在Z字段上大于0的比较算子
    //GT greater than
    //EQ equal
    //LT less than
    //GE greater than or equal
    //LE less than
    //为条件定义对象添加比较算子
    range_cond->addComparison(pcl::FieldComparison<pcl::PointXYZ>::ConstPtr(new pcl::FieldComparison<pcl::PointXYZ>(axis.c_str(), pcl::ComparisonOps::GT, min))); //添加在Z字段上大于0的比较算子
    range_cond->addComparison(pcl::FieldComparison<pcl::PointXYZ>::ConstPtr(new pcl::FieldComparison<pcl::PointXYZ>(axis.c_str(), pcl::ComparisonOps::LT, max))); //添加在Z字段上小于0.8的比较算子
    // 创建滤波器并用条件定义对象初始化
    pcl::ConditionalRemoval<pcl::PointXYZ> condrem;
    condrem.setCondition(range_cond);
    condrem.setInputCloud(cloud);   //输入点云
    //condrem.setKeepOrganized(false); //设置保持点云的结构
                                    // 如果设置为true且不设置setUserFilterValue的值，则用nan填充点云
    // 执行滤波
    condrem.filter(*cloudOut); //大于0.0小于0.8这两个条件用于建立滤波器

    DWORD end_time = GetTickCount();
    std::cout << "滤波耗时: " << (end_time - start_time) << " ms" << std::endl;
}
/// <summary>
/// 显示折线图
/// </summary>
/// <param name="data_y"></param>	y轴数据
/// <param name="data_x"></param>	x轴数据
/// <param name="title"></param>	标题
/// <param name="titleX"></param>	X轴标题
/// <param name="titleY"></param>	Y轴标题
/// <param name="name"></param>		折线名称
void PCLTool::showPointPlot(vector<double> data_y, vector<double> data_x, string title, string titleX, string titleY, string name)
{
    if (data_x.empty())
    {
        for (int i = 0; i < data_y.size(); i++)
        {
            data_x.push_back(i);
        }
    }
    //显示
    pcl::visualization::PCLPlotter* plot(new pcl::visualization::PCLPlotter("Elevation and Point Number Breakdown Map"));
    plot->setShowLegend(true);
    plot->setBackgroundColor(1, 1, 1);
    plot->setTitle(title.c_str());
    plot->setXTitle(titleX.c_str());
    plot->setYTitle(titleY.c_str());
    plot->addPlotData(data_x, data_y, name.c_str());
    plot->plot();//绘制曲线
}
/// <summary>
/// PCL ICP算法
/// </summary>
/// <param name="cloud"></param>			点云数据
/// <param name="cloudOut"></param>			点云数据
/// <param name="cloudAligned"></param>		输出拼接点云
/// <param name="score"></param>			分数限制
/// <returns></returns>
bool PCLTool::cloudICP(PointCloudT::Ptr cloud, PointCloudT::Ptr cloudOut, PointCloudT::Ptr& cloudAligned, double score)
{
    pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
    icp.setInputSource(cloud);
    icp.setInputTarget(cloudOut);

    pcl::PointCloud<pcl::PointXYZ> Final;
    icp.align(Final);

    if (icp.getFitnessScore() < score)
    {
        return false;
    }

    pcl::transformPointCloud(*cloud, *cloudAligned,
        icp.getFinalTransformation());
    return true;
}
