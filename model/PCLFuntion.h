#ifndef PCLFUNTION_H
#define PCLFUNTION_H

#include <iostream>
#include <pcl/io/pcd_io.h> 
#include <pcl/io/io.h>                      //I/O���ͷ�ļ�����
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

namespace PCLFuntion
{
	/// <summary>
	/// PCL�ļ�����
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
	/// ��������ϵ
	/// </summary>
	enum CLOUDP
	{
		CloudX = 0,
		CloudY,
		CloudZ
	};
	/// <summary>
	/// �㽺�������
	/// </summary>
	enum GlueType
	{
		GlueMulti = 0,
		GlueBreak,
		GlueLess
	};
	/// <summary>
	/// �ָ�string�ַ���
	/// </summary>
	/// <param name="str"></param>		Ҫ�ָ���ַ���
	/// <param name="split"></param>	�ָ��ַ�
	/// <param name="outStr"></param>	����ָ���ַ�
    void stringSplit(string str, const char split, vector<string>& outStr)
	{
		istringstream iss(str);	// ������
		string token;			// ���ջ�����
		while (getline(iss, token, split))	// ��splitΪ�ָ���
		{
			outStr.push_back(token);
		}
	}
	/// <summary>
	/// ���ص������ݣ�ͬʱ���е������
	/// </summary>
	/// <param name="fileNameA"></param>	����A
	/// <param name="fileNameB"></param>	����B
	/// <param name="cloud"></param>		�����������ƣ�B-A��
	/// <param name="model"></param>		�ļ����ط�ʽ
	void LoadCloud(string fileNameA, string fileNameB, PointCloudT::Ptr& cloud, int FileType, float err=1.0)
	{
		DWORD start_time = GetTickCount();
		switch (FileType)
		{
		case PCDASCII:
		case PCDBINARY:		//���������⣬��û��˳��
		{
			PointCloudT::Ptr cloudA(new PointCloudT);
			PointCloudT::Ptr cloudB(new PointCloudT);
			pcl::io::loadPCDFile(fileNameA, *cloudA);
            qDebug() << "Load CloudA Over";
			pcl::io::loadPCDFile(fileNameB, *cloudB);
            qDebug() << "Load CloudB Over" ;
			//���е������
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
			string lineA, lineB; //��������ÿһ��
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
				//��������������
				cloud->push_back(PointT(X_B, Y_B, (Z_B - Z_A)));
				num++;
                qDebug() << "Read Data Len: " << num;
			}
		}
		default:
			break;
		}
		DWORD end_time = GetTickCount();
		std::cout << "������ƺ�ʱ: " << (end_time - start_time) << " ms" << std::endl;
	}
	/// <summary>
	/// ���ļ����ص���
	/// </summary>
	/// <param name="path"></param>		�ļ�·��
	/// <param name="cloud"></param>		�������
	/// <param name="FileType"></param>		�ļ�����
	void LoadCloud(string path, PointCloudT::Ptr& cloud, int FileType)
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
			string line; //��������ÿһ��
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
        qDebug() << "������ƺ�ʱ: " << (end_time - start_time) << " ms" ;
	}
	/// <summary>
	/// �������  cloudOut = cloudA - cloudB
	/// </summary>
	/// <param name="cloudA"></param>  �������
	/// <param name="cloudB"></param>  �������
	/// <param name="cloudOut"></param> �������
	void cloudReduce(PointCloudT::Ptr cloudA, PointCloudT::Ptr cloudB, PointCloudT::Ptr& cloudOut)
	{
		int lenA = cloudA->size();
		int lenB = cloudB->size();
		//���Ʋ���
		if (lenA > lenB)
		{
			for (int i = 0; i < lenA - cloudB->size(); i++)
			{
				cloudB->push_back(PointT(0, 0, 0));
			}
		}
		//�������
		if (lenA < lenB)
		{
			cloudOut->width = lenB;
		}
		else
		{
			cloudOut->width = lenA;
		}
		cloudOut->height = 1;
		double x = 0, y = 0, z = 0;
		for (int i = 0; i < lenA; i++)
		{
			x = cloudA->points[i].x;
			y = cloudA->points[i].y;
			z = cloudA->points[i].z - cloudB->points[i].z;
			cloudOut->push_back(PointT(x, y, z));
		}
		//���B���ƱȽ϶�ֱ�ӽ�B�Ķ�ĵ��Ƹ������
		if (lenA < lenB)
		{
			for (int i = lenA; i < lenB; i++)
			{
				x = cloudB->points[i].x;
				y = cloudB->points[i].y;
				z = cloudB->points[i].z;
				cloudOut->push_back(PointT(x, y, z));
			}
		}
	}
	/// <summary>
	/// ��ʾ��������
	/// </summary>
	/// <param name="cloud"></param>	�������
	void showCloud(PointCloudT::Ptr cloud)
	{
		pcl::visualization::CloudViewer viewer("cloud viewer");
		viewer.showCloud(cloud);
		while (!viewer.wasStopped());
	}
	/// <summary>
	/// �����������
	/// </summary>
	/// <param name="cloud"></param>	�������
	/// <param name="name"></param>		�ļ�����
	/// <param name="type"></param>	�����ļ�����
	void saveCloud(PointCloudT::Ptr& cloud, string name, int type)
	{
		string path = "./data2/";
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
            qDebug() << "��ʱ��֧��" ;
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
	/// <summary>
	/// �Ե��ƽ��в��ء����ٶ�̫��������
	/// </summary>
	/// <param name="cloud"></param>		���ص���
	/// <param name="searchPoint"></param>	���ص�
	/// <returns></returns>
	bool checkDuplicate(PointCloudT::Ptr& cloud, PointT searchPoint)
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
	/// �Ե��ƽ��в��ء����ٶ�̫��������
	/// </summary>
	/// <param name="cloud"></param>  �������
	void checkDuplicate(PointCloudT::Ptr& cloud)
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
                qDebug() << "��ǰ����: " << i ;
                qDebug() << "����ظ�������: " << pointIdxRadiusSearch.size() ;
				for (int j = 1; j < pointIdxRadiusSearch.size(); j++)
				{
					PointCloudT::iterator it = cloud->begin() + pointIdxRadiusSearch[j];		//�ҵ��ظ��ĵ���
					cloud->erase(it);	//ɾ����Ӧ�ĵ���
					for (int k = j+1; k < pointIdxRadiusSearch.size(); k++)
					{
						pointIdxRadiusSearch[k] -= 1;
					}
				}
				kdtree.setInputCloud(cloud);
			}
		}
		DWORD checkEnd = GetTickCount64();
        qDebug() << "���غ�ʱ: " << checkEnd - checkStart << " ms" ;
	}
	/// <summary>
	///  �Ե�����ͶӰ
	/// </summary>
	/// <param name="cloudInput"></param>	�������
	/// <param name="cloudOut"></param>		�������
	/// <param name="direct"></param>		ͶӰ����
	void projectCloud(PointCloudT::Ptr cloudInput, PointCloudT::Ptr& cloudOut, int direct)
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
			//����ͶӰ
			if (direct == CloudX) //X����
			{
				x = 0;
			}
			else if (direct == CloudY)  //Y����
			{
				y = 0;
			}
			else if (direct == CloudZ)  //Z����
			{
				z = 0;
			}
			//PointT searchP = PointT(x, y, z);
			//��ʾ����
			count++;
			if (count >= ps)
			{
				count_t++;
				char outStr[50] = { 0 };
				sprintf_s(outStr, "Projection Schedule: %d/100", count_t);
                qDebug() << outStr ;
				count = 0;
			}
			//����ͶӰ����
			cloudOut->push_back(PointT(x, y, z));
		}
		DWORD end_time = GetTickCount64();
        qDebug() << "ͶӰ��ʱ: " << end_time - start_time << " ms" << endl;
	}
	/// <summary>
	/// ��ȡ��������
	/// </summary>
	/// <param name="cloud"></param>	�������
	/// <param name="outline"></param>	�������
	/// <param name="outY"></param>		���Y������----Y��ͶӰʧЧ
	/// <param name="direct"></param>	��������
	void outlineCloud(PointCloudT::Ptr cloud, vector<double>& outline, vector<double>* outY, int direct)
	{
		float row = 0;
		float min = 0, max = 0;
		float y = 0;
		if (direct == CloudX)  //ͶӰx�����������Ե
		{
			row = cloud->points[0].y;
			min = cloud->points[0].z;
			y = cloud->points[0].y;
		}
		else if (direct == CloudY)  //ͶӰy�����������Ե
		{
			row = cloud->points[0].x;
			min = cloud->points[0].z;
		}
		else if (direct == CloudZ) //ͶӰz�����������Ե
		{
			row = cloud->points[0].y;
			min = cloud->points[0].x;
			y = cloud->points[0].y;
		}

		int cloudSize = cloud->size();

		for (int i = 0; i < cloudSize; i++)
		{
			if (direct == CloudX)  //ͶӰx�����������Ե
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
			else if (direct == CloudY)  //ͶӰy�����������Ե
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
			else if (direct == CloudZ) //ͶӰz�����������Ե
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
	/// �Ե��ƽ���ǰ����
	/// </summary>
	/// <param name="outline"></param>	��������
	/// <param name="diff"></param>		��ֽ��
	void outlineDiff(vector<double> outline, vector<double>& diff)
	{
		int size = outline.size() - 1;
		for (int i = 0; i < size; i++)
		{
			diff.push_back((outline[i + 1] - outline[i]));
		}
	}
	/// <summary>
	/// �����ݲ��
	/// </summary>
	/// <param name="outline1"></param>		���������
	/// <param name="outline2"></param>		�������
	/// <param name="diff"></param>			��ֽ��
	void outlineDiff(vector<double> outline1, vector<double> outline2, vector<double>& diff)
	{
		int len = outline1.size();
		if (len > outline2.size())
		{
			len = outline2.size();
		}
		//���в�� outline2 - outline1
		for (int i = 0; i < len; i++)
		{
			diff.push_back((outline2[i] - outline1[i]));
		}
	}
	/// <summary>
	/// �㽺���
	/// </summary>
	/// <param name="diff"></param>			�������
	/// <param name="threshold"></param>	��ֵ
	/// <param name="numThrd"></param>		������ֵ
	/// <param name="type"></param>			�������
	/// <returns></returns>
	bool decideGlue(vector<double> diff, float threshold, int numThrd, int type)
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
	/// �Ե��ƽ�����Ⱥ���޳��˲�
	/// </summary>
	/// <param name="cloud"></param>			�������
	/// <param name="cloudOut"></param>			�������
	/// <param name="radius"></param>			�����ڽ�����
	/// <param name="radiusSearch"></param>		�뾶
	void cloudOutlierRemovalRadius(PointCloudT::Ptr cloud, PointCloudT::Ptr& cloudOut, int radius=20, float radiusSearch=0.8)
	{
		DWORD start_time = GetTickCount();
		
		pcl::RadiusOutlierRemoval<PointT> outrem; //�����˲���
		outrem.setInputCloud(cloud);       //�����������
		outrem.setRadiusSearch(radiusSearch);       //���ð뾶ΪradiusSearch�ķ�Χ�����ٽ���
		outrem.setMinNeighborsInRadius(radius); //���ò�ѯ�������㼯��С��radius��ɾ��
		// apply filter
		outrem.filter(*cloudOut); //ִ�������˲�   �ڰ뾶Ϊ0.8 �ڴ˰뾶�ڱ���Ҫ�������ھӵ㣬�˵�Żᱣ��

		DWORD end_time = GetTickCount();
        qDebug() << "�˲���ʱ: " << (end_time - start_time) << " ms";
	}
	/// <summary>
	/// ��Ⱥ���޳��˲�
	/// </summary>
	/// <param name="cloud"></param>		�������
	/// <param name="cloudOut"></param>		�������
	/// <param name="negative"></param>		��������������˲���������޳�����
	/// <param name="meanK"></param>		��ѯ���ڽ�����
	/// <param name="thresh"></param>		��Ⱥ����ֵ
	void cloudOutlierRemoval(PointCloudT::Ptr cloud, PointCloudT::Ptr& cloudOut,
							bool negative=false, int meanK=50, float thresh=0.5)
	{
		DWORD start_time = GetTickCount();

		/*�����˲�������ÿ����������ٽ���ĸ�������Ϊ50 ��������׼��ı�������Ϊ1  ����ζ�����һ
		����ľ��볬����ƽ������һ����׼�����ϣ���õ㱻���Ϊ��Ⱥ�㣬�������Ƴ����洢����*/
		pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;  //�����˲�������
		sor.setInputCloud(cloud);							//���ô��˲��ĵ���
		sor.setMeanK(meanK);								//�����ڽ���ͳ��ʱ���ǲ�ѯ���ٽ�����
		sor.setStddevMulThresh(thresh);						//�����ж��Ƿ�Ϊ��Ⱥ��ķ�ֵ
		sor.setNegative(negative);							//true���˲����ȡ���������˵ĵ�  false: ��ȡ���˺�ĵ�	
		sor.filter(*cloudOut);								//�洢

		DWORD end_time = GetTickCount();
        qDebug() << "�˲���ʱ: " << (end_time - start_time) << " ms";
	}
	/// <summary>
	/// �����޶��˲���
	/// </summary>
	/// <param name="cloud"></param>		�������
	/// <param name="cloudOut"></param>		�������
	/// <param name="axis"></param>			�˲�������
	/// <param name="min"></param>			����ֵ
	/// <param name="max"></param>			����ֵ
	void cloudCondition(PointCloudT::Ptr cloud, PointCloudT::Ptr& cloudOut, string axis="z", float min=0.0, float max=0.8)
	{
		DWORD start_time = GetTickCount();
		//���������޶����µ��˲���
		pcl::ConditionAnd<pcl::PointXYZ>::Ptr range_cond(new pcl::ConditionAnd<pcl::PointXYZ>()); //���������������
		//�����Z�ֶ��ϴ���0�ıȽ�����
		//GT greater than
		//EQ equal
		//LT less than
		//GE greater than or equal
		//LE less than
		//Ϊ�������������ӱȽ�����
		range_cond->addComparison(pcl::FieldComparison<pcl::PointXYZ>::ConstPtr(new pcl::FieldComparison<pcl::PointXYZ>(axis.c_str(), pcl::ComparisonOps::GT, min))); //�����Z�ֶ��ϴ���0�ıȽ�����
		range_cond->addComparison(pcl::FieldComparison<pcl::PointXYZ>::ConstPtr(new pcl::FieldComparison<pcl::PointXYZ>(axis.c_str(), pcl::ComparisonOps::LT, max))); //�����Z�ֶ���С��0.8�ıȽ�����
		// �����˲�������������������ʼ��
		pcl::ConditionalRemoval<pcl::PointXYZ> condrem;
		condrem.setCondition(range_cond);
		condrem.setInputCloud(cloud);   //�������
		//condrem.setKeepOrganized(false); //���ñ��ֵ��ƵĽṹ
										// �������Ϊtrue�Ҳ�����setUserFilterValue��ֵ������nan������
		// ִ���˲�
		condrem.filter(*cloudOut); //����0.0С��0.8�������������ڽ����˲���

		DWORD end_time = GetTickCount();
		std::cout << "�˲���ʱ: " << (end_time - start_time) << " ms" << std::endl;
	}
	/// <summary>
	/// ��ʾ����ͼ
	/// </summary>
	/// <param name="data_y"></param>	y������
	/// <param name="data_x"></param>	x������
	/// <param name="title"></param>	����
	/// <param name="titleX"></param>	X�����
	/// <param name="titleY"></param>	Y�����
	/// <param name="name"></param>		��������
	void showPointPlot(vector<double> data_y, vector<double> data_x = vector<double>(),
		string title="Display Data", string titleX="X", string titleY="Y", string name="Display Line")
	{
		if (data_x.empty())
		{
			for (int i = 0; i < data_y.size(); i++)
			{
				data_x.push_back(i);
			}
		}
		//��ʾ
		pcl::visualization::PCLPlotter* plot(new pcl::visualization::PCLPlotter("Elevation and Point Number Breakdown Map"));
		plot->setShowLegend(true);
		plot->setBackgroundColor(1, 1, 1);
		plot->setTitle(title.c_str());
		plot->setXTitle(titleX.c_str());
		plot->setYTitle(titleY.c_str());
		plot->addPlotData(data_x, data_y, name.c_str());
		plot->plot();//��������
	}
	/// <summary>
	/// PCL ICP�㷨
	/// </summary>
	/// <param name="cloud"></param>			��������
	/// <param name="cloudOut"></param>			��������
	/// <param name="cloudAligned"></param>		���ƴ�ӵ���
	/// <param name="score"></param>			��������
	/// <returns></returns>
	bool cloudICP(PointCloudT::Ptr cloud, PointCloudT::Ptr cloudOut, PointCloudT::Ptr& cloudAligned, double score=0.85)
	{
		pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
		icp.setInputSource(cloud);
		icp.setInputTarget(cloudOut);

		pcl::PointCloud<pcl::PointXYZ> Final;
		icp.align(Final);

		/*std::cout << "has converged:" << icp.hasConverged()
			<< " score: " << icp.getFitnessScore() << std::endl;
		std::cout << icp.getFinalTransformation() << std::endl;*/

		if (icp.getFitnessScore() < score)
		{
			return false;
		}

		pcl::transformPointCloud(*cloud, *cloudAligned,
			icp.getFinalTransformation());
		return true;
	}
}

namespace FileFuntion
{
	/// <summary>
	/// �ļ�����
	/// </summary>
	enum FileType
	{
		TXT = 0,
		BIN
	};
	/// <summary>
	/// �����ļ�
	/// </summary>
	/// <param name="data"></param>		���������
	/// <param name="name"></param>		��������
	/// <param name="type"></param>		��������
	void saveFile(vector<double> data, string name, int type)
	{
		string path = "./data2/";
		switch (type)
		{
		case TXT:
		{
			path += (name + ".txt");
			std::fstream file;
			file.open(path, std::ios::out);
			int size = data.size();
			float ps = size / 100;
			int count = 0, count_t = 0;
			for (int i = 0; i < size; i++)
			{
				char str[20] = { 0 };
				sprintf_s(str, "%f\n", data[i]);
				file.write(str, strlen(str));
				count++;
				if (count >= ps)
				{
					count_t++;
					char outStr[50] = { 0 };
					sprintf_s(outStr, "Export Schedule: %d/100", count_t);
                    qDebug() << outStr ;
					count = 0;
				}
			}
			break;
		}
		case BIN:
		{
			path += (name + ".bin");
			std::fstream file;
			file.open(path, std::ios::out | std::ios::binary);
			int size = data.size();
			float ps = size / 100;
			int count = 0, count_t = 0;
			for (int i = 0; i < size; i++)
			{
				char str[20] = { 0 };
				sprintf_s(str, "%f\n", data[i]);
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
		}
	}
	/// <summary>
	/// �����ļ�
	/// </summary>
	/// <param name="path"></param>		�ļ�·��
	/// <param name="data"></param>		�������
	/// <param name="type"></param>		�ļ�����
	void loadFile(string path, vector<double>& data, int type)
	{
		if (type == TXT)
		{
			std::fstream file;
			file.open(path, std::ios::in);
			string line;
			while (getline(file, line))
			{
				float d = atof(line.c_str());
				data.push_back(d);
			}
		}
		else if(type == BIN)
		{
			std::fstream file;
			file.open(path, std::ios::in | std::ios::binary);
			string line;
			while (getline(file, line))
			{
				float d = atof(line.c_str());
				data.push_back(d);
			}
		}
        qDebug() << "Load File Over!";
	}
}
#endif // PCLFUNTION_H
