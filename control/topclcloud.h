#ifndef TOPCLCLOUD_H
#define TOPCLCLOUD_H

#include <QObject>
#include <QThread>
#include "../model/cameractr.h"

class ToPCLCloud : public QThread
{
    Q_OBJECT
public:
    enum CLOUDTHREADTYPE
    {
        ONEAXIS = 0,
        MOREAXIS,
        KEEPDATA
    };
    static ToPCLCloud* getInstance();
    void StartThread(int model);
    void StartThread(int model, QString fileName, PointCloudT::Ptr cloud);
    void getCloud(int num);
    PointCloudT::Ptr getMCloud();
    int getThreadState() const;
    int getCloudListSize();

signals:
    void sendCloud(PointCloudT::Ptr cloud);

protected:
    void run() override;

private:
    static ToPCLCloud* myself;
    ToPCLCloud();
    cameraCtr* camera;
    PointCloudT::Ptr m_cloud;               //整幅点云图
    PointCloudT::Ptr exportCloud;           //导出点云
    QList<PointCloudT::Ptr> cloudList;     //点云数据集合
    int m_model;
    QString fileName;
    int threadState;
    int m_flage;
};

#endif // TOPCLCLOUD_H
