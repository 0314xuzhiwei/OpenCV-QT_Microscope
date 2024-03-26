#ifndef SHOWIMAGE_H
#define SHOWIMAGE_H

#include <QWidget>

namespace Ui {
class showImage;
}

class showImage : public QWidget
{
    Q_OBJECT

public:
    explicit showImage(QWidget *parent = nullptr);
    ~showImage();
    void imgInitialization();

private slots:
    void on_preBtn_clicked();

    void on_openFile_clicked();

    void on_nextBtn_clicked();

private:
    Ui::showImage *ui;

        QPixmap pixMap;     //图片对象
        QStringList filenameList;   //图片列表
        int imgLength;  //图片张数
        int index_img;  //当前图片下标
};

#endif // SHOWIMAGE_H
