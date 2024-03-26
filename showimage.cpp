#include "showimage.h"
#include "ui_showimage.h"
#include<QFileDialog>
#include<QDebug>
#include<QFileInfo>
#include<QFile>

showImage::showImage(QWidget *parent) :
    QWidget(parent),
    ui(new Ui::showImage)
{
    ui->setupUi(this);

    this->setStyleSheet("#showImage{background-color: qlineargradient(spread:pad, x1:0, y1:0, x2:1, y2:0, stop:0 rgba(94, 125, 179, 200),"
                        " stop:1 rgba(183, 202, 216, 200));}");

}

showImage::~showImage()
{
    delete ui;
}

void showImage::imgInitialization()
{
    this->ui->imgLabel->setPixmap(QPixmap("./picture/0.bmp"));
    ui->imgLabel->setScaledContents(true);//设置自适应
}

void showImage::on_preBtn_clicked()
{
        index_img--;
        if(filenameList.isEmpty()) return;
        //显示上一张图片
        if(index_img < 0){
            index_img = imgLength - 1;
        }
        QString filename = filenameList.at(index_img);
        pixMap.load(filename);
        ui->imgLabel->setPixmap(pixMap);

}

void showImage::on_openFile_clicked()
{

    filenameList =  QFileDialog::getOpenFileNames(this,"打开多张图片",
                                  "D:/0314/qt/trap/ProgrammingLog/Microscope_1_10/program/picture","Images (*.png *.bmp *.jpg *.png);;");

    imgLength = filenameList.length();
    QString filename = filenameList.at(index_img);
    pixMap.load(filename);
    ui->imgLabel->setPixmap(pixMap);
    //ui->path->setText(filename);
    qDebug()<<"length: "<<imgLength;

}

void showImage::on_nextBtn_clicked()
{
       //显示下一张图片
       if(filenameList.isEmpty()) return;
       if(index_img > imgLength-1){
           index_img = 0;
       }
       QString filename = filenameList.at(index_img);
       pixMap.load(filename);
       ui->imgLabel->setPixmap(pixMap);
       index_img++;

}
