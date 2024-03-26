/********************************************************************************
** Form generated from reading UI file 'showimage.ui'
**
** Created by: Qt User Interface Compiler version 5.14.2
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_SHOWIMAGE_H
#define UI_SHOWIMAGE_H

#include <QtCore/QVariant>
#include <QtGui/QIcon>
#include <QtWidgets/QApplication>
#include <QtWidgets/QGridLayout>
#include <QtWidgets/QLabel>
#include <QtWidgets/QPushButton>
#include <QtWidgets/QWidget>

QT_BEGIN_NAMESPACE

class Ui_showImage
{
public:
    QGridLayout *gridLayout;
    QPushButton *preBtn;
    QPushButton *openFile;
    QPushButton *nextBtn;
    QLabel *imgLabel;

    void setupUi(QWidget *showImage)
    {
        if (showImage->objectName().isEmpty())
            showImage->setObjectName(QString::fromUtf8("showImage"));
        showImage->resize(1019, 704);
        QFont font;
        font.setFamily(QString::fromUtf8("\345\256\213\344\275\223"));
        font.setPointSize(12);
        showImage->setFont(font);
        QIcon icon;
        icon.addFile(QString::fromUtf8(":/new/prefix1/icon/xianwei_1.webp"), QSize(), QIcon::Normal, QIcon::Off);
        showImage->setWindowIcon(icon);
        gridLayout = new QGridLayout(showImage);
        gridLayout->setObjectName(QString::fromUtf8("gridLayout"));
        preBtn = new QPushButton(showImage);
        preBtn->setObjectName(QString::fromUtf8("preBtn"));
        preBtn->setMinimumSize(QSize(0, 40));
        preBtn->setStyleSheet(QString::fromUtf8("background-color: qlineargradient(spread:pad, x1:0, y1:0, x2:1, y2:1, stop:0 rgba(109, 158, 235, 255), stop:1 rgba(255, 255, 255, 255));"));

        gridLayout->addWidget(preBtn, 1, 0, 1, 1);

        openFile = new QPushButton(showImage);
        openFile->setObjectName(QString::fromUtf8("openFile"));
        openFile->setMinimumSize(QSize(0, 40));
        openFile->setStyleSheet(QString::fromUtf8("background-color: qlineargradient(spread:pad, x1:0, y1:0, x2:1, y2:1, stop:0 rgba(109, 158, 235, 255), stop:1 rgba(255, 255, 255, 255));"));

        gridLayout->addWidget(openFile, 1, 1, 1, 1);

        nextBtn = new QPushButton(showImage);
        nextBtn->setObjectName(QString::fromUtf8("nextBtn"));
        nextBtn->setMinimumSize(QSize(0, 40));
        nextBtn->setStyleSheet(QString::fromUtf8("background-color: qlineargradient(spread:pad, x1:0, y1:0, x2:1, y2:1, stop:0 rgba(109, 158, 235, 255), stop:1 rgba(255, 255, 255, 255));"));

        gridLayout->addWidget(nextBtn, 1, 2, 1, 1);

        imgLabel = new QLabel(showImage);
        imgLabel->setObjectName(QString::fromUtf8("imgLabel"));
        imgLabel->setFrameShape(QFrame::Panel);

        gridLayout->addWidget(imgLabel, 0, 0, 1, 3);


        retranslateUi(showImage);

        QMetaObject::connectSlotsByName(showImage);
    } // setupUi

    void retranslateUi(QWidget *showImage)
    {
        showImage->setWindowTitle(QCoreApplication::translate("showImage", "\345\233\276\347\211\207\351\242\204\350\247\210", nullptr));
        preBtn->setText(QCoreApplication::translate("showImage", "\344\270\212\344\270\200\345\274\240", nullptr));
        openFile->setText(QCoreApplication::translate("showImage", "\346\211\223\345\274\200\346\226\207\344\273\266", nullptr));
#if QT_CONFIG(tooltip)
        nextBtn->setToolTip(QString());
#endif // QT_CONFIG(tooltip)
        nextBtn->setText(QCoreApplication::translate("showImage", "\344\270\213\344\270\200\345\274\240", nullptr));
        imgLabel->setText(QString());
    } // retranslateUi

};

namespace Ui {
    class showImage: public Ui_showImage {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_SHOWIMAGE_H
