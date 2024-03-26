/********************************************************************************
** Form generated from reading UI file 'yoloc.ui'
**
** Created by: Qt User Interface Compiler version 5.14.2
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_YOLOC_H
#define UI_YOLOC_H

#include <QtCore/QVariant>
#include <QtGui/QIcon>
#include <QtWidgets/QApplication>
#include <QtWidgets/QComboBox>
#include <QtWidgets/QGridLayout>
#include <QtWidgets/QHBoxLayout>
#include <QtWidgets/QLabel>
#include <QtWidgets/QPushButton>
#include <QtWidgets/QTextEdit>
#include <QtWidgets/QWidget>

QT_BEGIN_NAMESPACE

class Ui_yoloC
{
public:
    QGridLayout *gridLayout_3;
    QWidget *widget_2;
    QGridLayout *gridLayout;
    QWidget *widget;
    QHBoxLayout *horizontalLayout;
    QPushButton *openfile;
    QPushButton *loadfile;
    QComboBox *comboBox;
    QPushButton *startdetect;
    QPushButton *stopdetect;
    QLabel *label;
    QTextEdit *textEditlog;

    void setupUi(QWidget *yoloC)
    {
        if (yoloC->objectName().isEmpty())
            yoloC->setObjectName(QString::fromUtf8("yoloC"));
        yoloC->resize(1019, 726);
        QFont font;
        font.setFamily(QString::fromUtf8("\345\256\213\344\275\223"));
        font.setPointSize(12);
        yoloC->setFont(font);
        QIcon icon;
        icon.addFile(QString::fromUtf8(":/new/prefix1/icon/xianwei_1.webp"), QSize(), QIcon::Normal, QIcon::Off);
        yoloC->setWindowIcon(icon);
        gridLayout_3 = new QGridLayout(yoloC);
        gridLayout_3->setObjectName(QString::fromUtf8("gridLayout_3"));
        widget_2 = new QWidget(yoloC);
        widget_2->setObjectName(QString::fromUtf8("widget_2"));
        gridLayout = new QGridLayout(widget_2);
        gridLayout->setSpacing(0);
        gridLayout->setObjectName(QString::fromUtf8("gridLayout"));
        gridLayout->setContentsMargins(0, 0, 0, 0);
        widget = new QWidget(widget_2);
        widget->setObjectName(QString::fromUtf8("widget"));
        widget->setMinimumSize(QSize(0, 60));
        widget->setMaximumSize(QSize(16777215, 80));
        horizontalLayout = new QHBoxLayout(widget);
        horizontalLayout->setObjectName(QString::fromUtf8("horizontalLayout"));
        openfile = new QPushButton(widget);
        openfile->setObjectName(QString::fromUtf8("openfile"));
        QSizePolicy sizePolicy(QSizePolicy::Minimum, QSizePolicy::Fixed);
        sizePolicy.setHorizontalStretch(3);
        sizePolicy.setVerticalStretch(0);
        sizePolicy.setHeightForWidth(openfile->sizePolicy().hasHeightForWidth());
        openfile->setSizePolicy(sizePolicy);
        openfile->setMinimumSize(QSize(0, 40));
        openfile->setContextMenuPolicy(Qt::NoContextMenu);
        openfile->setStyleSheet(QString::fromUtf8("font: 12pt \"\345\256\213\344\275\223\";"));
        QIcon icon1;
        icon1.addFile(QString::fromUtf8(":/new/prefix1/icon/15.png"), QSize(), QIcon::Normal, QIcon::Off);
        openfile->setIcon(icon1);
        openfile->setIconSize(QSize(35, 35));

        horizontalLayout->addWidget(openfile);

        loadfile = new QPushButton(widget);
        loadfile->setObjectName(QString::fromUtf8("loadfile"));
        sizePolicy.setHeightForWidth(loadfile->sizePolicy().hasHeightForWidth());
        loadfile->setSizePolicy(sizePolicy);
        loadfile->setMinimumSize(QSize(0, 40));
        loadfile->setStyleSheet(QString::fromUtf8("font: 12pt \"\345\256\213\344\275\223\";"));
        QIcon icon2;
        icon2.addFile(QString::fromUtf8(":/new/prefix1/icon/18.png"), QSize(), QIcon::Normal, QIcon::Off);
        loadfile->setIcon(icon2);
        loadfile->setIconSize(QSize(35, 35));

        horizontalLayout->addWidget(loadfile);

        comboBox = new QComboBox(widget);
        comboBox->addItem(QString());
        comboBox->addItem(QString());
        comboBox->addItem(QString());
        comboBox->addItem(QString());
        comboBox->addItem(QString());
        comboBox->setObjectName(QString::fromUtf8("comboBox"));
        QSizePolicy sizePolicy1(QSizePolicy::Preferred, QSizePolicy::Fixed);
        sizePolicy1.setHorizontalStretch(3);
        sizePolicy1.setVerticalStretch(0);
        sizePolicy1.setHeightForWidth(comboBox->sizePolicy().hasHeightForWidth());
        comboBox->setSizePolicy(sizePolicy1);
        comboBox->setMinimumSize(QSize(0, 40));
        comboBox->setStyleSheet(QString::fromUtf8("font: 12pt \"\345\256\213\344\275\223\";"));

        horizontalLayout->addWidget(comboBox);

        startdetect = new QPushButton(widget);
        startdetect->setObjectName(QString::fromUtf8("startdetect"));
        sizePolicy.setHeightForWidth(startdetect->sizePolicy().hasHeightForWidth());
        startdetect->setSizePolicy(sizePolicy);
        startdetect->setMinimumSize(QSize(0, 40));
        startdetect->setStyleSheet(QString::fromUtf8("font: 12pt \"\345\256\213\344\275\223\";"));
        QIcon icon3;
        icon3.addFile(QString::fromUtf8(":/new/prefix1/icon/21.png"), QSize(), QIcon::Normal, QIcon::Off);
        startdetect->setIcon(icon3);
        startdetect->setIconSize(QSize(35, 35));

        horizontalLayout->addWidget(startdetect);

        stopdetect = new QPushButton(widget);
        stopdetect->setObjectName(QString::fromUtf8("stopdetect"));
        sizePolicy.setHeightForWidth(stopdetect->sizePolicy().hasHeightForWidth());
        stopdetect->setSizePolicy(sizePolicy);
        stopdetect->setMinimumSize(QSize(0, 40));
        stopdetect->setStyleSheet(QString::fromUtf8("font: 12pt \"\345\256\213\344\275\223\";"));
        QIcon icon4;
        icon4.addFile(QString::fromUtf8(":/new/prefix1/icon/all5.png"), QSize(), QIcon::Normal, QIcon::Off);
        stopdetect->setIcon(icon4);
        stopdetect->setIconSize(QSize(35, 35));

        horizontalLayout->addWidget(stopdetect);


        gridLayout->addWidget(widget, 0, 0, 1, 1);

        label = new QLabel(widget_2);
        label->setObjectName(QString::fromUtf8("label"));
        label->setMinimumSize(QSize(500, 300));
        label->setFrameShape(QFrame::Panel);

        gridLayout->addWidget(label, 1, 0, 1, 1);


        gridLayout_3->addWidget(widget_2, 0, 0, 1, 1);

        textEditlog = new QTextEdit(yoloC);
        textEditlog->setObjectName(QString::fromUtf8("textEditlog"));
        textEditlog->setMinimumSize(QSize(250, 0));
        textEditlog->setMaximumSize(QSize(250, 16777215));

        gridLayout_3->addWidget(textEditlog, 0, 1, 1, 1);


        retranslateUi(yoloC);

        QMetaObject::connectSlotsByName(yoloC);
    } // setupUi

    void retranslateUi(QWidget *yoloC)
    {
        yoloC->setWindowTitle(QCoreApplication::translate("yoloC", "yolov5\345\256\236\346\227\266\346\243\200\346\265\213", nullptr));
#if QT_CONFIG(tooltip)
        openfile->setToolTip(QCoreApplication::translate("yoloC", "\346\211\223\345\274\200\346\226\207\344\273\266", nullptr));
#endif // QT_CONFIG(tooltip)
        openfile->setText(QCoreApplication::translate("yoloC", "\346\211\223\345\274\200\346\226\207\344\273\266", nullptr));
#if QT_CONFIG(tooltip)
        loadfile->setToolTip(QCoreApplication::translate("yoloC", "\345\212\240\350\275\275\346\226\207\344\273\266", nullptr));
#endif // QT_CONFIG(tooltip)
        loadfile->setText(QCoreApplication::translate("yoloC", "\345\212\240\350\275\275\346\250\241\345\236\213", nullptr));
        comboBox->setItemText(0, QCoreApplication::translate("yoloC", "yolov5s", nullptr));
        comboBox->setItemText(1, QCoreApplication::translate("yoloC", "yolov5m", nullptr));
        comboBox->setItemText(2, QCoreApplication::translate("yoloC", "yolov5n", nullptr));
        comboBox->setItemText(3, QCoreApplication::translate("yoloC", "yolov5l", nullptr));
        comboBox->setItemText(4, QCoreApplication::translate("yoloC", "yolov5x", nullptr));

#if QT_CONFIG(tooltip)
        comboBox->setToolTip(QCoreApplication::translate("yoloC", "\351\200\211\346\213\251\346\250\241\345\236\213", nullptr));
#endif // QT_CONFIG(tooltip)
#if QT_CONFIG(tooltip)
        startdetect->setToolTip(QCoreApplication::translate("yoloC", "\345\274\200\345\247\213\346\243\200\346\265\213", nullptr));
#endif // QT_CONFIG(tooltip)
        startdetect->setText(QCoreApplication::translate("yoloC", "yolov5\346\243\200\346\265\213", nullptr));
#if QT_CONFIG(tooltip)
        stopdetect->setToolTip(QCoreApplication::translate("yoloC", "\345\201\234\346\255\242\346\243\200\346\265\213", nullptr));
#endif // QT_CONFIG(tooltip)
        stopdetect->setText(QCoreApplication::translate("yoloC", "\345\201\234\346\255\242\346\243\200\346\265\213", nullptr));
#if QT_CONFIG(tooltip)
        label->setToolTip(QCoreApplication::translate("yoloC", "\350\277\220\350\241\214\347\225\214\351\235\242", nullptr));
#endif // QT_CONFIG(tooltip)
        label->setText(QString());
#if QT_CONFIG(tooltip)
        textEditlog->setToolTip(QCoreApplication::translate("yoloC", "\347\273\223\346\236\234\346\230\276\347\244\272", nullptr));
#endif // QT_CONFIG(tooltip)
    } // retranslateUi

};

namespace Ui {
    class yoloC: public Ui_yoloC {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_YOLOC_H
