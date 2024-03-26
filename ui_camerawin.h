/********************************************************************************
** Form generated from reading UI file 'camerawin.ui'
**
** Created by: Qt User Interface Compiler version 5.14.2
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_CAMERAWIN_H
#define UI_CAMERAWIN_H

#include <QtCore/QVariant>
#include <QtWidgets/QApplication>
#include <QtWidgets/QLabel>
#include <QtWidgets/QPushButton>
#include <QtWidgets/QWidget>

QT_BEGIN_NAMESPACE

class Ui_camerawin
{
public:
    QPushButton *openBtn;
    QPushButton *setBtn;
    QPushButton *saveBtn;
    QLabel *showLab;

    void setupUi(QWidget *camerawin)
    {
        if (camerawin->objectName().isEmpty())
            camerawin->setObjectName(QString::fromUtf8("camerawin"));
        camerawin->resize(788, 575);
        openBtn = new QPushButton(camerawin);
        openBtn->setObjectName(QString::fromUtf8("openBtn"));
        openBtn->setGeometry(QRect(140, 30, 93, 28));
        setBtn = new QPushButton(camerawin);
        setBtn->setObjectName(QString::fromUtf8("setBtn"));
        setBtn->setGeometry(QRect(300, 30, 93, 28));
        saveBtn = new QPushButton(camerawin);
        saveBtn->setObjectName(QString::fromUtf8("saveBtn"));
        saveBtn->setGeometry(QRect(460, 30, 93, 28));
        showLab = new QLabel(camerawin);
        showLab->setObjectName(QString::fromUtf8("showLab"));
        showLab->setGeometry(QRect(40, 80, 581, 461));
        showLab->setFrameShape(QFrame::Box);
        showLab->setFrameShadow(QFrame::Raised);

        retranslateUi(camerawin);

        QMetaObject::connectSlotsByName(camerawin);
    } // setupUi

    void retranslateUi(QWidget *camerawin)
    {
        camerawin->setWindowTitle(QCoreApplication::translate("camerawin", "\347\233\270\346\234\272\347\225\214\351\235\242", nullptr));
        openBtn->setText(QCoreApplication::translate("camerawin", "\346\211\223\345\274\200\347\233\270\346\234\272", nullptr));
        setBtn->setText(QCoreApplication::translate("camerawin", "\345\261\236\346\200\247\350\256\276\347\275\256", nullptr));
        saveBtn->setText(QCoreApplication::translate("camerawin", "\346\213\215\347\205\247\344\277\235\345\255\230", nullptr));
        showLab->setText(QString());
    } // retranslateUi

};

namespace Ui {
    class camerawin: public Ui_camerawin {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_CAMERAWIN_H
