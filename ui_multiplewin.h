/********************************************************************************
** Form generated from reading UI file 'multiplewin.ui'
**
** Created by: Qt User Interface Compiler version 5.14.2
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_MULTIPLEWIN_H
#define UI_MULTIPLEWIN_H

#include <QtCore/QVariant>
#include <QtWidgets/QApplication>
#include <QtWidgets/QLabel>
#include <QtWidgets/QLineEdit>
#include <QtWidgets/QPushButton>
#include <QtWidgets/QRadioButton>
#include <QtWidgets/QWidget>

QT_BEGIN_NAMESPACE

class Ui_multiplewin
{
public:
    QLabel *label_2;
    QLabel *label_3;
    QPushButton *frontBut;
    QLabel *label_7;
    QPushButton *stopBut;
    QLineEdit *speedEdit;
    QPushButton *disconnectBut;
    QRadioButton *continueBut;
    QLabel *label;
    QPushButton *connectBut;
    QLabel *label_4;
    QLineEdit *srampEdit;
    QLabel *label_5;
    QPushButton *setOrigin;
    QLineEdit *distanceEdit;
    QLineEdit *IPEdit;
    QLineEdit *accSpeedEdit;
    QLabel *positionLab;
    QLineEdit *lSpeedEdit;
    QLabel *label_6;
    QPushButton *pressreversalBut;
    QLabel *label_8;
    QPushButton *pressfrontBut;
    QPushButton *reversalBut;
    QRadioButton *motionBut;
    QLineEdit *pulseEdit;
    QLineEdit *decSpeedEdit;
    QLabel *camLab;
    QPushButton *openBtn;

    void setupUi(QWidget *multiplewin)
    {
        if (multiplewin->objectName().isEmpty())
            multiplewin->setObjectName(QString::fromUtf8("multiplewin"));
        multiplewin->resize(1148, 620);
        label_2 = new QLabel(multiplewin);
        label_2->setObjectName(QString::fromUtf8("label_2"));
        label_2->setGeometry(QRect(80, 210, 81, 41));
        label_3 = new QLabel(multiplewin);
        label_3->setObjectName(QString::fromUtf8("label_3"));
        label_3->setGeometry(QRect(80, 260, 81, 41));
        frontBut = new QPushButton(multiplewin);
        frontBut->setObjectName(QString::fromUtf8("frontBut"));
        frontBut->setGeometry(QRect(450, 400, 93, 28));
        label_7 = new QLabel(multiplewin);
        label_7->setObjectName(QString::fromUtf8("label_7"));
        label_7->setGeometry(QRect(70, 460, 91, 41));
        stopBut = new QPushButton(multiplewin);
        stopBut->setObjectName(QString::fromUtf8("stopBut"));
        stopBut->setGeometry(QRect(670, 440, 101, 51));
        speedEdit = new QLineEdit(multiplewin);
        speedEdit->setObjectName(QString::fromUtf8("speedEdit"));
        speedEdit->setGeometry(QRect(160, 260, 181, 41));
        disconnectBut = new QPushButton(multiplewin);
        disconnectBut->setObjectName(QString::fromUtf8("disconnectBut"));
        disconnectBut->setGeometry(QRect(520, 70, 93, 28));
        continueBut = new QRadioButton(multiplewin);
        continueBut->setObjectName(QString::fromUtf8("continueBut"));
        continueBut->setGeometry(QRect(560, 160, 115, 19));
        label = new QLabel(multiplewin);
        label->setObjectName(QString::fromUtf8("label"));
        label->setGeometry(QRect(100, 70, 51, 41));
        connectBut = new QPushButton(multiplewin);
        connectBut->setObjectName(QString::fromUtf8("connectBut"));
        connectBut->setGeometry(QRect(400, 70, 93, 28));
        label_4 = new QLabel(multiplewin);
        label_4->setObjectName(QString::fromUtf8("label_4"));
        label_4->setGeometry(QRect(470, 240, 91, 51));
        srampEdit = new QLineEdit(multiplewin);
        srampEdit->setObjectName(QString::fromUtf8("srampEdit"));
        srampEdit->setGeometry(QRect(160, 470, 181, 41));
        label_5 = new QLabel(multiplewin);
        label_5->setObjectName(QString::fromUtf8("label_5"));
        label_5->setGeometry(QRect(80, 310, 71, 41));
        setOrigin = new QPushButton(multiplewin);
        setOrigin->setObjectName(QString::fromUtf8("setOrigin"));
        setOrigin->setGeometry(QRect(660, 390, 93, 28));
        distanceEdit = new QLineEdit(multiplewin);
        distanceEdit->setObjectName(QString::fromUtf8("distanceEdit"));
        distanceEdit->setGeometry(QRect(550, 240, 181, 51));
        IPEdit = new QLineEdit(multiplewin);
        IPEdit->setObjectName(QString::fromUtf8("IPEdit"));
        IPEdit->setGeometry(QRect(170, 70, 181, 41));
        accSpeedEdit = new QLineEdit(multiplewin);
        accSpeedEdit->setObjectName(QString::fromUtf8("accSpeedEdit"));
        accSpeedEdit->setGeometry(QRect(160, 360, 181, 41));
        positionLab = new QLabel(multiplewin);
        positionLab->setObjectName(QString::fromUtf8("positionLab"));
        positionLab->setGeometry(QRect(540, 330, 161, 31));
        lSpeedEdit = new QLineEdit(multiplewin);
        lSpeedEdit->setObjectName(QString::fromUtf8("lSpeedEdit"));
        lSpeedEdit->setGeometry(QRect(160, 310, 181, 41));
        label_6 = new QLabel(multiplewin);
        label_6->setObjectName(QString::fromUtf8("label_6"));
        label_6->setGeometry(QRect(90, 420, 81, 31));
        pressreversalBut = new QPushButton(multiplewin);
        pressreversalBut->setObjectName(QString::fromUtf8("pressreversalBut"));
        pressreversalBut->setGeometry(QRect(560, 450, 93, 28));
        pressreversalBut->setAutoRepeat(false);
        label_8 = new QLabel(multiplewin);
        label_8->setObjectName(QString::fromUtf8("label_8"));
        label_8->setGeometry(QRect(90, 370, 81, 31));
        pressfrontBut = new QPushButton(multiplewin);
        pressfrontBut->setObjectName(QString::fromUtf8("pressfrontBut"));
        pressfrontBut->setGeometry(QRect(450, 450, 93, 28));
        pressfrontBut->setAutoRepeat(false);
        reversalBut = new QPushButton(multiplewin);
        reversalBut->setObjectName(QString::fromUtf8("reversalBut"));
        reversalBut->setGeometry(QRect(560, 400, 93, 28));
        motionBut = new QRadioButton(multiplewin);
        motionBut->setObjectName(QString::fromUtf8("motionBut"));
        motionBut->setGeometry(QRect(560, 190, 115, 19));
        pulseEdit = new QLineEdit(multiplewin);
        pulseEdit->setObjectName(QString::fromUtf8("pulseEdit"));
        pulseEdit->setGeometry(QRect(160, 210, 181, 41));
        decSpeedEdit = new QLineEdit(multiplewin);
        decSpeedEdit->setObjectName(QString::fromUtf8("decSpeedEdit"));
        decSpeedEdit->setGeometry(QRect(160, 420, 181, 41));
        camLab = new QLabel(multiplewin);
        camLab->setObjectName(QString::fromUtf8("camLab"));
        camLab->setGeometry(QRect(780, 200, 341, 311));
        camLab->setFrameShape(QFrame::Box);
        camLab->setFrameShadow(QFrame::Raised);
        openBtn = new QPushButton(multiplewin);
        openBtn->setObjectName(QString::fromUtf8("openBtn"));
        openBtn->setGeometry(QRect(910, 130, 93, 28));

        retranslateUi(multiplewin);

        QMetaObject::connectSlotsByName(multiplewin);
    } // setupUi

    void retranslateUi(QWidget *multiplewin)
    {
        multiplewin->setWindowTitle(QCoreApplication::translate("multiplewin", "\345\217\230\345\200\215\347\225\214\351\235\242", nullptr));
        label_2->setText(QCoreApplication::translate("multiplewin", "\350\204\211\345\206\262\345\275\223\351\207\217:", nullptr));
        label_3->setText(QCoreApplication::translate("multiplewin", "\347\224\265\346\234\272\351\200\237\345\272\246:", nullptr));
        frontBut->setText(QCoreApplication::translate("multiplewin", "\347\224\265\346\234\272\346\255\243\350\275\254", nullptr));
        label_7->setText(QCoreApplication::translate("multiplewin", "S\346\233\262\347\272\277\346\227\266\351\227\264:", nullptr));
        stopBut->setText(QCoreApplication::translate("multiplewin", "\345\201\234\346\255\242\350\277\220\345\212\250", nullptr));
        disconnectBut->setText(QCoreApplication::translate("multiplewin", "\346\226\255\345\274\200\350\277\236\346\216\245", nullptr));
        continueBut->setText(QCoreApplication::translate("multiplewin", "\346\214\201\347\273\255\350\277\220\345\212\250", nullptr));
        label->setText(QCoreApplication::translate("multiplewin", "\350\277\236\346\216\245IP:", nullptr));
        connectBut->setText(QCoreApplication::translate("multiplewin", "\350\277\236\346\216\245", nullptr));
        label_4->setText(QCoreApplication::translate("multiplewin", "\347\247\273\345\212\250\350\267\235\347\246\273:", nullptr));
        label_5->setText(QCoreApplication::translate("multiplewin", "\350\265\267\345\247\213\351\200\237\345\272\246:", nullptr));
        setOrigin->setText(QCoreApplication::translate("multiplewin", "\350\256\276\347\275\256\345\216\237\347\202\271", nullptr));
        positionLab->setText(QCoreApplication::translate("multiplewin", "\350\275\264\344\275\215\347\275\256:", nullptr));
        label_6->setText(QCoreApplication::translate("multiplewin", "\345\207\217\351\200\237\345\272\246:", nullptr));
        pressreversalBut->setText(QCoreApplication::translate("multiplewin", "\351\225\277\346\214\211\345\217\215\350\275\254", nullptr));
        label_8->setText(QCoreApplication::translate("multiplewin", "\345\212\240\351\200\237\345\272\246:", nullptr));
        pressfrontBut->setText(QCoreApplication::translate("multiplewin", "\351\225\277\346\214\211\346\255\243\350\275\254", nullptr));
        reversalBut->setText(QCoreApplication::translate("multiplewin", "\347\224\265\346\234\272\345\217\215\350\275\254", nullptr));
        motionBut->setText(QCoreApplication::translate("multiplewin", "\345\257\270\345\212\250", nullptr));
        camLab->setText(QString());
        openBtn->setText(QCoreApplication::translate("multiplewin", "\346\211\223\345\274\200\347\233\270\346\234\272", nullptr));
    } // retranslateUi

};

namespace Ui {
    class multiplewin: public Ui_multiplewin {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_MULTIPLEWIN_H
