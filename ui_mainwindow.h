/********************************************************************************
** Form generated from reading UI file 'mainwindow.ui'
**
** Created by: Qt User Interface Compiler version 5.14.2
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_MAINWINDOW_H
#define UI_MAINWINDOW_H

#include <QtCore/QVariant>
#include <QtGui/QIcon>
#include <QtWidgets/QApplication>
#include <QtWidgets/QComboBox>
#include <QtWidgets/QGridLayout>
#include <QtWidgets/QLabel>
#include <QtWidgets/QLineEdit>
#include <QtWidgets/QMainWindow>
#include <QtWidgets/QPushButton>
#include <QtWidgets/QRadioButton>
#include <QtWidgets/QScrollArea>
#include <QtWidgets/QSpacerItem>
#include <QtWidgets/QStackedWidget>
#include <QtWidgets/QTextEdit>
#include <QtWidgets/QToolBox>
#include <QtWidgets/QVBoxLayout>
#include <QtWidgets/QWidget>

QT_BEGIN_NAMESPACE

class Ui_MainWindow
{
public:
    QWidget *centralwidget;
    QGridLayout *gridLayout_45;
    QWidget *widget_2;
    QGridLayout *gridLayout_2;
    QWidget *widget;
    QGridLayout *gridLayout;
    QSpacerItem *verticalSpacer;
    QPushButton *breakIP;
    QWidget *widget_6;
    QGridLayout *gridLayout_6;
    QPushButton *openBtn;
    QPushButton *saveBtn;
    QWidget *widget_3;
    QVBoxLayout *verticalLayout_2;
    QToolBox *toolBox;
    QWidget *widget_4;
    QGridLayout *gridLayout_20;
    QScrollArea *scrollArea;
    QWidget *scrollAreaWidgetContents;
    QGridLayout *gridLayout_12;
    QWidget *widget_20;
    QGridLayout *gridLayout_19;
    QWidget *widget_8;
    QGridLayout *gridLayout_10;
    QWidget *widget_10;
    QGridLayout *gridLayout_7;
    QLineEdit *decSpeedEdit1;
    QLineEdit *srampEdit1;
    QLineEdit *speedEdit1;
    QLineEdit *pulseEdit1;
    QLineEdit *accSpeedEdit1;
    QLineEdit *ISpeedEdit1;
    QWidget *widget_9;
    QGridLayout *gridLayout_9;
    QLabel *label_2;
    QLabel *label_3;
    QLabel *label_4;
    QLabel *label_7;
    QLabel *label_6;
    QLabel *label_5;
    QWidget *widget_19;
    QGridLayout *gridLayout_16;
    QRadioButton *continueBut1;
    QRadioButton *motionBut1;
    QWidget *widget_11;
    QGridLayout *gridLayout_11;
    QLineEdit *nowdistanceEdit1;
    QLineEdit *movedistanceEdit1;
    QLabel *label_8;
    QLabel *label_9;
    QWidget *widget_5;
    QGridLayout *gridLayout_8;
    QPushButton *frontBut1;
    QPushButton *reversalBut1;
    QPushButton *pressfrontBut1;
    QPushButton *pressreversalBut1;
    QPushButton *setOrigin1;
    QPushButton *rebackOrigin1;
    QPushButton *stopBut1;
    QWidget *page_4;
    QGridLayout *gridLayout_21;
    QWidget *widget_15;
    QGridLayout *gridLayout_18;
    QWidget *widget_17;
    QGridLayout *gridLayout_4;
    QPushButton *reversalBut2;
    QPushButton *rebackOrigin2;
    QPushButton *pressfrontBut2;
    QPushButton *frontBut2;
    QPushButton *stopBut2;
    QPushButton *pressreversalBut2;
    QPushButton *setOrigin2;
    QWidget *widget_16;
    QGridLayout *gridLayout_17;
    QLabel *label_16;
    QLabel *label_17;
    QLineEdit *nowdistanceEdit2;
    QLineEdit *movedistanceEdit2;
    QWidget *widget_12;
    QGridLayout *gridLayout_13;
    QWidget *widget_13;
    QGridLayout *gridLayout_14;
    QLineEdit *speedEdit2;
    QLineEdit *pulseEdit2;
    QLineEdit *srampEdit2;
    QLineEdit *accSpeedEdit2;
    QLineEdit *decSpeedEdit2;
    QLineEdit *ISpeedEdit2;
    QWidget *widget_14;
    QGridLayout *gridLayout_15;
    QLabel *label_10;
    QLabel *label_11;
    QLabel *label_12;
    QLabel *label_13;
    QLabel *label_14;
    QLabel *label_15;
    QWidget *widget_18;
    QGridLayout *gridLayout_3;
    QRadioButton *continueBut2;
    QRadioButton *motionBut2;
    QWidget *widget_7;
    QGridLayout *gridLayout_22;
    QLabel *label_20;
    QComboBox *comboBox_2;
    QLabel *label_21;
    QPushButton *pushButton_6;
    QPushButton *OpenSerialButton_2;
    QTextEdit *DataReceived_2;
    QPushButton *autoFocous;
    QComboBox *PortBox_2;
    QLabel *label;
    QPushButton *connectIP;
    QLineEdit *IPEdit;
    QStackedWidget *stackedWidget;
    QWidget *page;
    QGridLayout *gridLayout_5;
    QPushButton *aotoPicture;
    QPushButton *advancePreview;
    QPushButton *showCamera;
    QSpacerItem *horizontalSpacer;
    QLabel *showimage;
    QPushButton *yolov5Btn;
    QWidget *page_2;

    void setupUi(QMainWindow *MainWindow)
    {
        if (MainWindow->objectName().isEmpty())
            MainWindow->setObjectName(QString::fromUtf8("MainWindow"));
        MainWindow->resize(1165, 754);
        QIcon icon;
        icon.addFile(QString::fromUtf8(":/new/prefix1/icon/xianwei.webp"), QSize(), QIcon::Normal, QIcon::Off);
        MainWindow->setWindowIcon(icon);
        centralwidget = new QWidget(MainWindow);
        centralwidget->setObjectName(QString::fromUtf8("centralwidget"));
        gridLayout_45 = new QGridLayout(centralwidget);
        gridLayout_45->setObjectName(QString::fromUtf8("gridLayout_45"));
        widget_2 = new QWidget(centralwidget);
        widget_2->setObjectName(QString::fromUtf8("widget_2"));
        widget_2->setMinimumSize(QSize(300, 0));
        widget_2->setMaximumSize(QSize(300, 16777215));
        QFont font;
        font.setFamily(QString::fromUtf8("\345\256\213\344\275\223"));
        font.setPointSize(12);
        widget_2->setFont(font);
        gridLayout_2 = new QGridLayout(widget_2);
        gridLayout_2->setSpacing(0);
        gridLayout_2->setObjectName(QString::fromUtf8("gridLayout_2"));
        gridLayout_2->setContentsMargins(0, 0, 0, 0);
        widget = new QWidget(widget_2);
        widget->setObjectName(QString::fromUtf8("widget"));
        gridLayout = new QGridLayout(widget);
        gridLayout->setObjectName(QString::fromUtf8("gridLayout"));
        verticalSpacer = new QSpacerItem(20, 15, QSizePolicy::Minimum, QSizePolicy::Maximum);

        gridLayout->addItem(verticalSpacer, 4, 0, 1, 1);

        breakIP = new QPushButton(widget);
        breakIP->setObjectName(QString::fromUtf8("breakIP"));

        gridLayout->addWidget(breakIP, 3, 0, 1, 2);

        widget_6 = new QWidget(widget);
        widget_6->setObjectName(QString::fromUtf8("widget_6"));
        gridLayout_6 = new QGridLayout(widget_6);
        gridLayout_6->setSpacing(0);
        gridLayout_6->setObjectName(QString::fromUtf8("gridLayout_6"));
        gridLayout_6->setContentsMargins(0, 0, 0, 0);
        openBtn = new QPushButton(widget_6);
        openBtn->setObjectName(QString::fromUtf8("openBtn"));
        QSizePolicy sizePolicy(QSizePolicy::Minimum, QSizePolicy::Fixed);
        sizePolicy.setHorizontalStretch(0);
        sizePolicy.setVerticalStretch(0);
        sizePolicy.setHeightForWidth(openBtn->sizePolicy().hasHeightForWidth());
        openBtn->setSizePolicy(sizePolicy);
        openBtn->setMinimumSize(QSize(0, 30));
        openBtn->setMaximumSize(QSize(16777215, 50));
        openBtn->setSizeIncrement(QSize(0, 40));
        openBtn->setBaseSize(QSize(0, 38));

        gridLayout_6->addWidget(openBtn, 0, 0, 1, 1);

        saveBtn = new QPushButton(widget_6);
        saveBtn->setObjectName(QString::fromUtf8("saveBtn"));

        gridLayout_6->addWidget(saveBtn, 0, 1, 1, 1);


        gridLayout->addWidget(widget_6, 5, 0, 1, 2);

        widget_3 = new QWidget(widget);
        widget_3->setObjectName(QString::fromUtf8("widget_3"));
        verticalLayout_2 = new QVBoxLayout(widget_3);
        verticalLayout_2->setSpacing(0);
        verticalLayout_2->setObjectName(QString::fromUtf8("verticalLayout_2"));
        verticalLayout_2->setContentsMargins(0, 0, 0, 0);
        toolBox = new QToolBox(widget_3);
        toolBox->setObjectName(QString::fromUtf8("toolBox"));
        widget_4 = new QWidget();
        widget_4->setObjectName(QString::fromUtf8("widget_4"));
        widget_4->setGeometry(QRect(0, 0, 282, 494));
        gridLayout_20 = new QGridLayout(widget_4);
        gridLayout_20->setObjectName(QString::fromUtf8("gridLayout_20"));
        scrollArea = new QScrollArea(widget_4);
        scrollArea->setObjectName(QString::fromUtf8("scrollArea"));
        scrollArea->setFont(font);
        scrollArea->setWidgetResizable(false);
        scrollAreaWidgetContents = new QWidget();
        scrollAreaWidgetContents->setObjectName(QString::fromUtf8("scrollAreaWidgetContents"));
        scrollAreaWidgetContents->setGeometry(QRect(0, 0, 248, 650));
        gridLayout_12 = new QGridLayout(scrollAreaWidgetContents);
        gridLayout_12->setSpacing(0);
        gridLayout_12->setObjectName(QString::fromUtf8("gridLayout_12"));
        gridLayout_12->setContentsMargins(0, 0, 0, 0);
        widget_20 = new QWidget(scrollAreaWidgetContents);
        widget_20->setObjectName(QString::fromUtf8("widget_20"));
        gridLayout_19 = new QGridLayout(widget_20);
        gridLayout_19->setObjectName(QString::fromUtf8("gridLayout_19"));
        widget_8 = new QWidget(widget_20);
        widget_8->setObjectName(QString::fromUtf8("widget_8"));
        widget_8->setMinimumSize(QSize(0, 185));
        widget_8->setMaximumSize(QSize(16777215, 150));
        gridLayout_10 = new QGridLayout(widget_8);
        gridLayout_10->setObjectName(QString::fromUtf8("gridLayout_10"));
        gridLayout_10->setHorizontalSpacing(0);
        gridLayout_10->setVerticalSpacing(3);
        gridLayout_10->setContentsMargins(0, 0, 0, 0);
        widget_10 = new QWidget(widget_8);
        widget_10->setObjectName(QString::fromUtf8("widget_10"));
        gridLayout_7 = new QGridLayout(widget_10);
        gridLayout_7->setObjectName(QString::fromUtf8("gridLayout_7"));
        decSpeedEdit1 = new QLineEdit(widget_10);
        decSpeedEdit1->setObjectName(QString::fromUtf8("decSpeedEdit1"));

        gridLayout_7->addWidget(decSpeedEdit1, 4, 0, 1, 1);

        srampEdit1 = new QLineEdit(widget_10);
        srampEdit1->setObjectName(QString::fromUtf8("srampEdit1"));

        gridLayout_7->addWidget(srampEdit1, 2, 0, 1, 1);

        speedEdit1 = new QLineEdit(widget_10);
        speedEdit1->setObjectName(QString::fromUtf8("speedEdit1"));

        gridLayout_7->addWidget(speedEdit1, 1, 0, 1, 1);

        pulseEdit1 = new QLineEdit(widget_10);
        pulseEdit1->setObjectName(QString::fromUtf8("pulseEdit1"));
        pulseEdit1->setToolTipDuration(-2);

        gridLayout_7->addWidget(pulseEdit1, 0, 0, 1, 1);

        accSpeedEdit1 = new QLineEdit(widget_10);
        accSpeedEdit1->setObjectName(QString::fromUtf8("accSpeedEdit1"));

        gridLayout_7->addWidget(accSpeedEdit1, 3, 0, 1, 1);

        ISpeedEdit1 = new QLineEdit(widget_10);
        ISpeedEdit1->setObjectName(QString::fromUtf8("ISpeedEdit1"));

        gridLayout_7->addWidget(ISpeedEdit1, 5, 0, 1, 1);


        gridLayout_10->addWidget(widget_10, 0, 1, 2, 1);

        widget_9 = new QWidget(widget_8);
        widget_9->setObjectName(QString::fromUtf8("widget_9"));
        gridLayout_9 = new QGridLayout(widget_9);
        gridLayout_9->setObjectName(QString::fromUtf8("gridLayout_9"));
        label_2 = new QLabel(widget_9);
        label_2->setObjectName(QString::fromUtf8("label_2"));

        gridLayout_9->addWidget(label_2, 0, 0, 1, 1);

        label_3 = new QLabel(widget_9);
        label_3->setObjectName(QString::fromUtf8("label_3"));

        gridLayout_9->addWidget(label_3, 3, 0, 1, 1);

        label_4 = new QLabel(widget_9);
        label_4->setObjectName(QString::fromUtf8("label_4"));

        gridLayout_9->addWidget(label_4, 1, 0, 1, 1);

        label_7 = new QLabel(widget_9);
        label_7->setObjectName(QString::fromUtf8("label_7"));

        gridLayout_9->addWidget(label_7, 5, 0, 1, 1);

        label_6 = new QLabel(widget_9);
        label_6->setObjectName(QString::fromUtf8("label_6"));

        gridLayout_9->addWidget(label_6, 4, 0, 1, 1);

        label_5 = new QLabel(widget_9);
        label_5->setObjectName(QString::fromUtf8("label_5"));

        gridLayout_9->addWidget(label_5, 2, 0, 1, 1);


        gridLayout_10->addWidget(widget_9, 0, 0, 2, 1);


        gridLayout_19->addWidget(widget_8, 0, 0, 1, 1);

        widget_19 = new QWidget(widget_20);
        widget_19->setObjectName(QString::fromUtf8("widget_19"));
        gridLayout_16 = new QGridLayout(widget_19);
        gridLayout_16->setObjectName(QString::fromUtf8("gridLayout_16"));
        continueBut1 = new QRadioButton(widget_19);
        continueBut1->setObjectName(QString::fromUtf8("continueBut1"));

        gridLayout_16->addWidget(continueBut1, 2, 0, 1, 1);

        motionBut1 = new QRadioButton(widget_19);
        motionBut1->setObjectName(QString::fromUtf8("motionBut1"));

        gridLayout_16->addWidget(motionBut1, 1, 0, 1, 1);


        gridLayout_19->addWidget(widget_19, 1, 0, 1, 1);

        widget_11 = new QWidget(widget_20);
        widget_11->setObjectName(QString::fromUtf8("widget_11"));
        gridLayout_11 = new QGridLayout(widget_11);
        gridLayout_11->setObjectName(QString::fromUtf8("gridLayout_11"));
        nowdistanceEdit1 = new QLineEdit(widget_11);
        nowdistanceEdit1->setObjectName(QString::fromUtf8("nowdistanceEdit1"));

        gridLayout_11->addWidget(nowdistanceEdit1, 2, 1, 1, 1);

        movedistanceEdit1 = new QLineEdit(widget_11);
        movedistanceEdit1->setObjectName(QString::fromUtf8("movedistanceEdit1"));

        gridLayout_11->addWidget(movedistanceEdit1, 0, 1, 1, 1);

        label_8 = new QLabel(widget_11);
        label_8->setObjectName(QString::fromUtf8("label_8"));
        label_8->setMaximumSize(QSize(100, 16777215));

        gridLayout_11->addWidget(label_8, 0, 0, 1, 1);

        label_9 = new QLabel(widget_11);
        label_9->setObjectName(QString::fromUtf8("label_9"));

        gridLayout_11->addWidget(label_9, 2, 0, 1, 1);


        gridLayout_19->addWidget(widget_11, 2, 0, 1, 1);

        widget_5 = new QWidget(widget_20);
        widget_5->setObjectName(QString::fromUtf8("widget_5"));
        gridLayout_8 = new QGridLayout(widget_5);
        gridLayout_8->setObjectName(QString::fromUtf8("gridLayout_8"));
        frontBut1 = new QPushButton(widget_5);
        frontBut1->setObjectName(QString::fromUtf8("frontBut1"));

        gridLayout_8->addWidget(frontBut1, 0, 0, 1, 1);

        reversalBut1 = new QPushButton(widget_5);
        reversalBut1->setObjectName(QString::fromUtf8("reversalBut1"));

        gridLayout_8->addWidget(reversalBut1, 0, 1, 1, 1);

        pressfrontBut1 = new QPushButton(widget_5);
        pressfrontBut1->setObjectName(QString::fromUtf8("pressfrontBut1"));

        gridLayout_8->addWidget(pressfrontBut1, 2, 0, 1, 1);

        pressreversalBut1 = new QPushButton(widget_5);
        pressreversalBut1->setObjectName(QString::fromUtf8("pressreversalBut1"));

        gridLayout_8->addWidget(pressreversalBut1, 2, 1, 1, 1);

        setOrigin1 = new QPushButton(widget_5);
        setOrigin1->setObjectName(QString::fromUtf8("setOrigin1"));

        gridLayout_8->addWidget(setOrigin1, 3, 0, 1, 1);

        rebackOrigin1 = new QPushButton(widget_5);
        rebackOrigin1->setObjectName(QString::fromUtf8("rebackOrigin1"));

        gridLayout_8->addWidget(rebackOrigin1, 3, 1, 1, 1);

        stopBut1 = new QPushButton(widget_5);
        stopBut1->setObjectName(QString::fromUtf8("stopBut1"));
        stopBut1->setMinimumSize(QSize(0, 40));
        stopBut1->setMaximumSize(QSize(16777215, 40));

        gridLayout_8->addWidget(stopBut1, 1, 0, 1, 2);


        gridLayout_19->addWidget(widget_5, 3, 0, 1, 1);


        gridLayout_12->addWidget(widget_20, 0, 0, 1, 1);

        scrollArea->setWidget(scrollAreaWidgetContents);

        gridLayout_20->addWidget(scrollArea, 0, 0, 1, 1);

        toolBox->addItem(widget_4, QString::fromUtf8("\344\270\273\350\277\220\345\212\250\346\216\247\345\210\266"));
        page_4 = new QWidget();
        page_4->setObjectName(QString::fromUtf8("page_4"));
        page_4->setGeometry(QRect(0, 0, 261, 738));
        gridLayout_21 = new QGridLayout(page_4);
        gridLayout_21->setObjectName(QString::fromUtf8("gridLayout_21"));
        widget_15 = new QWidget(page_4);
        widget_15->setObjectName(QString::fromUtf8("widget_15"));
        gridLayout_18 = new QGridLayout(widget_15);
        gridLayout_18->setObjectName(QString::fromUtf8("gridLayout_18"));
        widget_17 = new QWidget(widget_15);
        widget_17->setObjectName(QString::fromUtf8("widget_17"));
        gridLayout_4 = new QGridLayout(widget_17);
        gridLayout_4->setObjectName(QString::fromUtf8("gridLayout_4"));
        reversalBut2 = new QPushButton(widget_17);
        reversalBut2->setObjectName(QString::fromUtf8("reversalBut2"));

        gridLayout_4->addWidget(reversalBut2, 0, 1, 1, 1);

        rebackOrigin2 = new QPushButton(widget_17);
        rebackOrigin2->setObjectName(QString::fromUtf8("rebackOrigin2"));

        gridLayout_4->addWidget(rebackOrigin2, 3, 1, 1, 1);

        pressfrontBut2 = new QPushButton(widget_17);
        pressfrontBut2->setObjectName(QString::fromUtf8("pressfrontBut2"));

        gridLayout_4->addWidget(pressfrontBut2, 2, 0, 1, 1);

        frontBut2 = new QPushButton(widget_17);
        frontBut2->setObjectName(QString::fromUtf8("frontBut2"));

        gridLayout_4->addWidget(frontBut2, 0, 0, 1, 1);

        stopBut2 = new QPushButton(widget_17);
        stopBut2->setObjectName(QString::fromUtf8("stopBut2"));
        stopBut2->setMinimumSize(QSize(0, 35));
        stopBut2->setMaximumSize(QSize(16777215, 35));
        stopBut2->setSizeIncrement(QSize(0, 35));
        stopBut2->setBaseSize(QSize(0, 40));

        gridLayout_4->addWidget(stopBut2, 1, 0, 1, 2);

        pressreversalBut2 = new QPushButton(widget_17);
        pressreversalBut2->setObjectName(QString::fromUtf8("pressreversalBut2"));

        gridLayout_4->addWidget(pressreversalBut2, 2, 1, 1, 1);

        setOrigin2 = new QPushButton(widget_17);
        setOrigin2->setObjectName(QString::fromUtf8("setOrigin2"));

        gridLayout_4->addWidget(setOrigin2, 3, 0, 1, 1);


        gridLayout_18->addWidget(widget_17, 3, 0, 1, 1);

        widget_16 = new QWidget(widget_15);
        widget_16->setObjectName(QString::fromUtf8("widget_16"));
        gridLayout_17 = new QGridLayout(widget_16);
        gridLayout_17->setObjectName(QString::fromUtf8("gridLayout_17"));
        label_16 = new QLabel(widget_16);
        label_16->setObjectName(QString::fromUtf8("label_16"));

        gridLayout_17->addWidget(label_16, 2, 0, 1, 1);

        label_17 = new QLabel(widget_16);
        label_17->setObjectName(QString::fromUtf8("label_17"));
        label_17->setMaximumSize(QSize(100, 16777215));

        gridLayout_17->addWidget(label_17, 1, 0, 1, 1);

        nowdistanceEdit2 = new QLineEdit(widget_16);
        nowdistanceEdit2->setObjectName(QString::fromUtf8("nowdistanceEdit2"));

        gridLayout_17->addWidget(nowdistanceEdit2, 2, 1, 1, 1);

        movedistanceEdit2 = new QLineEdit(widget_16);
        movedistanceEdit2->setObjectName(QString::fromUtf8("movedistanceEdit2"));

        gridLayout_17->addWidget(movedistanceEdit2, 1, 1, 1, 1);


        gridLayout_18->addWidget(widget_16, 2, 0, 1, 1);

        widget_12 = new QWidget(widget_15);
        widget_12->setObjectName(QString::fromUtf8("widget_12"));
        widget_12->setMinimumSize(QSize(0, 185));
        widget_12->setMaximumSize(QSize(16777215, 150));
        gridLayout_13 = new QGridLayout(widget_12);
        gridLayout_13->setObjectName(QString::fromUtf8("gridLayout_13"));
        gridLayout_13->setHorizontalSpacing(0);
        gridLayout_13->setVerticalSpacing(3);
        gridLayout_13->setContentsMargins(0, 0, 0, 0);
        widget_13 = new QWidget(widget_12);
        widget_13->setObjectName(QString::fromUtf8("widget_13"));
        gridLayout_14 = new QGridLayout(widget_13);
        gridLayout_14->setObjectName(QString::fromUtf8("gridLayout_14"));
        speedEdit2 = new QLineEdit(widget_13);
        speedEdit2->setObjectName(QString::fromUtf8("speedEdit2"));

        gridLayout_14->addWidget(speedEdit2, 1, 0, 1, 1);

        pulseEdit2 = new QLineEdit(widget_13);
        pulseEdit2->setObjectName(QString::fromUtf8("pulseEdit2"));
        pulseEdit2->setToolTipDuration(-2);

        gridLayout_14->addWidget(pulseEdit2, 0, 0, 1, 1);

        srampEdit2 = new QLineEdit(widget_13);
        srampEdit2->setObjectName(QString::fromUtf8("srampEdit2"));

        gridLayout_14->addWidget(srampEdit2, 2, 0, 1, 1);

        accSpeedEdit2 = new QLineEdit(widget_13);
        accSpeedEdit2->setObjectName(QString::fromUtf8("accSpeedEdit2"));

        gridLayout_14->addWidget(accSpeedEdit2, 3, 0, 1, 1);

        decSpeedEdit2 = new QLineEdit(widget_13);
        decSpeedEdit2->setObjectName(QString::fromUtf8("decSpeedEdit2"));

        gridLayout_14->addWidget(decSpeedEdit2, 4, 0, 1, 1);

        ISpeedEdit2 = new QLineEdit(widget_13);
        ISpeedEdit2->setObjectName(QString::fromUtf8("ISpeedEdit2"));

        gridLayout_14->addWidget(ISpeedEdit2, 5, 0, 1, 1);


        gridLayout_13->addWidget(widget_13, 0, 1, 2, 1);

        widget_14 = new QWidget(widget_12);
        widget_14->setObjectName(QString::fromUtf8("widget_14"));
        gridLayout_15 = new QGridLayout(widget_14);
        gridLayout_15->setObjectName(QString::fromUtf8("gridLayout_15"));
        label_10 = new QLabel(widget_14);
        label_10->setObjectName(QString::fromUtf8("label_10"));

        gridLayout_15->addWidget(label_10, 0, 0, 1, 1);

        label_11 = new QLabel(widget_14);
        label_11->setObjectName(QString::fromUtf8("label_11"));

        gridLayout_15->addWidget(label_11, 3, 0, 1, 1);

        label_12 = new QLabel(widget_14);
        label_12->setObjectName(QString::fromUtf8("label_12"));

        gridLayout_15->addWidget(label_12, 1, 0, 1, 1);

        label_13 = new QLabel(widget_14);
        label_13->setObjectName(QString::fromUtf8("label_13"));

        gridLayout_15->addWidget(label_13, 5, 0, 1, 1);

        label_14 = new QLabel(widget_14);
        label_14->setObjectName(QString::fromUtf8("label_14"));

        gridLayout_15->addWidget(label_14, 4, 0, 1, 1);

        label_15 = new QLabel(widget_14);
        label_15->setObjectName(QString::fromUtf8("label_15"));

        gridLayout_15->addWidget(label_15, 2, 0, 1, 1);


        gridLayout_13->addWidget(widget_14, 0, 0, 2, 1);


        gridLayout_18->addWidget(widget_12, 0, 0, 1, 1);

        widget_18 = new QWidget(widget_15);
        widget_18->setObjectName(QString::fromUtf8("widget_18"));
        gridLayout_3 = new QGridLayout(widget_18);
        gridLayout_3->setObjectName(QString::fromUtf8("gridLayout_3"));
        continueBut2 = new QRadioButton(widget_18);
        continueBut2->setObjectName(QString::fromUtf8("continueBut2"));

        gridLayout_3->addWidget(continueBut2, 2, 0, 1, 1);

        motionBut2 = new QRadioButton(widget_18);
        motionBut2->setObjectName(QString::fromUtf8("motionBut2"));

        gridLayout_3->addWidget(motionBut2, 1, 0, 1, 1);


        gridLayout_18->addWidget(widget_18, 1, 0, 1, 1);

        widget_7 = new QWidget(widget_15);
        widget_7->setObjectName(QString::fromUtf8("widget_7"));
        widget_7->setMinimumSize(QSize(0, 185));
        widget_7->setMaximumSize(QSize(16777215, 185));
        gridLayout_22 = new QGridLayout(widget_7);
        gridLayout_22->setObjectName(QString::fromUtf8("gridLayout_22"));
        label_20 = new QLabel(widget_7);
        label_20->setObjectName(QString::fromUtf8("label_20"));
        label_20->setMinimumSize(QSize(100, 30));
        label_20->setMaximumSize(QSize(100, 16777215));
        label_20->setBaseSize(QSize(0, 30));

        gridLayout_22->addWidget(label_20, 0, 1, 1, 2);

        comboBox_2 = new QComboBox(widget_7);
        comboBox_2->addItem(QString());
        comboBox_2->addItem(QString());
        comboBox_2->addItem(QString());
        comboBox_2->addItem(QString());
        comboBox_2->addItem(QString());
        comboBox_2->addItem(QString());
        comboBox_2->addItem(QString());
        comboBox_2->addItem(QString());
        comboBox_2->addItem(QString());
        comboBox_2->addItem(QString());
        comboBox_2->addItem(QString());
        comboBox_2->addItem(QString());
        comboBox_2->addItem(QString());
        comboBox_2->addItem(QString());
        comboBox_2->addItem(QString());
        comboBox_2->addItem(QString());
        comboBox_2->setObjectName(QString::fromUtf8("comboBox_2"));
        comboBox_2->setMinimumSize(QSize(0, 30));
        comboBox_2->setBaseSize(QSize(0, 30));

        gridLayout_22->addWidget(comboBox_2, 0, 3, 1, 1);

        label_21 = new QLabel(widget_7);
        label_21->setObjectName(QString::fromUtf8("label_21"));
        label_21->setMinimumSize(QSize(0, 30));
        label_21->setBaseSize(QSize(0, 30));

        gridLayout_22->addWidget(label_21, 1, 1, 1, 1);

        pushButton_6 = new QPushButton(widget_7);
        pushButton_6->setObjectName(QString::fromUtf8("pushButton_6"));
        pushButton_6->setMinimumSize(QSize(0, 30));
        pushButton_6->setBaseSize(QSize(0, 30));

        gridLayout_22->addWidget(pushButton_6, 2, 1, 1, 2);

        OpenSerialButton_2 = new QPushButton(widget_7);
        OpenSerialButton_2->setObjectName(QString::fromUtf8("OpenSerialButton_2"));
        OpenSerialButton_2->setMinimumSize(QSize(0, 30));
        OpenSerialButton_2->setMaximumSize(QSize(16777215, 40));
        OpenSerialButton_2->setBaseSize(QSize(0, 30));

        gridLayout_22->addWidget(OpenSerialButton_2, 2, 3, 1, 1);

        DataReceived_2 = new QTextEdit(widget_7);
        DataReceived_2->setObjectName(QString::fromUtf8("DataReceived_2"));
        DataReceived_2->setMinimumSize(QSize(0, 30));
        DataReceived_2->setMaximumSize(QSize(500, 30));

        gridLayout_22->addWidget(DataReceived_2, 3, 1, 2, 3);

        autoFocous = new QPushButton(widget_7);
        autoFocous->setObjectName(QString::fromUtf8("autoFocous"));
        autoFocous->setMinimumSize(QSize(0, 30));
        autoFocous->setBaseSize(QSize(0, 30));
        autoFocous->setStyleSheet(QString::fromUtf8("background-color: rgb(0, 170, 255);"));

        gridLayout_22->addWidget(autoFocous, 5, 1, 1, 3);

        PortBox_2 = new QComboBox(widget_7);
        PortBox_2->addItem(QString());
        PortBox_2->setObjectName(QString::fromUtf8("PortBox_2"));
        PortBox_2->setMinimumSize(QSize(0, 30));
        PortBox_2->setMaximumSize(QSize(16777215, 40));
        PortBox_2->setBaseSize(QSize(0, 30));

        gridLayout_22->addWidget(PortBox_2, 1, 3, 1, 1);


        gridLayout_18->addWidget(widget_7, 4, 0, 1, 1);


        gridLayout_21->addWidget(widget_15, 0, 0, 1, 1);

        toolBox->addItem(page_4, QString::fromUtf8("\345\211\257\350\277\220\345\212\250\346\216\247\345\210\266"));

        verticalLayout_2->addWidget(toolBox);


        gridLayout->addWidget(widget_3, 6, 0, 1, 2);

        label = new QLabel(widget);
        label->setObjectName(QString::fromUtf8("label"));

        gridLayout->addWidget(label, 0, 0, 1, 1);

        connectIP = new QPushButton(widget);
        connectIP->setObjectName(QString::fromUtf8("connectIP"));

        gridLayout->addWidget(connectIP, 2, 0, 1, 2);

        IPEdit = new QLineEdit(widget);
        IPEdit->setObjectName(QString::fromUtf8("IPEdit"));

        gridLayout->addWidget(IPEdit, 0, 1, 1, 1);


        gridLayout_2->addWidget(widget, 0, 0, 1, 1);


        gridLayout_45->addWidget(widget_2, 0, 0, 1, 1);

        stackedWidget = new QStackedWidget(centralwidget);
        stackedWidget->setObjectName(QString::fromUtf8("stackedWidget"));
        page = new QWidget();
        page->setObjectName(QString::fromUtf8("page"));
        gridLayout_5 = new QGridLayout(page);
        gridLayout_5->setObjectName(QString::fromUtf8("gridLayout_5"));
        aotoPicture = new QPushButton(page);
        aotoPicture->setObjectName(QString::fromUtf8("aotoPicture"));
        aotoPicture->setMinimumSize(QSize(120, 0));
        aotoPicture->setFont(font);

        gridLayout_5->addWidget(aotoPicture, 0, 2, 1, 1);

        advancePreview = new QPushButton(page);
        advancePreview->setObjectName(QString::fromUtf8("advancePreview"));
        advancePreview->setMinimumSize(QSize(120, 0));
        advancePreview->setMaximumSize(QSize(120, 16777215));
        advancePreview->setFont(font);

        gridLayout_5->addWidget(advancePreview, 0, 3, 1, 1);

        showCamera = new QPushButton(page);
        showCamera->setObjectName(QString::fromUtf8("showCamera"));
        showCamera->setMinimumSize(QSize(120, 0));
        showCamera->setMaximumSize(QSize(120, 16777215));
        showCamera->setFont(font);

        gridLayout_5->addWidget(showCamera, 0, 0, 1, 1);

        horizontalSpacer = new QSpacerItem(30, 20, QSizePolicy::Expanding, QSizePolicy::Minimum);

        gridLayout_5->addItem(horizontalSpacer, 0, 5, 1, 1);

        showimage = new QLabel(page);
        showimage->setObjectName(QString::fromUtf8("showimage"));
        showimage->setFocusPolicy(Qt::StrongFocus);
        showimage->setFrameShape(QFrame::Box);

        gridLayout_5->addWidget(showimage, 3, 0, 1, 6);

        yolov5Btn = new QPushButton(page);
        yolov5Btn->setObjectName(QString::fromUtf8("yolov5Btn"));
        yolov5Btn->setFont(font);

        gridLayout_5->addWidget(yolov5Btn, 0, 4, 1, 1);

        stackedWidget->addWidget(page);
        page_2 = new QWidget();
        page_2->setObjectName(QString::fromUtf8("page_2"));
        stackedWidget->addWidget(page_2);

        gridLayout_45->addWidget(stackedWidget, 0, 1, 1, 1);

        MainWindow->setCentralWidget(centralwidget);

        retranslateUi(MainWindow);

        toolBox->setCurrentIndex(1);
        toolBox->layout()->setSpacing(4);


        QMetaObject::connectSlotsByName(MainWindow);
    } // setupUi

    void retranslateUi(QMainWindow *MainWindow)
    {
        MainWindow->setWindowTitle(QCoreApplication::translate("MainWindow", "\350\207\252\351\200\202\345\272\224\345\217\230\345\200\215\346\230\276\345\276\256\347\232\204\345\244\247\350\214\203\345\233\264\345\276\256\350\247\202\347\221\225\347\226\265\346\243\200\346\265\213\347\263\273\347\273\237", nullptr));
#if QT_CONFIG(tooltip)
        MainWindow->setToolTip(QCoreApplication::translate("MainWindow", "\350\207\252\351\200\202\345\272\224\345\217\230\345\200\215\346\230\276\345\276\256\347\232\204\345\244\247\350\214\203\345\233\264\350\241\250\351\235\242\345\276\256\350\247\202\347\221\225\347\226\265\346\243\200\346\265\213\347\263\273\347\273\237", nullptr));
#endif // QT_CONFIG(tooltip)
#if QT_CONFIG(tooltip)
        breakIP->setToolTip(QCoreApplication::translate("MainWindow", "\346\226\255\345\274\200\350\277\236\346\216\245", nullptr));
#endif // QT_CONFIG(tooltip)
        breakIP->setText(QCoreApplication::translate("MainWindow", "\346\226\255\345\274\200\350\277\236\346\216\245", nullptr));
#if QT_CONFIG(tooltip)
        openBtn->setToolTip(QCoreApplication::translate("MainWindow", "\347\233\270\346\234\272\350\256\276\347\275\256", nullptr));
#endif // QT_CONFIG(tooltip)
        openBtn->setText(QCoreApplication::translate("MainWindow", "\346\211\223\345\274\200\347\233\270\346\234\272", nullptr));
#if QT_CONFIG(tooltip)
        saveBtn->setToolTip(QCoreApplication::translate("MainWindow", "\344\277\235\345\255\230\345\233\276\347\211\207", nullptr));
#endif // QT_CONFIG(tooltip)
        saveBtn->setText(QCoreApplication::translate("MainWindow", "\344\277\235\345\255\230\345\233\276\347\211\207", nullptr));
#if QT_CONFIG(tooltip)
        toolBox->setToolTip(QCoreApplication::translate("MainWindow", "\344\270\273\350\277\220\345\212\250\350\256\276\347\275\256", nullptr));
#endif // QT_CONFIG(tooltip)
#if QT_CONFIG(tooltip)
        decSpeedEdit1->setToolTip(QCoreApplication::translate("MainWindow", "\345\207\217\351\200\237\345\272\246", nullptr));
#endif // QT_CONFIG(tooltip)
#if QT_CONFIG(tooltip)
        srampEdit1->setToolTip(QCoreApplication::translate("MainWindow", "\350\265\267\345\247\213\351\200\237\345\272\246", nullptr));
#endif // QT_CONFIG(tooltip)
#if QT_CONFIG(tooltip)
        speedEdit1->setToolTip(QCoreApplication::translate("MainWindow", "\347\224\265\346\234\272\351\200\237\345\272\246", nullptr));
#endif // QT_CONFIG(tooltip)
#if QT_CONFIG(tooltip)
        pulseEdit1->setToolTip(QCoreApplication::translate("MainWindow", "\350\204\211\345\206\262\345\275\223\351\207\217", nullptr));
#endif // QT_CONFIG(tooltip)
#if QT_CONFIG(tooltip)
        accSpeedEdit1->setToolTip(QCoreApplication::translate("MainWindow", "\345\212\240\351\200\237\345\272\246", nullptr));
#endif // QT_CONFIG(tooltip)
#if QT_CONFIG(tooltip)
        ISpeedEdit1->setToolTip(QCoreApplication::translate("MainWindow", "s\346\233\262\347\272\277\346\227\266\351\227\264", nullptr));
#endif // QT_CONFIG(tooltip)
#if QT_CONFIG(tooltip)
        label_2->setToolTip(QCoreApplication::translate("MainWindow", "\350\204\211\345\206\262\345\275\223\351\207\217", nullptr));
#endif // QT_CONFIG(tooltip)
        label_2->setText(QCoreApplication::translate("MainWindow", "\350\204\211\345\206\262\345\275\223\351\207\217", nullptr));
#if QT_CONFIG(tooltip)
        label_3->setToolTip(QCoreApplication::translate("MainWindow", "\345\212\240\351\200\237\345\272\246", nullptr));
#endif // QT_CONFIG(tooltip)
        label_3->setText(QCoreApplication::translate("MainWindow", "\345\212\240\351\200\237\345\272\246", nullptr));
#if QT_CONFIG(tooltip)
        label_4->setToolTip(QCoreApplication::translate("MainWindow", "\351\200\237\345\272\246", nullptr));
#endif // QT_CONFIG(tooltip)
        label_4->setText(QCoreApplication::translate("MainWindow", "\351\200\237\345\272\246", nullptr));
#if QT_CONFIG(tooltip)
        label_7->setToolTip(QCoreApplication::translate("MainWindow", "s\346\233\262\347\272\277\346\227\266\351\227\264", nullptr));
#endif // QT_CONFIG(tooltip)
        label_7->setText(QCoreApplication::translate("MainWindow", "s\346\233\262\347\272\277\346\227\266\351\227\264", nullptr));
#if QT_CONFIG(tooltip)
        label_6->setToolTip(QCoreApplication::translate("MainWindow", "\345\207\217\351\200\237\345\272\246", nullptr));
#endif // QT_CONFIG(tooltip)
        label_6->setText(QCoreApplication::translate("MainWindow", "\345\207\217\351\200\237\345\272\246", nullptr));
#if QT_CONFIG(tooltip)
        label_5->setToolTip(QCoreApplication::translate("MainWindow", "\350\265\267\345\247\213\351\200\237\345\272\246", nullptr));
#endif // QT_CONFIG(tooltip)
        label_5->setText(QCoreApplication::translate("MainWindow", "\350\265\267\345\247\213\351\200\237\345\272\246", nullptr));
        continueBut1->setText(QCoreApplication::translate("MainWindow", "\346\214\201\347\273\255\350\277\220\345\212\250", nullptr));
        motionBut1->setText(QCoreApplication::translate("MainWindow", "\347\224\265\346\234\272\345\257\270\345\212\250", nullptr));
#if QT_CONFIG(tooltip)
        nowdistanceEdit1->setToolTip(QCoreApplication::translate("MainWindow", "\350\275\264\345\275\223\345\211\215\344\275\215\347\275\256", nullptr));
#endif // QT_CONFIG(tooltip)
#if QT_CONFIG(tooltip)
        movedistanceEdit1->setToolTip(QCoreApplication::translate("MainWindow", "\347\247\273\345\212\250\350\267\235\347\246\273", nullptr));
#endif // QT_CONFIG(tooltip)
#if QT_CONFIG(tooltip)
        label_8->setToolTip(QCoreApplication::translate("MainWindow", "\347\247\273\345\212\250\350\267\235\347\246\273", nullptr));
#endif // QT_CONFIG(tooltip)
        label_8->setText(QCoreApplication::translate("MainWindow", "\347\247\273\345\212\250\350\267\235\347\246\273\357\274\232", nullptr));
#if QT_CONFIG(tooltip)
        label_9->setToolTip(QCoreApplication::translate("MainWindow", "\350\275\264\345\275\223\345\211\215\344\275\215\347\275\256", nullptr));
#endif // QT_CONFIG(tooltip)
        label_9->setText(QCoreApplication::translate("MainWindow", "\350\275\264\345\275\223\345\211\215\344\275\215\347\275\256\357\274\232", nullptr));
#if QT_CONFIG(tooltip)
        frontBut1->setToolTip(QCoreApplication::translate("MainWindow", "\347\224\265\346\234\272\346\255\243\350\275\254", nullptr));
#endif // QT_CONFIG(tooltip)
        frontBut1->setText(QCoreApplication::translate("MainWindow", "\347\224\265\346\234\272\346\255\243\350\275\254", nullptr));
#if QT_CONFIG(tooltip)
        reversalBut1->setToolTip(QCoreApplication::translate("MainWindow", "\347\224\265\346\234\272\346\226\271\350\275\254", nullptr));
#endif // QT_CONFIG(tooltip)
        reversalBut1->setText(QCoreApplication::translate("MainWindow", "\347\224\265\346\234\272\345\217\215\350\275\254", nullptr));
#if QT_CONFIG(tooltip)
        pressfrontBut1->setToolTip(QCoreApplication::translate("MainWindow", "\351\225\277\346\214\211\346\255\243\350\275\254", nullptr));
#endif // QT_CONFIG(tooltip)
        pressfrontBut1->setText(QCoreApplication::translate("MainWindow", "\351\225\277\346\214\211\346\255\243\350\275\254", nullptr));
#if QT_CONFIG(tooltip)
        pressreversalBut1->setToolTip(QCoreApplication::translate("MainWindow", "\351\225\277\346\214\211\346\226\271\350\275\254", nullptr));
#endif // QT_CONFIG(tooltip)
        pressreversalBut1->setText(QCoreApplication::translate("MainWindow", "\351\225\277\346\214\211\345\217\215\350\275\254", nullptr));
#if QT_CONFIG(tooltip)
        setOrigin1->setToolTip(QCoreApplication::translate("MainWindow", "\350\256\276\347\275\256\345\216\237\347\202\271", nullptr));
#endif // QT_CONFIG(tooltip)
        setOrigin1->setText(QCoreApplication::translate("MainWindow", "\350\256\276\347\275\256\345\216\237\347\202\271", nullptr));
#if QT_CONFIG(tooltip)
        rebackOrigin1->setToolTip(QCoreApplication::translate("MainWindow", "\345\233\236\345\210\260\345\216\237\347\202\271", nullptr));
#endif // QT_CONFIG(tooltip)
        rebackOrigin1->setText(QCoreApplication::translate("MainWindow", "\345\233\236\345\210\260\345\216\237\347\202\271", nullptr));
        stopBut1->setText(QCoreApplication::translate("MainWindow", "\345\201\234\346\255\242", nullptr));
        toolBox->setItemText(toolBox->indexOf(widget_4), QCoreApplication::translate("MainWindow", "\344\270\273\350\277\220\345\212\250\346\216\247\345\210\266", nullptr));
#if QT_CONFIG(tooltip)
        reversalBut2->setToolTip(QCoreApplication::translate("MainWindow", "\347\224\265\346\234\272\345\217\215\350\275\254", nullptr));
#endif // QT_CONFIG(tooltip)
        reversalBut2->setText(QCoreApplication::translate("MainWindow", "\347\224\265\346\234\272\345\217\215\350\275\254", nullptr));
#if QT_CONFIG(tooltip)
        rebackOrigin2->setToolTip(QCoreApplication::translate("MainWindow", "\345\233\236\345\210\260\345\216\237\347\202\271", nullptr));
#endif // QT_CONFIG(tooltip)
        rebackOrigin2->setText(QCoreApplication::translate("MainWindow", "\350\207\252\345\212\250\350\260\203\351\233\266", nullptr));
#if QT_CONFIG(tooltip)
        pressfrontBut2->setToolTip(QCoreApplication::translate("MainWindow", "\351\225\277\346\214\211\346\255\243\350\275\254", nullptr));
#endif // QT_CONFIG(tooltip)
        pressfrontBut2->setText(QCoreApplication::translate("MainWindow", "\351\225\277\346\214\211\346\255\243\350\275\254", nullptr));
#if QT_CONFIG(tooltip)
        frontBut2->setToolTip(QCoreApplication::translate("MainWindow", "\347\224\265\346\234\272\346\255\243\350\275\254", nullptr));
#endif // QT_CONFIG(tooltip)
        frontBut2->setText(QCoreApplication::translate("MainWindow", "\347\224\265\346\234\272\346\255\243\350\275\254", nullptr));
        stopBut2->setText(QCoreApplication::translate("MainWindow", "\345\201\234\346\255\242", nullptr));
#if QT_CONFIG(tooltip)
        pressreversalBut2->setToolTip(QCoreApplication::translate("MainWindow", "\351\225\277\346\214\211\345\217\215\350\275\254", nullptr));
#endif // QT_CONFIG(tooltip)
        pressreversalBut2->setText(QCoreApplication::translate("MainWindow", "\351\225\277\346\214\211\345\217\215\350\275\254", nullptr));
#if QT_CONFIG(tooltip)
        setOrigin2->setToolTip(QCoreApplication::translate("MainWindow", "\350\256\276\347\275\256\345\216\237\347\202\271", nullptr));
#endif // QT_CONFIG(tooltip)
        setOrigin2->setText(QCoreApplication::translate("MainWindow", "\350\256\276\347\275\256\345\216\237\347\202\271", nullptr));
#if QT_CONFIG(tooltip)
        label_16->setToolTip(QCoreApplication::translate("MainWindow", "\350\275\264\345\275\223\345\211\215\344\275\215\347\275\256", nullptr));
#endif // QT_CONFIG(tooltip)
        label_16->setText(QCoreApplication::translate("MainWindow", "\350\275\264\345\275\223\345\211\215\344\275\215\347\275\256\357\274\232", nullptr));
#if QT_CONFIG(tooltip)
        label_17->setToolTip(QCoreApplication::translate("MainWindow", "\347\247\273\345\212\250\350\267\235\347\246\273", nullptr));
#endif // QT_CONFIG(tooltip)
        label_17->setText(QCoreApplication::translate("MainWindow", "\347\247\273\345\212\250\350\267\235\347\246\273\357\274\232", nullptr));
#if QT_CONFIG(tooltip)
        nowdistanceEdit2->setToolTip(QCoreApplication::translate("MainWindow", "\350\275\264\345\275\223\345\211\215\344\275\215\347\275\256", nullptr));
#endif // QT_CONFIG(tooltip)
#if QT_CONFIG(tooltip)
        movedistanceEdit2->setToolTip(QCoreApplication::translate("MainWindow", "\347\247\273\345\212\250\350\267\235\347\246\273", nullptr));
#endif // QT_CONFIG(tooltip)
#if QT_CONFIG(tooltip)
        speedEdit2->setToolTip(QCoreApplication::translate("MainWindow", "\347\224\265\346\234\272\351\200\237\345\272\246", nullptr));
#endif // QT_CONFIG(tooltip)
#if QT_CONFIG(tooltip)
        pulseEdit2->setToolTip(QCoreApplication::translate("MainWindow", "\350\204\211\345\206\262\345\275\223\351\207\217", nullptr));
#endif // QT_CONFIG(tooltip)
#if QT_CONFIG(tooltip)
        srampEdit2->setToolTip(QCoreApplication::translate("MainWindow", "\350\265\267\345\247\213\351\200\237\345\272\246", nullptr));
#endif // QT_CONFIG(tooltip)
#if QT_CONFIG(tooltip)
        accSpeedEdit2->setToolTip(QCoreApplication::translate("MainWindow", "\345\212\240\351\200\237\345\272\246", nullptr));
#endif // QT_CONFIG(tooltip)
#if QT_CONFIG(tooltip)
        decSpeedEdit2->setToolTip(QCoreApplication::translate("MainWindow", "\345\207\217\351\200\237\345\272\246", nullptr));
#endif // QT_CONFIG(tooltip)
#if QT_CONFIG(tooltip)
        ISpeedEdit2->setToolTip(QCoreApplication::translate("MainWindow", "s\346\233\262\347\272\277\346\227\266\351\227\264", nullptr));
#endif // QT_CONFIG(tooltip)
#if QT_CONFIG(tooltip)
        label_10->setToolTip(QCoreApplication::translate("MainWindow", "\350\204\211\345\206\262\345\275\223\351\207\217", nullptr));
#endif // QT_CONFIG(tooltip)
        label_10->setText(QCoreApplication::translate("MainWindow", "\350\204\211\345\206\262\345\275\223\351\207\217", nullptr));
#if QT_CONFIG(tooltip)
        label_11->setToolTip(QCoreApplication::translate("MainWindow", "\345\212\240\351\200\237\345\272\246", nullptr));
#endif // QT_CONFIG(tooltip)
        label_11->setText(QCoreApplication::translate("MainWindow", "\345\212\240\351\200\237\345\272\246", nullptr));
#if QT_CONFIG(tooltip)
        label_12->setToolTip(QCoreApplication::translate("MainWindow", "\351\200\237\345\272\246", nullptr));
#endif // QT_CONFIG(tooltip)
        label_12->setText(QCoreApplication::translate("MainWindow", "\351\200\237\345\272\246", nullptr));
#if QT_CONFIG(tooltip)
        label_13->setToolTip(QCoreApplication::translate("MainWindow", "s\346\233\262\347\272\277\346\227\266\351\227\264", nullptr));
#endif // QT_CONFIG(tooltip)
        label_13->setText(QCoreApplication::translate("MainWindow", "s\346\233\262\347\272\277\346\227\266\351\227\264", nullptr));
#if QT_CONFIG(tooltip)
        label_14->setToolTip(QCoreApplication::translate("MainWindow", "\345\207\217\351\200\237\345\272\246", nullptr));
#endif // QT_CONFIG(tooltip)
        label_14->setText(QCoreApplication::translate("MainWindow", "\345\207\217\351\200\237\345\272\246", nullptr));
#if QT_CONFIG(tooltip)
        label_15->setToolTip(QCoreApplication::translate("MainWindow", "\350\265\267\345\247\213\351\200\237\345\272\246", nullptr));
#endif // QT_CONFIG(tooltip)
        label_15->setText(QCoreApplication::translate("MainWindow", "\350\265\267\345\247\213\351\200\237\345\272\246", nullptr));
        continueBut2->setText(QCoreApplication::translate("MainWindow", "\346\214\201\347\273\255\350\277\220\345\212\250", nullptr));
        motionBut2->setText(QCoreApplication::translate("MainWindow", "\347\224\265\346\234\272\345\257\270\345\212\250", nullptr));
        label_20->setText(QCoreApplication::translate("MainWindow", "\345\200\215\346\225\260\351\200\211\346\213\251\357\274\232", nullptr));
        comboBox_2->setItemText(0, QCoreApplication::translate("MainWindow", "16\345\200\215", nullptr));
        comboBox_2->setItemText(1, QCoreApplication::translate("MainWindow", "16.5\345\200\215", nullptr));
        comboBox_2->setItemText(2, QCoreApplication::translate("MainWindow", "17\345\200\215", nullptr));
        comboBox_2->setItemText(3, QCoreApplication::translate("MainWindow", "17.5\345\200\215", nullptr));
        comboBox_2->setItemText(4, QCoreApplication::translate("MainWindow", "18\345\200\215", nullptr));
        comboBox_2->setItemText(5, QCoreApplication::translate("MainWindow", "18.5\345\200\215", nullptr));
        comboBox_2->setItemText(6, QCoreApplication::translate("MainWindow", "19\345\200\215", nullptr));
        comboBox_2->setItemText(7, QCoreApplication::translate("MainWindow", "19.5\345\200\215", nullptr));
        comboBox_2->setItemText(8, QCoreApplication::translate("MainWindow", "20\345\200\215", nullptr));
        comboBox_2->setItemText(9, QCoreApplication::translate("MainWindow", "20.5\345\200\215", nullptr));
        comboBox_2->setItemText(10, QCoreApplication::translate("MainWindow", "21\345\200\215", nullptr));
        comboBox_2->setItemText(11, QCoreApplication::translate("MainWindow", "21.5\345\200\215", nullptr));
        comboBox_2->setItemText(12, QCoreApplication::translate("MainWindow", "22\345\200\215", nullptr));
        comboBox_2->setItemText(13, QCoreApplication::translate("MainWindow", "22.5\345\200\215", nullptr));
        comboBox_2->setItemText(14, QCoreApplication::translate("MainWindow", "23\345\200\215", nullptr));
        comboBox_2->setItemText(15, QCoreApplication::translate("MainWindow", "23.5\345\200\215", nullptr));

        label_21->setText(QCoreApplication::translate("MainWindow", "\344\270\262\345\217\243\351\200\211\346\213\251\357\274\232", nullptr));
        pushButton_6->setText(QCoreApplication::translate("MainWindow", "\345\274\200\345\247\213\346\224\276\345\244\247", nullptr));
        OpenSerialButton_2->setText(QCoreApplication::translate("MainWindow", "\346\211\223\345\274\200\344\270\262\345\217\243", nullptr));
        autoFocous->setText(QCoreApplication::translate("MainWindow", "\345\274\200\345\247\213\350\201\232\347\204\246", nullptr));
        PortBox_2->setItemText(0, QCoreApplication::translate("MainWindow", "\345\210\267\346\226\260", nullptr));

#if QT_CONFIG(tooltip)
        PortBox_2->setToolTip(QCoreApplication::translate("MainWindow", "\344\270\262\345\217\243\351\200\211\346\213\251", nullptr));
#endif // QT_CONFIG(tooltip)
        toolBox->setItemText(toolBox->indexOf(page_4), QCoreApplication::translate("MainWindow", "\345\211\257\350\277\220\345\212\250\346\216\247\345\210\266", nullptr));
#if QT_CONFIG(tooltip)
        label->setToolTip(QCoreApplication::translate("MainWindow", "ip\350\277\236\346\216\245", nullptr));
#endif // QT_CONFIG(tooltip)
        label->setText(QCoreApplication::translate("MainWindow", "ip\350\277\236\346\216\245\357\274\232", nullptr));
#if QT_CONFIG(tooltip)
        connectIP->setToolTip(QCoreApplication::translate("MainWindow", "\350\277\236\346\216\245ip", nullptr));
#endif // QT_CONFIG(tooltip)
        connectIP->setText(QCoreApplication::translate("MainWindow", "\350\277\236\346\216\245IP", nullptr));
#if QT_CONFIG(tooltip)
        IPEdit->setToolTip(QCoreApplication::translate("MainWindow", "ip\345\234\260\345\235\200", nullptr));
#endif // QT_CONFIG(tooltip)
        IPEdit->setPlaceholderText(QCoreApplication::translate("MainWindow", "\350\257\267\350\276\223\345\205\245", nullptr));
        aotoPicture->setText(QCoreApplication::translate("MainWindow", "\350\207\252\345\212\250\351\207\207\345\233\276", nullptr));
        advancePreview->setText(QCoreApplication::translate("MainWindow", "\345\233\276\347\211\207\351\242\204\350\247\210", nullptr));
        showCamera->setText(QCoreApplication::translate("MainWindow", "\347\233\270\346\234\272\346\230\276\347\244\272", nullptr));
        showimage->setText(QString());
        yolov5Btn->setText(QCoreApplication::translate("MainWindow", "yolo\346\243\200\346\265\213", nullptr));
    } // retranslateUi

};

namespace Ui {
    class MainWindow: public Ui_MainWindow {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_MAINWINDOW_H
