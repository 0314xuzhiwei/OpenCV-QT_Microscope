QT       += core gui
QT       += serialport #串口通信

greaterThan(QT_MAJOR_VERSION, 4): QT += widgets

CONFIG += c++11

# The following define makes your compiler emit warnings if you use
# any Qt feature that has been marked deprecated (the exact warnings
# depend on your compiler). Please consult the documentation of the
# deprecated API in order to know how to port your code away from it.
DEFINES += QT_DEPRECATED_WARNINGS

# You can also make your code fail to compile if it uses deprecated APIs.
# In order to do so, uncomment the following line.
# You can also select to disable deprecated APIs only up to a certain version of Qt.
#DEFINES += QT_DISABLE_DEPRECATED_BEFORE=0x060000    # disables all the APIs deprecated before Qt 6.0.0

SOURCES += \
    control/camerathread.cpp \
    control/cammotorthread.cpp \
    control/controlall.cpp \
    control/mainwinthread.cpp \
    control/motorthread.cpp \
    main.cpp \
    mainwindow.cpp \
    model/cameractr.cpp \
    model/motorctr.cpp \
    showimage.cpp \
    yoloc.cpp \
    yolov5.cpp

HEADERS += \
    control/camerathread.h \
    control/cammotorthread.h \
    control/controlall.h \
    control/mainwinthread.h \
    control/motorthread.h \
    mainwindow.h \
    model/cameractr.h \
    model/motorctr.h \
    myDefine.h \
    showimage.h \
    yoloc.h \
    yolov5.h

FORMS += \
    mainwindow.ui \
 \    #smollwidget.ui
    showimage.ui \
    yoloc.ui

# Default rules for deployment.
qnx: target.path = /tmp/$${TARGET}/bin
else: unix:!android: target.path = /opt/$${TARGET}/bin
!isEmpty(target.path): INSTALLS += target



# 运动控制卡配置
INCLUDEPATH +=  $$PWD/inc

LIBS        +=  -L$$PWD/DLL_LIB \
                -lzauxdll \
                -lzmotion

#映美精相机配置
#INCLUDEPATH += $$quote(D:/QTProject/pointCloudProject/include)
#LIBS += -LD:/QTProject/pointCloudProject/bin/x64 \
        #-ltisgrabber_x64

INCLUDEPATH += $$quote(D:/0314/qt/trap/ProgrammingLog/Microscope_1_8/program/include)
LIBS += -LD:/0314/qt/trap/ProgrammingLog/Microscope_1_8/program/LIBRARY \
        -ltisgrabber_x64

include(D:\0314\qt\trap\ProgrammingLog\Microscope_1_8\program\opencv_env.pri)

#include(D:/0314/qt/trap/ProgrammingLog/Microscope_1_8/program/opencv_env.pri)

##OpenCV配置
#OPENCV = D:/0314/opencv/build

#LIBS += -LD:/0314/opencv/build/x64/vc15/lib/ -lopencv_world410d
#LIBS += -LC:/Windows/SysWOW64/ -lgdi32

#INCLUDEPATH += $$OPENCV/include
#DEPENDPATH += $$OPENCV/x64/vc15/bin

RESOURCES += \
    icon.qrc

DISTFILES += \
    opencv.pri \
    opencv配置.pri
