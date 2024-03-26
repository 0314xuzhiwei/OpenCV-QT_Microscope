#OpenCV配置
OPENCV = D:/0314/opencv/build


LIBS += -LD:/0314/opencv/build/x64/vc15/lib/ -lopencv_world410d
LIBS += -LC:/Windows/SysWOW64/ -lgdi32

INCLUDEPATH += $$OPENCV/include
DEPENDPATH += $$OPENCV/x64/vc15/bin

