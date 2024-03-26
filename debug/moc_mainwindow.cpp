/****************************************************************************
** Meta object code from reading C++ file 'mainwindow.h'
**
** Created by: The Qt Meta Object Compiler version 67 (Qt 5.14.2)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include <memory>
#include "../mainwindow.h"
#include <QtCore/qbytearray.h>
#include <QtCore/qmetatype.h>
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'mainwindow.h' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 67
#error "This file was generated using the moc from 5.14.2. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

QT_BEGIN_MOC_NAMESPACE
QT_WARNING_PUSH
QT_WARNING_DISABLE_DEPRECATED
struct qt_meta_stringdata_MainWindow_t {
    QByteArrayData data[41];
    char stringdata0[840];
};
#define QT_MOC_LITERAL(idx, ofs, len) \
    Q_STATIC_BYTE_ARRAY_DATA_HEADER_INITIALIZER_WITH_OFFSET(len, \
    qptrdiff(offsetof(qt_meta_stringdata_MainWindow_t, stringdata0) + ofs \
        - idx * sizeof(QByteArrayData)) \
    )
static const qt_meta_stringdata_MainWindow_t qt_meta_stringdata_MainWindow = {
    {
QT_MOC_LITERAL(0, 0, 10), // "MainWindow"
QT_MOC_LITERAL(1, 11, 6), // "upDate"
QT_MOC_LITERAL(2, 18, 0), // ""
QT_MOC_LITERAL(3, 19, 9), // "frontMove"
QT_MOC_LITERAL(4, 29, 12), // "reversalMove"
QT_MOC_LITERAL(5, 42, 12), // "camfrontMove"
QT_MOC_LITERAL(6, 55, 15), // "camreversalMove"
QT_MOC_LITERAL(7, 71, 14), // "SerialPortInit"
QT_MOC_LITERAL(8, 86, 17), // "RefreshSerialPort"
QT_MOC_LITERAL(9, 104, 5), // "index"
QT_MOC_LITERAL(10, 110, 12), // "DataReceived"
QT_MOC_LITERAL(11, 123, 18), // "on_openBtn_clicked"
QT_MOC_LITERAL(12, 142, 18), // "on_saveBtn_clicked"
QT_MOC_LITERAL(13, 161, 20), // "on_connectIP_clicked"
QT_MOC_LITERAL(14, 182, 18), // "on_breakIP_clicked"
QT_MOC_LITERAL(15, 201, 20), // "on_frontBut1_clicked"
QT_MOC_LITERAL(16, 222, 23), // "on_reversalBut1_clicked"
QT_MOC_LITERAL(17, 246, 21), // "on_setOrigin1_clicked"
QT_MOC_LITERAL(18, 268, 24), // "on_rebackOrigin1_clicked"
QT_MOC_LITERAL(19, 293, 19), // "on_stopBut1_clicked"
QT_MOC_LITERAL(20, 313, 21), // "on_motionBut1_clicked"
QT_MOC_LITERAL(21, 335, 23), // "on_continueBut1_clicked"
QT_MOC_LITERAL(22, 359, 25), // "on_pressfrontBut1_pressed"
QT_MOC_LITERAL(23, 385, 28), // "on_pressreversalBut1_pressed"
QT_MOC_LITERAL(24, 414, 26), // "on_pressfrontBut1_released"
QT_MOC_LITERAL(25, 441, 29), // "on_pressreversalBut1_released"
QT_MOC_LITERAL(26, 471, 20), // "on_frontBut2_clicked"
QT_MOC_LITERAL(27, 492, 23), // "on_reversalBut2_clicked"
QT_MOC_LITERAL(28, 516, 19), // "on_stopBut2_clicked"
QT_MOC_LITERAL(29, 536, 25), // "on_pressfrontBut2_pressed"
QT_MOC_LITERAL(30, 562, 26), // "on_pressfrontBut2_released"
QT_MOC_LITERAL(31, 589, 28), // "on_pressreversalBut2_pressed"
QT_MOC_LITERAL(32, 618, 29), // "on_pressreversalBut2_released"
QT_MOC_LITERAL(33, 648, 21), // "on_motionBut2_clicked"
QT_MOC_LITERAL(34, 670, 23), // "on_continueBut2_clicked"
QT_MOC_LITERAL(35, 694, 29), // "on_OpenSerialButton_2_clicked"
QT_MOC_LITERAL(36, 724, 23), // "on_pushButton_6_clicked"
QT_MOC_LITERAL(37, 748, 21), // "on_autoFocous_clicked"
QT_MOC_LITERAL(38, 770, 22), // "on_aotoPicture_clicked"
QT_MOC_LITERAL(39, 793, 25), // "on_advancePreview_clicked"
QT_MOC_LITERAL(40, 819, 20) // "on_yolov5Btn_clicked"

    },
    "MainWindow\0upDate\0\0frontMove\0reversalMove\0"
    "camfrontMove\0camreversalMove\0"
    "SerialPortInit\0RefreshSerialPort\0index\0"
    "DataReceived\0on_openBtn_clicked\0"
    "on_saveBtn_clicked\0on_connectIP_clicked\0"
    "on_breakIP_clicked\0on_frontBut1_clicked\0"
    "on_reversalBut1_clicked\0on_setOrigin1_clicked\0"
    "on_rebackOrigin1_clicked\0on_stopBut1_clicked\0"
    "on_motionBut1_clicked\0on_continueBut1_clicked\0"
    "on_pressfrontBut1_pressed\0"
    "on_pressreversalBut1_pressed\0"
    "on_pressfrontBut1_released\0"
    "on_pressreversalBut1_released\0"
    "on_frontBut2_clicked\0on_reversalBut2_clicked\0"
    "on_stopBut2_clicked\0on_pressfrontBut2_pressed\0"
    "on_pressfrontBut2_released\0"
    "on_pressreversalBut2_pressed\0"
    "on_pressreversalBut2_released\0"
    "on_motionBut2_clicked\0on_continueBut2_clicked\0"
    "on_OpenSerialButton_2_clicked\0"
    "on_pushButton_6_clicked\0on_autoFocous_clicked\0"
    "on_aotoPicture_clicked\0on_advancePreview_clicked\0"
    "on_yolov5Btn_clicked"
};
#undef QT_MOC_LITERAL

static const uint qt_meta_data_MainWindow[] = {

 // content:
       8,       // revision
       0,       // classname
       0,    0, // classinfo
      38,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       0,       // signalCount

 // slots: name, argc, parameters, tag, flags
       1,    0,  204,    2, 0x0a /* Public */,
       3,    0,  205,    2, 0x0a /* Public */,
       4,    0,  206,    2, 0x0a /* Public */,
       5,    0,  207,    2, 0x0a /* Public */,
       6,    0,  208,    2, 0x0a /* Public */,
       7,    0,  209,    2, 0x0a /* Public */,
       8,    1,  210,    2, 0x0a /* Public */,
      10,    0,  213,    2, 0x0a /* Public */,
      11,    0,  214,    2, 0x08 /* Private */,
      12,    0,  215,    2, 0x08 /* Private */,
      13,    0,  216,    2, 0x08 /* Private */,
      14,    0,  217,    2, 0x08 /* Private */,
      15,    0,  218,    2, 0x08 /* Private */,
      16,    0,  219,    2, 0x08 /* Private */,
      17,    0,  220,    2, 0x08 /* Private */,
      18,    0,  221,    2, 0x08 /* Private */,
      19,    0,  222,    2, 0x08 /* Private */,
      20,    0,  223,    2, 0x08 /* Private */,
      21,    0,  224,    2, 0x08 /* Private */,
      22,    0,  225,    2, 0x08 /* Private */,
      23,    0,  226,    2, 0x08 /* Private */,
      24,    0,  227,    2, 0x08 /* Private */,
      25,    0,  228,    2, 0x08 /* Private */,
      26,    0,  229,    2, 0x08 /* Private */,
      27,    0,  230,    2, 0x08 /* Private */,
      28,    0,  231,    2, 0x08 /* Private */,
      29,    0,  232,    2, 0x08 /* Private */,
      30,    0,  233,    2, 0x08 /* Private */,
      31,    0,  234,    2, 0x08 /* Private */,
      32,    0,  235,    2, 0x08 /* Private */,
      33,    0,  236,    2, 0x08 /* Private */,
      34,    0,  237,    2, 0x08 /* Private */,
      35,    0,  238,    2, 0x08 /* Private */,
      36,    0,  239,    2, 0x08 /* Private */,
      37,    0,  240,    2, 0x08 /* Private */,
      38,    0,  241,    2, 0x08 /* Private */,
      39,    0,  242,    2, 0x08 /* Private */,
      40,    0,  243,    2, 0x08 /* Private */,

 // slots: parameters
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void, QMetaType::Int,    9,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,

       0        // eod
};

void MainWindow::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        auto *_t = static_cast<MainWindow *>(_o);
        Q_UNUSED(_t)
        switch (_id) {
        case 0: _t->upDate(); break;
        case 1: _t->frontMove(); break;
        case 2: _t->reversalMove(); break;
        case 3: _t->camfrontMove(); break;
        case 4: _t->camreversalMove(); break;
        case 5: _t->SerialPortInit(); break;
        case 6: _t->RefreshSerialPort((*reinterpret_cast< int(*)>(_a[1]))); break;
        case 7: _t->DataReceived(); break;
        case 8: _t->on_openBtn_clicked(); break;
        case 9: _t->on_saveBtn_clicked(); break;
        case 10: _t->on_connectIP_clicked(); break;
        case 11: _t->on_breakIP_clicked(); break;
        case 12: _t->on_frontBut1_clicked(); break;
        case 13: _t->on_reversalBut1_clicked(); break;
        case 14: _t->on_setOrigin1_clicked(); break;
        case 15: _t->on_rebackOrigin1_clicked(); break;
        case 16: _t->on_stopBut1_clicked(); break;
        case 17: _t->on_motionBut1_clicked(); break;
        case 18: _t->on_continueBut1_clicked(); break;
        case 19: _t->on_pressfrontBut1_pressed(); break;
        case 20: _t->on_pressreversalBut1_pressed(); break;
        case 21: _t->on_pressfrontBut1_released(); break;
        case 22: _t->on_pressreversalBut1_released(); break;
        case 23: _t->on_frontBut2_clicked(); break;
        case 24: _t->on_reversalBut2_clicked(); break;
        case 25: _t->on_stopBut2_clicked(); break;
        case 26: _t->on_pressfrontBut2_pressed(); break;
        case 27: _t->on_pressfrontBut2_released(); break;
        case 28: _t->on_pressreversalBut2_pressed(); break;
        case 29: _t->on_pressreversalBut2_released(); break;
        case 30: _t->on_motionBut2_clicked(); break;
        case 31: _t->on_continueBut2_clicked(); break;
        case 32: _t->on_OpenSerialButton_2_clicked(); break;
        case 33: _t->on_pushButton_6_clicked(); break;
        case 34: _t->on_autoFocous_clicked(); break;
        case 35: _t->on_aotoPicture_clicked(); break;
        case 36: _t->on_advancePreview_clicked(); break;
        case 37: _t->on_yolov5Btn_clicked(); break;
        default: ;
        }
    }
}

QT_INIT_METAOBJECT const QMetaObject MainWindow::staticMetaObject = { {
    QMetaObject::SuperData::link<QMainWindow::staticMetaObject>(),
    qt_meta_stringdata_MainWindow.data,
    qt_meta_data_MainWindow,
    qt_static_metacall,
    nullptr,
    nullptr
} };


const QMetaObject *MainWindow::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->dynamicMetaObject() : &staticMetaObject;
}

void *MainWindow::qt_metacast(const char *_clname)
{
    if (!_clname) return nullptr;
    if (!strcmp(_clname, qt_meta_stringdata_MainWindow.stringdata0))
        return static_cast<void*>(this);
    return QMainWindow::qt_metacast(_clname);
}

int MainWindow::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = QMainWindow::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    if (_c == QMetaObject::InvokeMetaMethod) {
        if (_id < 38)
            qt_static_metacall(this, _c, _id, _a);
        _id -= 38;
    } else if (_c == QMetaObject::RegisterMethodArgumentMetaType) {
        if (_id < 38)
            *reinterpret_cast<int*>(_a[0]) = -1;
        _id -= 38;
    }
    return _id;
}
QT_WARNING_POP
QT_END_MOC_NAMESPACE
