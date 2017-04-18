/****************************************************************************
** Meta object code from reading C++ file 'mainwindow.h'
**
** Created by: The Qt Meta Object Compiler version 63 (Qt 4.8.6)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include "mainwindow.h"
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'mainwindow.h' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 63
#error "This file was generated using the moc from 4.8.6. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

QT_BEGIN_MOC_NAMESPACE
static const uint qt_meta_data_MainWindow[] = {

 // content:
       6,       // revision
       0,       // classname
       0,    0, // classinfo
      19,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       0,       // signalCount

 // slots: signature, parameters, type, tag, flags
      11,   31,   37,   37, 0x08,
      38,   67,   37,   37, 0x08,
      80,   37,   37,   37, 0x08,
      96,  139,   37,   37, 0x08,
     156,   37,   37,   37, 0x08,
     175,   37,   37,   37, 0x08,
     204,   37,   37,   37, 0x08,
     226,  270,   37,   37, 0x08,
     293,   37,   37,   37, 0x08,
     320,   37,   37,   37, 0x08,
     352,   37,   37,   37, 0x08,
     389,   37,   37,   37, 0x08,
     420,   37,   37,   37, 0x08,
     446,   37,   37,   37, 0x08,
     478,   37,   37,   37, 0x08,
     520,   37,   37,   37, 0x08,
     562,   37,   37,   37, 0x08,
     588,   37,   37,   37, 0x08,
     619,   37,   37,   37, 0x08,

       0        // eod
};

static const char qt_meta_stringdata_MainWindow[] = {
    "MainWindow\0handleRunState(int)\0state\0"
    "\0handleConnected(Device,bool)\0"
    "device,state\0handleActions()\0"
    "handleActionScriptlet(QString,QStringList)\0"
    "action,scriptlet\0handleLoadParams()\0"
    "handleConfigDialogFinished()\0"
    "interpreterFinished()\0"
    "handleVersion(ushort,ushort,ushort,QString)\0"
    "major,minor,build,type\0"
    "on_actionAbout_triggered()\0"
    "on_actionPlay_Pause_triggered()\0"
    "on_actionDefault_program_triggered()\0"
    "on_actionConfigure_triggered()\0"
    "on_actionHelp_triggered()\0"
    "on_actionSave_Image_triggered()\0"
    "on_actionSave_Pixy_parameters_triggered()\0"
    "on_actionLoad_Pixy_parameters_triggered()\0"
    "on_actionExit_triggered()\0"
    "on_actionRaw_video_triggered()\0"
    "on_actionCooked_video_triggered()\0"
};

void MainWindow::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        Q_ASSERT(staticMetaObject.cast(_o));
        MainWindow *_t = static_cast<MainWindow *>(_o);
        switch (_id) {
        case 0: _t->handleRunState((*reinterpret_cast< int(*)>(_a[1]))); break;
        case 1: _t->handleConnected((*reinterpret_cast< Device(*)>(_a[1])),(*reinterpret_cast< bool(*)>(_a[2]))); break;
        case 2: _t->handleActions(); break;
        case 3: _t->handleActionScriptlet((*reinterpret_cast< QString(*)>(_a[1])),(*reinterpret_cast< QStringList(*)>(_a[2]))); break;
        case 4: _t->handleLoadParams(); break;
        case 5: _t->handleConfigDialogFinished(); break;
        case 6: _t->interpreterFinished(); break;
        case 7: _t->handleVersion((*reinterpret_cast< ushort(*)>(_a[1])),(*reinterpret_cast< ushort(*)>(_a[2])),(*reinterpret_cast< ushort(*)>(_a[3])),(*reinterpret_cast< QString(*)>(_a[4]))); break;
        case 8: _t->on_actionAbout_triggered(); break;
        case 9: _t->on_actionPlay_Pause_triggered(); break;
        case 10: _t->on_actionDefault_program_triggered(); break;
        case 11: _t->on_actionConfigure_triggered(); break;
        case 12: _t->on_actionHelp_triggered(); break;
        case 13: _t->on_actionSave_Image_triggered(); break;
        case 14: _t->on_actionSave_Pixy_parameters_triggered(); break;
        case 15: _t->on_actionLoad_Pixy_parameters_triggered(); break;
        case 16: _t->on_actionExit_triggered(); break;
        case 17: _t->on_actionRaw_video_triggered(); break;
        case 18: _t->on_actionCooked_video_triggered(); break;
        default: ;
        }
    }
}

const QMetaObjectExtraData MainWindow::staticMetaObjectExtraData = {
    0,  qt_static_metacall 
};

const QMetaObject MainWindow::staticMetaObject = {
    { &QMainWindow::staticMetaObject, qt_meta_stringdata_MainWindow,
      qt_meta_data_MainWindow, &staticMetaObjectExtraData }
};

#ifdef Q_NO_DATA_RELOCATION
const QMetaObject &MainWindow::getStaticMetaObject() { return staticMetaObject; }
#endif //Q_NO_DATA_RELOCATION

const QMetaObject *MainWindow::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->metaObject : &staticMetaObject;
}

void *MainWindow::qt_metacast(const char *_clname)
{
    if (!_clname) return 0;
    if (!strcmp(_clname, qt_meta_stringdata_MainWindow))
        return static_cast<void*>(const_cast< MainWindow*>(this));
    return QMainWindow::qt_metacast(_clname);
}

int MainWindow::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = QMainWindow::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    if (_c == QMetaObject::InvokeMetaMethod) {
        if (_id < 19)
            qt_static_metacall(this, _c, _id, _a);
        _id -= 19;
    }
    return _id;
}
QT_END_MOC_NAMESPACE
