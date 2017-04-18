/****************************************************************************
** Meta object code from reading C++ file 'interpreter.h'
**
** Created by: The Qt Meta Object Compiler version 63 (Qt 4.8.6)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include "interpreter.h"
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'interpreter.h' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 63
#error "This file was generated using the moc from 4.8.6. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

QT_BEGIN_MOC_NAMESPACE
static const uint qt_meta_data_Interpreter[] = {

 // content:
       6,       // revision
       0,       // classname
       0,    0, // classinfo
      17,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
      14,       // signalCount

 // signals: signature, parameters, type, tag, flags
      12,   26,   32,   32, 0x05,
      33,   57,   32,   32, 0x05,
      68,   85,   32,   32, 0x25,
      90,   85,   32,   32, 0x05,
     105,   85,   32,   32, 0x05,
     129,   85,   32,   32, 0x05,
     145,  180,   32,   32, 0x05,
     185,  205,   32,   32, 0x05,
     212,  235,   32,   32, 0x05,
     248,  289,   32,   32, 0x05,
     312,  349,   32,   32, 0x05,
     366,  396,   32,   32, 0x05,
     404,   32,   32,   32, 0x05,
     418,  456,   32,   32, 0x05,

 // slots: signature, parameters, type, tag, flags
     479,  499,   32,   32, 0x08,
     503,  520,   32,   32, 0x08,
     528,  561,   32,   32, 0x08,

       0        // eod
};

static const char qt_meta_stringdata_Interpreter[] = {
    "Interpreter\0runState(int)\0state\0\0"
    "textOut(QString,QColor)\0text,color\0"
    "textOut(QString)\0text\0error(QString)\0"
    "consoleCommand(QString)\0prompt(QString)\0"
    "videoInput(VideoWidget::InputMode)\0"
    "mode\0enableConsole(bool)\0enable\0"
    "connected(Device,bool)\0device,state\0"
    "actionScriptlet(int,QString,QStringList)\0"
    "index,action,scriptlet\0"
    "actionScriptlet(QString,QStringList)\0"
    "action,scriptlet\0parameter(QString,QByteArray)\0"
    "id,data\0paramLoaded()\0"
    "version(ushort,ushort,ushort,QString)\0"
    "major,minor,build,type\0controlKey(Qt::Key)\0"
    "key\0command(QString)\0command\0"
    "handleSelection(int,int,int,int)\0"
    "x0,y0,width,height\0"
};

void Interpreter::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        Q_ASSERT(staticMetaObject.cast(_o));
        Interpreter *_t = static_cast<Interpreter *>(_o);
        switch (_id) {
        case 0: _t->runState((*reinterpret_cast< int(*)>(_a[1]))); break;
        case 1: _t->textOut((*reinterpret_cast< QString(*)>(_a[1])),(*reinterpret_cast< QColor(*)>(_a[2]))); break;
        case 2: _t->textOut((*reinterpret_cast< QString(*)>(_a[1]))); break;
        case 3: _t->error((*reinterpret_cast< QString(*)>(_a[1]))); break;
        case 4: _t->consoleCommand((*reinterpret_cast< QString(*)>(_a[1]))); break;
        case 5: _t->prompt((*reinterpret_cast< QString(*)>(_a[1]))); break;
        case 6: _t->videoInput((*reinterpret_cast< VideoWidget::InputMode(*)>(_a[1]))); break;
        case 7: _t->enableConsole((*reinterpret_cast< bool(*)>(_a[1]))); break;
        case 8: _t->connected((*reinterpret_cast< Device(*)>(_a[1])),(*reinterpret_cast< bool(*)>(_a[2]))); break;
        case 9: _t->actionScriptlet((*reinterpret_cast< int(*)>(_a[1])),(*reinterpret_cast< QString(*)>(_a[2])),(*reinterpret_cast< QStringList(*)>(_a[3]))); break;
        case 10: _t->actionScriptlet((*reinterpret_cast< QString(*)>(_a[1])),(*reinterpret_cast< QStringList(*)>(_a[2]))); break;
        case 11: _t->parameter((*reinterpret_cast< QString(*)>(_a[1])),(*reinterpret_cast< QByteArray(*)>(_a[2]))); break;
        case 12: _t->paramLoaded(); break;
        case 13: _t->version((*reinterpret_cast< ushort(*)>(_a[1])),(*reinterpret_cast< ushort(*)>(_a[2])),(*reinterpret_cast< ushort(*)>(_a[3])),(*reinterpret_cast< QString(*)>(_a[4]))); break;
        case 14: _t->controlKey((*reinterpret_cast< Qt::Key(*)>(_a[1]))); break;
        case 15: _t->command((*reinterpret_cast< const QString(*)>(_a[1]))); break;
        case 16: _t->handleSelection((*reinterpret_cast< int(*)>(_a[1])),(*reinterpret_cast< int(*)>(_a[2])),(*reinterpret_cast< int(*)>(_a[3])),(*reinterpret_cast< int(*)>(_a[4]))); break;
        default: ;
        }
    }
}

const QMetaObjectExtraData Interpreter::staticMetaObjectExtraData = {
    0,  qt_static_metacall 
};

const QMetaObject Interpreter::staticMetaObject = {
    { &QThread::staticMetaObject, qt_meta_stringdata_Interpreter,
      qt_meta_data_Interpreter, &staticMetaObjectExtraData }
};

#ifdef Q_NO_DATA_RELOCATION
const QMetaObject &Interpreter::getStaticMetaObject() { return staticMetaObject; }
#endif //Q_NO_DATA_RELOCATION

const QMetaObject *Interpreter::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->metaObject : &staticMetaObject;
}

void *Interpreter::qt_metacast(const char *_clname)
{
    if (!_clname) return 0;
    if (!strcmp(_clname, qt_meta_stringdata_Interpreter))
        return static_cast<void*>(const_cast< Interpreter*>(this));
    return QThread::qt_metacast(_clname);
}

int Interpreter::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = QThread::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    if (_c == QMetaObject::InvokeMetaMethod) {
        if (_id < 17)
            qt_static_metacall(this, _c, _id, _a);
        _id -= 17;
    }
    return _id;
}

// SIGNAL 0
void Interpreter::runState(int _t1)
{
    void *_a[] = { 0, const_cast<void*>(reinterpret_cast<const void*>(&_t1)) };
    QMetaObject::activate(this, &staticMetaObject, 0, _a);
}

// SIGNAL 1
void Interpreter::textOut(QString _t1, QColor _t2)
{
    void *_a[] = { 0, const_cast<void*>(reinterpret_cast<const void*>(&_t1)), const_cast<void*>(reinterpret_cast<const void*>(&_t2)) };
    QMetaObject::activate(this, &staticMetaObject, 1, _a);
}

// SIGNAL 3
void Interpreter::error(QString _t1)
{
    void *_a[] = { 0, const_cast<void*>(reinterpret_cast<const void*>(&_t1)) };
    QMetaObject::activate(this, &staticMetaObject, 3, _a);
}

// SIGNAL 4
void Interpreter::consoleCommand(QString _t1)
{
    void *_a[] = { 0, const_cast<void*>(reinterpret_cast<const void*>(&_t1)) };
    QMetaObject::activate(this, &staticMetaObject, 4, _a);
}

// SIGNAL 5
void Interpreter::prompt(QString _t1)
{
    void *_a[] = { 0, const_cast<void*>(reinterpret_cast<const void*>(&_t1)) };
    QMetaObject::activate(this, &staticMetaObject, 5, _a);
}

// SIGNAL 6
void Interpreter::videoInput(VideoWidget::InputMode _t1)
{
    void *_a[] = { 0, const_cast<void*>(reinterpret_cast<const void*>(&_t1)) };
    QMetaObject::activate(this, &staticMetaObject, 6, _a);
}

// SIGNAL 7
void Interpreter::enableConsole(bool _t1)
{
    void *_a[] = { 0, const_cast<void*>(reinterpret_cast<const void*>(&_t1)) };
    QMetaObject::activate(this, &staticMetaObject, 7, _a);
}

// SIGNAL 8
void Interpreter::connected(Device _t1, bool _t2)
{
    void *_a[] = { 0, const_cast<void*>(reinterpret_cast<const void*>(&_t1)), const_cast<void*>(reinterpret_cast<const void*>(&_t2)) };
    QMetaObject::activate(this, &staticMetaObject, 8, _a);
}

// SIGNAL 9
void Interpreter::actionScriptlet(int _t1, QString _t2, QStringList _t3)
{
    void *_a[] = { 0, const_cast<void*>(reinterpret_cast<const void*>(&_t1)), const_cast<void*>(reinterpret_cast<const void*>(&_t2)), const_cast<void*>(reinterpret_cast<const void*>(&_t3)) };
    QMetaObject::activate(this, &staticMetaObject, 9, _a);
}

// SIGNAL 10
void Interpreter::actionScriptlet(QString _t1, QStringList _t2)
{
    void *_a[] = { 0, const_cast<void*>(reinterpret_cast<const void*>(&_t1)), const_cast<void*>(reinterpret_cast<const void*>(&_t2)) };
    QMetaObject::activate(this, &staticMetaObject, 10, _a);
}

// SIGNAL 11
void Interpreter::parameter(QString _t1, QByteArray _t2)
{
    void *_a[] = { 0, const_cast<void*>(reinterpret_cast<const void*>(&_t1)), const_cast<void*>(reinterpret_cast<const void*>(&_t2)) };
    QMetaObject::activate(this, &staticMetaObject, 11, _a);
}

// SIGNAL 12
void Interpreter::paramLoaded()
{
    QMetaObject::activate(this, &staticMetaObject, 12, 0);
}

// SIGNAL 13
void Interpreter::version(ushort _t1, ushort _t2, ushort _t3, QString _t4)
{
    void *_a[] = { 0, const_cast<void*>(reinterpret_cast<const void*>(&_t1)), const_cast<void*>(reinterpret_cast<const void*>(&_t2)), const_cast<void*>(reinterpret_cast<const void*>(&_t3)), const_cast<void*>(reinterpret_cast<const void*>(&_t4)) };
    QMetaObject::activate(this, &staticMetaObject, 13, _a);
}
QT_END_MOC_NAMESPACE
