/****************************************************************************
** Meta object code from reading C++ file 'connectevent.h'
**
** Created by: The Qt Meta Object Compiler version 63 (Qt 4.8.6)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include "connectevent.h"
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'connectevent.h' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 63
#error "This file was generated using the moc from 4.8.6. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

QT_BEGIN_MOC_NAMESPACE
static const uint qt_meta_data_ConnectEvent[] = {

 // content:
       6,       // revision
       0,       // classname
       0,    0, // classinfo
       1,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       1,       // signalCount

 // signals: signature, parameters, type, tag, flags
      13,   36,   49,   49, 0x05,

       0        // eod
};

static const char qt_meta_stringdata_ConnectEvent[] = {
    "ConnectEvent\0connected(Device,bool)\0"
    "device,state\0\0"
};

void ConnectEvent::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        Q_ASSERT(staticMetaObject.cast(_o));
        ConnectEvent *_t = static_cast<ConnectEvent *>(_o);
        switch (_id) {
        case 0: _t->connected((*reinterpret_cast< Device(*)>(_a[1])),(*reinterpret_cast< bool(*)>(_a[2]))); break;
        default: ;
        }
    }
}

const QMetaObjectExtraData ConnectEvent::staticMetaObjectExtraData = {
    0,  qt_static_metacall 
};

const QMetaObject ConnectEvent::staticMetaObject = {
    { &QThread::staticMetaObject, qt_meta_stringdata_ConnectEvent,
      qt_meta_data_ConnectEvent, &staticMetaObjectExtraData }
};

#ifdef Q_NO_DATA_RELOCATION
const QMetaObject &ConnectEvent::getStaticMetaObject() { return staticMetaObject; }
#endif //Q_NO_DATA_RELOCATION

const QMetaObject *ConnectEvent::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->metaObject : &staticMetaObject;
}

void *ConnectEvent::qt_metacast(const char *_clname)
{
    if (!_clname) return 0;
    if (!strcmp(_clname, qt_meta_stringdata_ConnectEvent))
        return static_cast<void*>(const_cast< ConnectEvent*>(this));
    return QThread::qt_metacast(_clname);
}

int ConnectEvent::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = QThread::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    if (_c == QMetaObject::InvokeMetaMethod) {
        if (_id < 1)
            qt_static_metacall(this, _c, _id, _a);
        _id -= 1;
    }
    return _id;
}

// SIGNAL 0
void ConnectEvent::connected(Device _t1, bool _t2)
{
    void *_a[] = { 0, const_cast<void*>(reinterpret_cast<const void*>(&_t1)), const_cast<void*>(reinterpret_cast<const void*>(&_t2)) };
    QMetaObject::activate(this, &staticMetaObject, 0, _a);
}
QT_END_MOC_NAMESPACE
