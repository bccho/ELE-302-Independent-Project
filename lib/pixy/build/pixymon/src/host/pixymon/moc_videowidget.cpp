/****************************************************************************
** Meta object code from reading C++ file 'videowidget.h'
**
** Created by: The Qt Meta Object Compiler version 63 (Qt 4.8.6)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include "videowidget.h"
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'videowidget.h' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 63
#error "This file was generated using the moc from 4.8.6. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

QT_BEGIN_MOC_NAMESPACE
static const uint qt_meta_data_VideoWidget[] = {

 // content:
       6,       // revision
       0,       // classname
       0,    0, // classinfo
       3,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       1,       // signalCount

 // signals: signature, parameters, type, tag, flags
      12,   39,   58,   58, 0x05,

 // slots: signature, parameters, type, tag, flags
      59,   85,   58,   58, 0x0a,
     103,  139,   58,   58, 0x0a,

       0        // eod
};

static const char qt_meta_stringdata_VideoWidget[] = {
    "VideoWidget\0selection(int,int,int,int)\0"
    "x0,y0,width,height\0\0handleImage(QImage,uchar)\0"
    "image,renderFlags\0acceptInput(VideoWidget::InputMode)\0"
    "mode\0"
};

void VideoWidget::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        Q_ASSERT(staticMetaObject.cast(_o));
        VideoWidget *_t = static_cast<VideoWidget *>(_o);
        switch (_id) {
        case 0: _t->selection((*reinterpret_cast< int(*)>(_a[1])),(*reinterpret_cast< int(*)>(_a[2])),(*reinterpret_cast< int(*)>(_a[3])),(*reinterpret_cast< int(*)>(_a[4]))); break;
        case 1: _t->handleImage((*reinterpret_cast< QImage(*)>(_a[1])),(*reinterpret_cast< uchar(*)>(_a[2]))); break;
        case 2: _t->acceptInput((*reinterpret_cast< VideoWidget::InputMode(*)>(_a[1]))); break;
        default: ;
        }
    }
}

const QMetaObjectExtraData VideoWidget::staticMetaObjectExtraData = {
    0,  qt_static_metacall 
};

const QMetaObject VideoWidget::staticMetaObject = {
    { &QWidget::staticMetaObject, qt_meta_stringdata_VideoWidget,
      qt_meta_data_VideoWidget, &staticMetaObjectExtraData }
};

#ifdef Q_NO_DATA_RELOCATION
const QMetaObject &VideoWidget::getStaticMetaObject() { return staticMetaObject; }
#endif //Q_NO_DATA_RELOCATION

const QMetaObject *VideoWidget::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->metaObject : &staticMetaObject;
}

void *VideoWidget::qt_metacast(const char *_clname)
{
    if (!_clname) return 0;
    if (!strcmp(_clname, qt_meta_stringdata_VideoWidget))
        return static_cast<void*>(const_cast< VideoWidget*>(this));
    return QWidget::qt_metacast(_clname);
}

int VideoWidget::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = QWidget::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    if (_c == QMetaObject::InvokeMetaMethod) {
        if (_id < 3)
            qt_static_metacall(this, _c, _id, _a);
        _id -= 3;
    }
    return _id;
}

// SIGNAL 0
void VideoWidget::selection(int _t1, int _t2, int _t3, int _t4)
{
    void *_a[] = { 0, const_cast<void*>(reinterpret_cast<const void*>(&_t1)), const_cast<void*>(reinterpret_cast<const void*>(&_t2)), const_cast<void*>(reinterpret_cast<const void*>(&_t3)), const_cast<void*>(reinterpret_cast<const void*>(&_t4)) };
    QMetaObject::activate(this, &staticMetaObject, 0, _a);
}
QT_END_MOC_NAMESPACE
