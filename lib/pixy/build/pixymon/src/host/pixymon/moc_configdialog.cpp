/****************************************************************************
** Meta object code from reading C++ file 'configdialog.h'
**
** Created by: The Qt Meta Object Compiler version 63 (Qt 4.8.6)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include "configdialog.h"
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'configdialog.h' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 63
#error "This file was generated using the moc from 4.8.6. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

QT_BEGIN_MOC_NAMESPACE
static const uint qt_meta_data_ConfigDialog[] = {

 // content:
       6,       // revision
       0,       // classname
       0,    0, // classinfo
       8,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       0,       // signalCount

 // slots: signature, parameters, type, tag, flags
      13,   20,   20,   20, 0x0a,
      21,   45,   20,   20, 0x09,
      52,   20,   20,   20, 0x09,
      74,   20,   20,   20, 0x09,
      91,  109,   20,   20, 0x09,
     118,  138,   20,   20, 0x09,
     144,   20,   20,   20, 0x09,
     153,   20,   20,   20, 0x09,

       0        // eod
};

static const char qt_meta_stringdata_ConfigDialog[] = {
    "ConfigDialog\0load()\0\0apply(QAbstractButton*)\0"
    "button\0handleChangeClicked()\0"
    "handleCheckBox()\0handleSlider(int)\0"
    "position\0handleComboBox(int)\0index\0"
    "accept()\0reject()\0"
};

void ConfigDialog::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        Q_ASSERT(staticMetaObject.cast(_o));
        ConfigDialog *_t = static_cast<ConfigDialog *>(_o);
        switch (_id) {
        case 0: _t->load(); break;
        case 1: _t->apply((*reinterpret_cast< QAbstractButton*(*)>(_a[1]))); break;
        case 2: _t->handleChangeClicked(); break;
        case 3: _t->handleCheckBox(); break;
        case 4: _t->handleSlider((*reinterpret_cast< int(*)>(_a[1]))); break;
        case 5: _t->handleComboBox((*reinterpret_cast< int(*)>(_a[1]))); break;
        case 6: _t->accept(); break;
        case 7: _t->reject(); break;
        default: ;
        }
    }
}

const QMetaObjectExtraData ConfigDialog::staticMetaObjectExtraData = {
    0,  qt_static_metacall 
};

const QMetaObject ConfigDialog::staticMetaObject = {
    { &QDialog::staticMetaObject, qt_meta_stringdata_ConfigDialog,
      qt_meta_data_ConfigDialog, &staticMetaObjectExtraData }
};

#ifdef Q_NO_DATA_RELOCATION
const QMetaObject &ConfigDialog::getStaticMetaObject() { return staticMetaObject; }
#endif //Q_NO_DATA_RELOCATION

const QMetaObject *ConfigDialog::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->metaObject : &staticMetaObject;
}

void *ConfigDialog::qt_metacast(const char *_clname)
{
    if (!_clname) return 0;
    if (!strcmp(_clname, qt_meta_stringdata_ConfigDialog))
        return static_cast<void*>(const_cast< ConfigDialog*>(this));
    return QDialog::qt_metacast(_clname);
}

int ConfigDialog::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = QDialog::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    if (_c == QMetaObject::InvokeMetaMethod) {
        if (_id < 8)
            qt_static_metacall(this, _c, _id, _a);
        _id -= 8;
    }
    return _id;
}
QT_END_MOC_NAMESPACE
