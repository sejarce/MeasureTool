/****************************************************************************
** Meta object code from reading C++ file 'DataWrapper.h'
**
** Created by: The Qt Meta Object Compiler version 67 (Qt 5.7.0)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include "DataWrapper.h"
#include <QtCore/qbytearray.h>
#include <QtCore/qmetatype.h>
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'DataWrapper.h' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 67
#error "This file was generated using the moc from 5.7.0. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

QT_BEGIN_MOC_NAMESPACE
struct qt_meta_stringdata_DataWrapper_t {
    QByteArrayData data[15];
    char stringdata0[108];
};
#define QT_MOC_LITERAL(idx, ofs, len) \
    Q_STATIC_BYTE_ARRAY_DATA_HEADER_INITIALIZER_WITH_OFFSET(len, \
    qptrdiff(offsetof(qt_meta_stringdata_DataWrapper_t, stringdata0) + ofs \
        - idx * sizeof(QByteArrayData)) \
    )
static const qt_meta_stringdata_DataWrapper_t qt_meta_stringdata_DataWrapper = {
    {
QT_MOC_LITERAL(0, 0, 11), // "DataWrapper"
QT_MOC_LITERAL(1, 12, 7), // "setName"
QT_MOC_LITERAL(2, 20, 0), // ""
QT_MOC_LITERAL(3, 21, 4), // "name"
QT_MOC_LITERAL(4, 26, 8), // "setSaved"
QT_MOC_LITERAL(5, 35, 4), // "flag"
QT_MOC_LITERAL(6, 40, 5), // "value"
QT_MOC_LITERAL(7, 46, 4), // "type"
QT_MOC_LITERAL(8, 51, 8), // "editTime"
QT_MOC_LITERAL(9, 60, 7), // "image_n"
QT_MOC_LITERAL(10, 68, 7), // "image_s"
QT_MOC_LITERAL(11, 76, 8), // "neworOld"
QT_MOC_LITERAL(12, 85, 5), // "saved"
QT_MOC_LITERAL(13, 91, 8), // "isCreate"
QT_MOC_LITERAL(14, 100, 7) // "isDirty"

    },
    "DataWrapper\0setName\0\0name\0setSaved\0"
    "flag\0value\0type\0editTime\0image_n\0"
    "image_s\0neworOld\0saved\0isCreate\0isDirty"
};
#undef QT_MOC_LITERAL

static const uint qt_meta_data_DataWrapper[] = {

 // content:
       7,       // revision
       0,       // classname
       0,    0, // classinfo
       2,   14, // methods
      10,   30, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       0,       // signalCount

 // methods: name, argc, parameters, tag, flags
       1,    1,   24,    2, 0x02 /* Public */,
       4,    1,   27,    2, 0x02 /* Public */,

 // methods: parameters
    QMetaType::Void, QMetaType::QString,    3,
    QMetaType::Void, QMetaType::Bool,    5,

 // properties: name, type, flags
       3, QMetaType::QString, 0x00095103,
       6, QMetaType::Float, 0x00095001,
       7, QMetaType::QString, 0x00095001,
       8, QMetaType::QString, 0x00095001,
       9, QMetaType::QString, 0x00095001,
      10, QMetaType::QString, 0x00095001,
      11, QMetaType::Bool, 0x00095103,
      12, QMetaType::Bool, 0x00095103,
      13, QMetaType::Bool, 0x00095001,
      14, QMetaType::Bool, 0x00095001,

       0        // eod
};

void DataWrapper::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        DataWrapper *_t = static_cast<DataWrapper *>(_o);
        Q_UNUSED(_t)
        switch (_id) {
        case 0: _t->setName((*reinterpret_cast< QString(*)>(_a[1]))); break;
        case 1: _t->setSaved((*reinterpret_cast< bool(*)>(_a[1]))); break;
        default: ;
        }
    }
#ifndef QT_NO_PROPERTIES
    else if (_c == QMetaObject::ReadProperty) {
        DataWrapper *_t = static_cast<DataWrapper *>(_o);
        Q_UNUSED(_t)
        void *_v = _a[0];
        switch (_id) {
        case 0: *reinterpret_cast< QString*>(_v) = _t->name(); break;
        case 1: *reinterpret_cast< float*>(_v) = _t->value(); break;
        case 2: *reinterpret_cast< QString*>(_v) = _t->type(); break;
        case 3: *reinterpret_cast< QString*>(_v) = _t->editTime(); break;
        case 4: *reinterpret_cast< QString*>(_v) = _t->image_n(); break;
        case 5: *reinterpret_cast< QString*>(_v) = _t->image_s(); break;
        case 6: *reinterpret_cast< bool*>(_v) = _t->neworOld(); break;
        case 7: *reinterpret_cast< bool*>(_v) = _t->saved(); break;
        case 8: *reinterpret_cast< bool*>(_v) = _t->isCreate(); break;
        case 9: *reinterpret_cast< bool*>(_v) = _t->isDirty(); break;
        default: break;
        }
    } else if (_c == QMetaObject::WriteProperty) {
        DataWrapper *_t = static_cast<DataWrapper *>(_o);
        Q_UNUSED(_t)
        void *_v = _a[0];
        switch (_id) {
        case 0: _t->setName(*reinterpret_cast< QString*>(_v)); break;
        case 6: _t->setNeworOld(*reinterpret_cast< bool*>(_v)); break;
        case 7: _t->setSaved(*reinterpret_cast< bool*>(_v)); break;
        default: break;
        }
    } else if (_c == QMetaObject::ResetProperty) {
    }
#endif // QT_NO_PROPERTIES
}

const QMetaObject DataWrapper::staticMetaObject = {
    { &QObject::staticMetaObject, qt_meta_stringdata_DataWrapper.data,
      qt_meta_data_DataWrapper,  qt_static_metacall, Q_NULLPTR, Q_NULLPTR}
};


const QMetaObject *DataWrapper::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->dynamicMetaObject() : &staticMetaObject;
}

void *DataWrapper::qt_metacast(const char *_clname)
{
    if (!_clname) return Q_NULLPTR;
    if (!strcmp(_clname, qt_meta_stringdata_DataWrapper.stringdata0))
        return static_cast<void*>(const_cast< DataWrapper*>(this));
    return QObject::qt_metacast(_clname);
}

int DataWrapper::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = QObject::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    if (_c == QMetaObject::InvokeMetaMethod) {
        if (_id < 2)
            qt_static_metacall(this, _c, _id, _a);
        _id -= 2;
    } else if (_c == QMetaObject::RegisterMethodArgumentMetaType) {
        if (_id < 2)
            *reinterpret_cast<int*>(_a[0]) = -1;
        _id -= 2;
    }
#ifndef QT_NO_PROPERTIES
   else if (_c == QMetaObject::ReadProperty || _c == QMetaObject::WriteProperty
            || _c == QMetaObject::ResetProperty || _c == QMetaObject::RegisterPropertyMetaType) {
        qt_static_metacall(this, _c, _id, _a);
        _id -= 10;
    } else if (_c == QMetaObject::QueryPropertyDesignable) {
        _id -= 10;
    } else if (_c == QMetaObject::QueryPropertyScriptable) {
        _id -= 10;
    } else if (_c == QMetaObject::QueryPropertyStored) {
        _id -= 10;
    } else if (_c == QMetaObject::QueryPropertyEditable) {
        _id -= 10;
    } else if (_c == QMetaObject::QueryPropertyUser) {
        _id -= 10;
    }
#endif // QT_NO_PROPERTIES
    return _id;
}
QT_END_MOC_NAMESPACE
