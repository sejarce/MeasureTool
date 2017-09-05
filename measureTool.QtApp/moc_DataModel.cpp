/****************************************************************************
** Meta object code from reading C++ file 'DataModel.h'
**
** Created by: The Qt Meta Object Compiler version 67 (Qt 5.7.0)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include "DataModel.h"
#include <QtCore/qbytearray.h>
#include <QtCore/qmetatype.h>
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'DataModel.h' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 67
#error "This file was generated using the moc from 5.7.0. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

QT_BEGIN_MOC_NAMESPACE
struct qt_meta_stringdata_DataModel_t {
    QByteArrayData data[13];
    char stringdata0[153];
};
#define QT_MOC_LITERAL(idx, ofs, len) \
    Q_STATIC_BYTE_ARRAY_DATA_HEADER_INITIALIZER_WITH_OFFSET(len, \
    qptrdiff(offsetof(qt_meta_stringdata_DataModel_t, stringdata0) + ofs \
        - idx * sizeof(QByteArrayData)) \
    )
static const qt_meta_stringdata_DataModel_t qt_meta_stringdata_DataModel = {
    {
QT_MOC_LITERAL(0, 0, 9), // "DataModel"
QT_MOC_LITERAL(1, 10, 9), // "dataReady"
QT_MOC_LITERAL(2, 20, 0), // ""
QT_MOC_LITERAL(3, 21, 15), // "currDataChanged"
QT_MOC_LITERAL(4, 37, 13), // "clickCanceled"
QT_MOC_LITERAL(5, 51, 15), // "setCurrDataByUI"
QT_MOC_LITERAL(6, 67, 12), // "DataWrapper*"
QT_MOC_LITERAL(7, 80, 4), // "data"
QT_MOC_LITERAL(8, 85, 8), // "dataList"
QT_MOC_LITERAL(9, 94, 29), // "QQmlListProperty<DataWrapper>"
QT_MOC_LITERAL(10, 124, 11), // "newDataList"
QT_MOC_LITERAL(11, 136, 8), // "currData"
QT_MOC_LITERAL(12, 145, 7) // "isDirty"

    },
    "DataModel\0dataReady\0\0currDataChanged\0"
    "clickCanceled\0setCurrDataByUI\0"
    "DataWrapper*\0data\0dataList\0"
    "QQmlListProperty<DataWrapper>\0newDataList\0"
    "currData\0isDirty"
};
#undef QT_MOC_LITERAL

static const uint qt_meta_data_DataModel[] = {

 // content:
       7,       // revision
       0,       // classname
       0,    0, // classinfo
       4,   14, // methods
       4,   40, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       3,       // signalCount

 // signals: name, argc, parameters, tag, flags
       1,    0,   34,    2, 0x06 /* Public */,
       3,    0,   35,    2, 0x06 /* Public */,
       4,    0,   36,    2, 0x06 /* Public */,

 // methods: name, argc, parameters, tag, flags
       5,    1,   37,    2, 0x02 /* Public */,

 // signals: parameters
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,

 // methods: parameters
    QMetaType::Void, 0x80000000 | 6,    7,

 // properties: name, type, flags
       8, 0x80000000 | 9, 0x00095009,
      10, 0x80000000 | 9, 0x00095009,
      11, 0x80000000 | 6, 0x0009500b,
      12, QMetaType::Bool, 0x00095001,

       0        // eod
};

void DataModel::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        DataModel *_t = static_cast<DataModel *>(_o);
        Q_UNUSED(_t)
        switch (_id) {
        case 0: _t->dataReady(); break;
        case 1: _t->currDataChanged(); break;
        case 2: _t->clickCanceled(); break;
        case 3: _t->setCurrDataByUI((*reinterpret_cast< DataWrapper*(*)>(_a[1]))); break;
        default: ;
        }
    } else if (_c == QMetaObject::IndexOfMethod) {
        int *result = reinterpret_cast<int *>(_a[0]);
        void **func = reinterpret_cast<void **>(_a[1]);
        {
            typedef void (DataModel::*_t)();
            if (*reinterpret_cast<_t *>(func) == static_cast<_t>(&DataModel::dataReady)) {
                *result = 0;
                return;
            }
        }
        {
            typedef void (DataModel::*_t)();
            if (*reinterpret_cast<_t *>(func) == static_cast<_t>(&DataModel::currDataChanged)) {
                *result = 1;
                return;
            }
        }
        {
            typedef void (DataModel::*_t)();
            if (*reinterpret_cast<_t *>(func) == static_cast<_t>(&DataModel::clickCanceled)) {
                *result = 2;
                return;
            }
        }
    }
#ifndef QT_NO_PROPERTIES
    else if (_c == QMetaObject::ReadProperty) {
        DataModel *_t = static_cast<DataModel *>(_o);
        Q_UNUSED(_t)
        void *_v = _a[0];
        switch (_id) {
        case 0: *reinterpret_cast< QQmlListProperty<DataWrapper>*>(_v) = _t->dataList(); break;
        case 1: *reinterpret_cast< QQmlListProperty<DataWrapper>*>(_v) = _t->newDataList(); break;
        case 2: *reinterpret_cast< DataWrapper**>(_v) = _t->currData(); break;
        case 3: *reinterpret_cast< bool*>(_v) = _t->isDirty(); break;
        default: break;
        }
    } else if (_c == QMetaObject::WriteProperty) {
        DataModel *_t = static_cast<DataModel *>(_o);
        Q_UNUSED(_t)
        void *_v = _a[0];
        switch (_id) {
        case 2: _t->setCurrDataByUI(*reinterpret_cast< DataWrapper**>(_v)); break;
        default: break;
        }
    } else if (_c == QMetaObject::ResetProperty) {
    }
#endif // QT_NO_PROPERTIES
}

const QMetaObject DataModel::staticMetaObject = {
    { &QObject::staticMetaObject, qt_meta_stringdata_DataModel.data,
      qt_meta_data_DataModel,  qt_static_metacall, Q_NULLPTR, Q_NULLPTR}
};


const QMetaObject *DataModel::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->dynamicMetaObject() : &staticMetaObject;
}

void *DataModel::qt_metacast(const char *_clname)
{
    if (!_clname) return Q_NULLPTR;
    if (!strcmp(_clname, qt_meta_stringdata_DataModel.stringdata0))
        return static_cast<void*>(const_cast< DataModel*>(this));
    return QObject::qt_metacast(_clname);
}

int DataModel::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = QObject::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    if (_c == QMetaObject::InvokeMetaMethod) {
        if (_id < 4)
            qt_static_metacall(this, _c, _id, _a);
        _id -= 4;
    } else if (_c == QMetaObject::RegisterMethodArgumentMetaType) {
        if (_id < 4)
            *reinterpret_cast<int*>(_a[0]) = -1;
        _id -= 4;
    }
#ifndef QT_NO_PROPERTIES
   else if (_c == QMetaObject::ReadProperty || _c == QMetaObject::WriteProperty
            || _c == QMetaObject::ResetProperty || _c == QMetaObject::RegisterPropertyMetaType) {
        qt_static_metacall(this, _c, _id, _a);
        _id -= 4;
    } else if (_c == QMetaObject::QueryPropertyDesignable) {
        _id -= 4;
    } else if (_c == QMetaObject::QueryPropertyScriptable) {
        _id -= 4;
    } else if (_c == QMetaObject::QueryPropertyStored) {
        _id -= 4;
    } else if (_c == QMetaObject::QueryPropertyEditable) {
        _id -= 4;
    } else if (_c == QMetaObject::QueryPropertyUser) {
        _id -= 4;
    }
#endif // QT_NO_PROPERTIES
    return _id;
}

// SIGNAL 0
void DataModel::dataReady()
{
    QMetaObject::activate(this, &staticMetaObject, 0, Q_NULLPTR);
}

// SIGNAL 1
void DataModel::currDataChanged()
{
    QMetaObject::activate(this, &staticMetaObject, 1, Q_NULLPTR);
}

// SIGNAL 2
void DataModel::clickCanceled()
{
    QMetaObject::activate(this, &staticMetaObject, 2, Q_NULLPTR);
}
QT_END_MOC_NAMESPACE
