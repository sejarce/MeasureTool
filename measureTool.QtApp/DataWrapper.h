#pragma once  

#include "QtCore/QObject"
#include "QtCore/QString"

#include "DOM/DocumentFwd.h"
#include "DOM/IDOMFwd.h"

class DataWrapper : public QObject
{
	Q_OBJECT
		Q_PROPERTY(QString name READ name WRITE setName)
		Q_PROPERTY(float value READ value)
		Q_PROPERTY(QString type READ type)
		Q_PROPERTY(QString editTime READ editTime)
		Q_PROPERTY(QString image_n READ image_n)
		Q_PROPERTY(QString image_s READ image_s)
		Q_PROPERTY(bool neworOld READ neworOld WRITE setNeworOld)
		Q_PROPERTY(bool saved READ saved WRITE setSaved)
		Q_PROPERTY(bool isCreate READ isCreate)
		Q_PROPERTY(bool isDirty READ isDirty)
public:

	DataWrapper(QObject* parent = nullptr);

	DataWrapper(IDOMSPtr dom, QString image_n, QString image_s, bool neworold, bool saved = false);

	~DataWrapper();

	QString name();
	Q_INVOKABLE void setName(QString name);
	float value();
	QString type();
	QString editTime();
	QString image_n();
	QString image_s();
	bool isCreate() const;
	bool isDirty() const;
	bool neworOld();
	void setNeworOld(bool flag);
	bool saved();
	Q_INVOKABLE void setSaved(bool flag);
	IDOMSPtr DOM();
	void setDOM(IDOMSPtr dom);

protected:
	QString m_image_n;
	QString m_image_s;
	bool	m_NeworOld;

	std::weak_ptr<IDOM>	m_DOM;
};