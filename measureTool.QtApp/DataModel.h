#pragma once  

#include "QtCore/QObject"
#include "QtCore/QString"
#include "QtCore/QList"
#include "QtQml/QQmlEngine"

#include "DOM/DocumentFwd.h"
#include "DOM/IDOMFwd.h"

class	DataWrapper;

class DataModel : public QObject
{
	Q_OBJECT
		Q_PROPERTY(QQmlListProperty<DataWrapper> dataList READ dataList)
		Q_PROPERTY(QQmlListProperty<DataWrapper> newDataList READ newDataList)
		Q_PROPERTY(DataWrapper* currData READ currData WRITE setCurrDataByUI)
		Q_PROPERTY(bool isDirty READ isDirty)
public:
	static DataModel* getInstance();
	static QObject* instance(QQmlEngine* engine, QJSEngine* scriptEngine);

	void addData(DataWrapper* data);
	void removeData(int index);
	void eraseData(DataWrapper* data);
	void removeData(DataWrapper* data);
	void addnewData(DataWrapper* data);
	void removenewData(int index);
	void erasenewData(DataWrapper* data);
	void removenewData(DataWrapper* data);
	void removeFromDOM(const IDOMSPtr& dom);

	void setSavedFromDOM(const IDOMSPtr& dom);

	void setCurrData(DataWrapper* data);

	const QQmlListProperty<DataWrapper> dataList();
	QList<DataWrapper*> getDataList() const;
	const QQmlListProperty<DataWrapper> newDataList();
	QList<DataWrapper*> getNewDataList() const;

	DataWrapper* currData();

	Q_INVOKABLE void setCurrDataByUI(DataWrapper* data);

	bool isDirty() const;

	void setDirty(bool dirty);
signals:
	void dataReady();
	void currDataChanged();
	void clickCanceled();

private:
	DataModel(QObject* parent = nullptr);
	~DataModel();

	DataWrapper* m_currData;
	QList<DataWrapper*> m_dataList;
	QList<DataWrapper*> m_newDataList;
	static DataModel* m_instance;
	bool m_dirty;
};

static DataModel* getDataModel()
{
	return DataModel::getInstance();
}
