#pragma once

#include <QtCore/QObject>
#include <QtQml/QQmlEngine>

#include "DOM/DocumentFwd.h"
#include "DOM/IDOMFwd.h"

class EventWrapper : public QObject
{
	Q_OBJECT 
public:
	static EventWrapper* getInstance();
	static QObject* instance(QQmlEngine* engine, QJSEngine* scriptEngine);

	Q_INVOKABLE	bool openFileDlg();

	void createItemFromDOM(IDOMSPtr dom, bool regSignal, bool saved);
	Q_INVOKABLE void createItem(int index);
	Q_INVOKABLE void activeEditMode(int index);

	Q_INVOKABLE void saveCurrData();
	Q_INVOKABLE void saveAllData(const QString& filename);
	Q_INVOKABLE void saveFile();
	Q_INVOKABLE bool saveasAllData();
	Q_INVOKABLE bool exportData();

	Q_INVOKABLE void removeCurrData();
	Q_INVOKABLE void cancelCurrData();

	Q_INVOKABLE void resetCamera();
	Q_INVOKABLE void showAllData();
	Q_INVOKABLE void editFeaturePoints();
	Q_INVOKABLE void exitFeaturePoints();

signals:
	void loadSuccess();																					 
	void updateValue(float val);

	void updateEditTime();

	void selectedDOM(QString name);

	/*
	*@[parm] num:	0 都不高亮
	*				num & 0x1 高亮Btn1
	*				num & 2^N 高亮BtnN
	*/
	void updateToolBarState(int num);

	/*
	*@[parm] num:	0 都不高亮
	*				num & 0x1 高亮Btn1
	*				num & 2^N 高亮BtnN
	*/
	void updateFloatBarState(int num);

	/*
	*@[parm] num:	0 不高亮, 1高亮
	*/
	void updateFeaturepointsState(int num);

	/*
	*[parm] isDirty:	当前数据是否被污染(即是否需要保存)
	*/
	void dirtyChanged(bool isDirty);

	/*
	*@ 通知ui所有数据已经保存完毕
	*/
	void saveAlldataOver();

private:
	EventWrapper(QObject* parent = nullptr);
	~EventWrapper();

private:
	std::string m_openDir;
	static EventWrapper* m_instance;
};

static EventWrapper* getEventWrapper() { return EventWrapper::getInstance(); }
