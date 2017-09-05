#include "DataModel.h"
#include "DataWrapper.h"

#include "Render/OgreEnv.h"
#include "FrameEvent/ToolBarEvent.h"
#include "FrameEvent/ControlEvent.h"

#include "DOM/Document.h"
#include "DOM/IDOM.h"

DataModel* DataModel::m_instance = nullptr;

DataModel* DataModel::getInstance()
{
	if ( m_instance == nullptr )
		m_instance = new DataModel();
	return m_instance;
}

QObject* DataModel::instance(QQmlEngine* engine,
							 QJSEngine* scriptEngine)
{
	Q_UNUSED(engine)
		Q_UNUSED(scriptEngine)

		if ( m_instance == nullptr )
			m_instance = new DataModel();
	return m_instance;
}

DataModel::DataModel(QObject* parent)
	: QObject(parent)
	, m_dirty(false)
{}

DataModel::~DataModel()
{
	for ( int i = 0; i < m_dataList.size(); i++ )
	{
		if ( m_dataList[i] )
		{
			delete m_dataList[i];
			m_dataList[i] = NULL;
		}
	}
	for ( int i = 0; i < m_newDataList.size(); i++ )
	{
		if ( m_newDataList[i] )
		{
			delete m_newDataList[i];
			m_newDataList[i] = NULL;
		}
	}
	m_dataList.clear();
	QList<DataWrapper*>().swap(m_dataList);
	m_newDataList.clear();
	QList<DataWrapper*>().swap(m_newDataList);
}


void DataModel::addData(DataWrapper* data)
{
	m_dataList.append(data);
	emit dataReady();
}


void DataModel::removeData(int index)
{
	m_dataList.erase(m_dataList.begin() + index);
	emit dataReady();
}


void DataModel::removeData(DataWrapper* data)
{
	if ( m_dataList.removeOne(data) )
	{
		//delete data;
		emit dataReady();
	}
}


void DataModel::eraseData(DataWrapper* data)
{
	if ( m_dataList.removeOne(data) )
	{
		delete data;
		emit dataReady();
	}
}


void DataModel::addnewData(DataWrapper* data)
{
	m_newDataList.append(data);
	emit dataReady();
}


void DataModel::removenewData(int index)
{
	m_newDataList.erase(m_newDataList.begin() + index);
	//m_newDataList.removeAt(index);
	emit dataReady();
}



void DataModel::removenewData(DataWrapper* data)
{
	if ( m_newDataList.removeOne(data) )
	{
		//delete data;
		emit dataReady();
	}
}


void DataModel::removeFromDOM(const IDOMSPtr& dom)
{
	auto dataList = getDataList();
	for ( auto cur : dataList )
	{
		if ( dom == cur->DOM() )
		{
			eraseData(cur);
			return;
		}
	}

	auto newDataList = getNewDataList();
	for ( auto cur : newDataList )
	{
		if ( dom == cur->DOM() )
		{
			erasenewData(cur);
			return;
		}
	}

	qDebug("Ops:Can not find DataWrap on DOM");
}

void DataModel::setSavedFromDOM(const IDOMSPtr& dom)
{
	auto dataList = getDataList();
	for ( auto cur : dataList )
	{
		if ( dom == cur->DOM() )
		{
			cur->setSaved(true);
			return;
		}
	}

	auto newDataList = getNewDataList();
	for ( auto cur : newDataList )
	{
		if ( dom == cur->DOM() )
		{
			cur->setSaved(false);
			return;
		}
	}

	qDebug("Ops:Can not find DataWrap on DOM");
}

void DataModel::setCurrData(DataWrapper* data)
{
	if ( m_currData == data )
	{
		return;
	}
	m_currData = data;
	//ToUI
	emit currDataChanged();
}

void DataModel::erasenewData(DataWrapper* data)
{
	if ( m_newDataList.removeOne(data) )
	{
		delete data;
		emit dataReady();
	}
}


const QQmlListProperty<DataWrapper> DataModel::dataList()
{
	return QQmlListProperty<DataWrapper>(this, m_dataList);
}


QList<DataWrapper*> DataModel::getDataList() const
{
	return m_dataList;
}


const QQmlListProperty<DataWrapper> DataModel::newDataList()
{
	return QQmlListProperty<DataWrapper>(this, m_newDataList);
}


QList<DataWrapper*> DataModel::getNewDataList() const
{
	return m_newDataList;
}


DataWrapper* DataModel::currData()
{
	return m_currData;
}


void DataModel::setCurrDataByUI(DataWrapper* data)
{
	setCurrData(data);
	
	//To3D
	//通知3d进入浏览模式
	{
		SFE_SDC sfe;
		sfe.To3DState = SFE_SDC::ES_Save;
		OgreEnv::GetInstance().PostFrameEventTo3D(sfe.ConvertToFrameEvent());
	}

	{
		SFE_Selection sfe;
		sfe.SelectedDOM = data->DOM();
		OgreEnv::GetInstance().PostFrameEventTo3D(sfe.ConvertToFrameEvent());
	}
}

bool DataModel::isDirty() const
{
	return m_dirty;
}

void DataModel::setDirty(bool dirty)
{
	m_dirty = dirty;
}

#include "moc_DataModel.cpp"
