#include "EventWrapper.h"

#include <QtWidgets/QFileDialog>
#include <QtWidgets/QApplication>	  

#include "Render/OgreEnv.h"
#include "FrameEvent/FloatBarEvent.h"
#include "FrameEvent/ToolBarEvent.h"

#include "DOM/DocumentMgr.h"
#include "DOM/IDOM.h"
#include "DOM/Document.h"
#include "ROM/DocumentROM.h"

#include "Render/OgreEnv.h"

#include "DataWrapper.h"
#include "DataModel.h"

#include <chrono>

#include <boost/format.hpp>
#include <boost/filesystem/path.hpp>

EventWrapper* EventWrapper::m_instance = nullptr;

EventWrapper* EventWrapper::getInstance()
{
	if ( m_instance == nullptr )
		m_instance = new EventWrapper();
	return m_instance;
}


QObject* EventWrapper::instance(QQmlEngine* engine,
								QJSEngine* scriptEngine)
{
	Q_UNUSED(engine)
		Q_UNUSED(scriptEngine)

		if ( m_instance == nullptr )
			m_instance = new EventWrapper();
	return m_instance;
}


EventWrapper::EventWrapper(QObject* parent)
	: QObject(parent)
	, m_openDir("")
{
	qDebug("init eventWrapper");
}


EventWrapper::~EventWrapper()
{}
 

bool EventWrapper::openFileDlg()
{
	QString fileName = QFileDialog::getOpenFileName(NULL, tr("Open File"), m_openDir.c_str(), tr("YZM (*.yzm);;Mesh (*.mesh)"));						//.ply .mesh
	if ( fileName.size() == 0 )
	{
		return false;
	}
	auto str = fileName.toStdString();
	auto pos = str.rfind("/");
	if (pos == std::string::npos)
		pos = str.rfind("\\");
	if (pos != std::string::npos)
		m_openDir = str.substr(0, pos);

	if ( DocumentMgr::GetInstance().GetActiveDocument() )
	{
		DocumentMgr::GetInstance().CloseDocument(DocumentMgr::GetInstance().GetActiveDocument());
	}

	auto stdFileName = fileName.toStdWString();
	boost::filesystem::path pt(stdFileName);

	DocumentSPtr newDoc;
	if ( pt.extension() == ".mesh" )
	{
		newDoc = DocumentMgr::GetInstance().ImportOgreMesh(stdFileName);
	}
	else if ( pt.extension() == ".yzm" )
	{
		newDoc = DocumentMgr::GetInstance().OpenDocument(stdFileName);
	}

	if ( newDoc )
	{
		DocumentMgr::GetInstance().SetActiveDocument(newDoc);

		for ( auto& curDom : newDoc->GetDOMList() )
		{
			createItemFromDOM(curDom, true, true);
		}

		newDoc->GetListener().OnCreateDOM.connect([this](const IDOMSPtr& dom)
		{
			createItemFromDOM(dom, true, false);
		});

		newDoc->GetListener().OnRemoveDOM.connect([this](const IDOMSPtr& dom)
		{
			DataModel::getInstance()->removeFromDOM(dom);
		});

		DocumentROM::Create(newDoc, OgreEnv::GetInstance().GetGlobalSmgr());
	}
	else
	{
		//TODO
	}

	return true;
}


void EventWrapper::createItemFromDOM(IDOMSPtr dom, bool regSignal, bool saved)
{
	dom->GetListener().OnChangeUIValue.connect([](const std::wstring& name, float val, uint32_t lastEdittime)
	{
		if ( val > 0 )
		{
			emit EventWrapper::getInstance()->updateValue(val);	 //通知ui
		}
	});

	if ( regSignal )
	{
		std::weak_ptr<IDOM> wpDom = dom;
		dom->GetListener().OnChangeState.connect([wpDom, saved](IDOM::EDOMState state, bool val)
		{
			auto spDOM = wpDom.lock();
			if ( !spDOM )
			{
				return;
			}

			if ( state == IDOM::EDS_Editable && val )
			{
				DataModel::getInstance()->setSavedFromDOM(spDOM);
			}
			else if ( state == IDOM::EDS_Delete )
			{
				if ( val )
				{
					DataModel::getInstance()->removeFromDOM(spDOM);
				}
				else
				{
					EventWrapper::getInstance()->createItemFromDOM(spDOM, false, saved);
				}
			}
		});
	}

	SFE_EditItem sfe;
	sfe.DOM = dom;
	switch ( dom->GetType() )
	{
	case IDOM::EDT_Height:
	sfe.Type = SFE_EditItem::EIT_Height;
	break;
	case IDOM::EDT_LineLength:
	sfe.Type = SFE_EditItem::EIT_LineLength;
	break;
	case IDOM::EDT_CurveLength:
	sfe.Type = SFE_EditItem::EIT_CurveLength;
	break;
	case IDOM::EDT_Dimension:
	sfe.Type = SFE_EditItem::EIT_Dimension;
	break;
	case IDOM::EDT_ConvexDimension:
	sfe.Type = SFE_EditItem::EIT_ConvexDimension;
	break;
	default:
	break;
	}
	sfe.Status = SFE_EditItem::ES_CreateMode;
	OgreEnv::GetInstance().PostFrameEventTo3D(sfe.ConvertToFrameEvent());

	//创建一个newdata，并设为当前data
	DataWrapper* data = NULL;
	switch ( dom->GetType() )
	{
	case IDOM::EDT_Height:
	data = new DataWrapper(dom, "res/icon_adddata_height_n.png", "res/icon_adddata_height_s.png", !saved, saved);
	break;
	case IDOM::EDT_LineLength:
	data = new DataWrapper(dom, "res/icon_adddata_length_n.png", "res/icon_adddata_length_s.png", !saved, saved);
	break;
	case IDOM::EDT_CurveLength:
	data = new DataWrapper(dom, "res/icon_adddata_facelength_n.png", "res/icon_adddata_facelength_s.png", !saved, saved);
	break;
	case IDOM::EDT_Dimension:
	data = new DataWrapper(dom, "res/icon_adddata_facedim_n.png", "res/icon_adddata_facedim_s.png", !saved, saved);
	break;
	case IDOM::EDT_ConvexDimension:
	data = new DataWrapper(dom, "res/icon_adddata_dim_n.png", "res/icon_adddata_dim_s.png", !saved, saved);
	break;
	default:
	break;
	}

	data->setName(QString::fromStdWString(dom->GetName()));
	data->setSaved(saved);

	if ( !saved )
	{
		DataModel::getInstance()->addnewData(data);
	}
	else
	{
		DataModel::getInstance()->addData(data);
	}

	DataModel::getInstance()->setCurrDataByUI(data);
}


void EventWrapper::createItem(int index)
{
	auto curDocDOM = DocumentMgr::GetInstance().GetActiveDocument();
	IDOMSPtr curDOM;
	switch ( index )
	{
	case 0:
	curDOM = curDocDOM->CreateDOM(IDOM::EDT_Height);
	break;
	case 1:
	curDOM = curDocDOM->CreateDOM(IDOM::EDT_LineLength);
	break;
	case 2:
	curDOM = curDocDOM->CreateDOM(IDOM::EDT_CurveLength);
	break;
	case 3:
	curDOM = curDocDOM->CreateDOM(IDOM::EDT_Dimension);
	break;
	case 4:
	curDOM = curDocDOM->CreateDOM(IDOM::EDT_ConvexDimension);
	break;
	default:
	break;
	}
}


void EventWrapper::activeEditMode(int index)
{
	//to3D
	SFE_EditItem sfe;
	sfe.Status = SFE_EditItem::ES_EditMode;
	switch ( index )
	{
	case 0:
	sfe.Type = SFE_EditItem::EIT_Height;
	break;
	case 1:
	sfe.Type = SFE_EditItem::EIT_LineLength;
	break;
	case 2:
	sfe.Type = SFE_EditItem::EIT_CurveLength;
	break;
	case 3:
	sfe.Type = SFE_EditItem::EIT_Dimension;
	break;
	case 4:
	sfe.Type = SFE_EditItem::EIT_ConvexDimension;
	break;
	default:
	break;
	}
	sfe.DOM = DataModel::getInstance()->currData()->DOM();
	OgreEnv::GetInstance().PostFrameEventTo3D(sfe.ConvertToFrameEvent());
}


void EventWrapper::saveCurrData()
{
	// TO3D																									  
	//通知3d进入浏览模式
	SFE_SDC sfe;
	sfe.To3DState = SFE_SDC::ES_Save;
	OgreEnv::GetInstance().PostFrameEventTo3D(sfe.ConvertToFrameEvent());
	//ToUI
	//emit updateEditTime();
}


void EventWrapper::saveAllData(const QString& filename)
{
	//序列化
	// TO3D
	//通知3d进入浏览模式
	SFE_SDC sfe;
	sfe.To3DState = SFE_SDC::ES_Save;
	OgreEnv::GetInstance().PostFrameEventTo3D(sfe.ConvertToFrameEvent());
	//toUI
	while ( DataModel::getInstance()->getNewDataList().size() )
	{
		DataWrapper* dw = DataModel::getInstance()->getNewDataList().at(0);

		dw->setNeworOld(false);		//old:false,new:true
		dw->setSaved(true);
		DataModel::getInstance()->setCurrDataByUI(dw);
		//emit updateEditTime();		
		DataModel::getInstance()->addData(dw);
		DataModel::getInstance()->removenewData(dw);
	}
	emit DataModel::getInstance()->currDataChanged();

	DocumentMgr::GetInstance().GetActiveDocument()->SaveToFile(filename.toStdWString());

	DataModel::getInstance()->setDirty(false);
	emit saveAlldataOver();
}


Q_INVOKABLE void EventWrapper::saveFile()
{
	auto curDoc = DocumentMgr::GetInstance().GetActiveDocument();
	auto fp = curDoc->GetFilePath();
	if ( fp.empty() )
	{
		auto now = std::chrono::system_clock::now();
		auto t = std::chrono::system_clock::to_time_t(now);
		auto tm = std::localtime(&t);

		boost::format fmt("%d%02d%02d%02d%02d.yzm");
		fmt % ( tm->tm_year + 1900 ) % ( tm->tm_mon + 1 ) % tm->tm_mday % tm->tm_hour % tm->tm_min;
		QString fileName = QFileDialog::getSaveFileName(NULL, tr("Save File"), QString::fromStdString(fmt.str()), tr("yzm(*.yzm)"));//.csv
		fp = fileName.toStdWString();
	}

	saveAllData(QString::fromStdWString(fp));
}

bool EventWrapper::saveasAllData()
{
	QString fileName = QFileDialog::getSaveFileName(NULL, tr("Save File"), "", tr("yzm(*.yzm)"));
	if ( fileName.size() == 0 )
	{
		return false;
	}

	if ( DocumentMgr::GetInstance().GetActiveDocument() )
	{
		auto stdFileName = fileName.toStdWString();
		saveAllData(fileName);
	}
	return true;
}


bool EventWrapper::exportData()
{
	auto now = std::chrono::system_clock::now();
	auto t = std::chrono::system_clock::to_time_t(now);
	auto tm = std::localtime(&t);

	boost::format fmt("%d%02d%02d%02d%02d.csv");
	fmt % ( tm->tm_year + 1900 ) % ( tm->tm_mon + 1 ) % tm->tm_mday % tm->tm_hour % tm->tm_min;
	QString fileName = QFileDialog::getSaveFileName(NULL, tr("Save File"), QString::fromStdString(fmt.str()), tr("csv(*.csv)"));//.csv
	if ( fileName.isEmpty() )
	{
		return false;
	}
	if ( DocumentMgr::GetInstance().GetActiveDocument() )
	{
		auto stdFileName = fileName.toStdWString();
		DocumentMgr::GetInstance().GetActiveDocument()->ExportCSVFile(stdFileName);
	}
	return true;
}


void EventWrapper::removeCurrData()
{
	//TO3D
	SFE_SDC sfe;
	sfe.To3DState = SFE_SDC::ES_Delete;
	OgreEnv::GetInstance().PostFrameEventTo3D(sfe.ConvertToFrameEvent());

	auto dom = DataModel::getInstance()->currData()->DOM();
	dom->GetDocument()->RemoveDOM(dom);
}


void EventWrapper::cancelCurrData()
{
	//to3D
	SFE_SDC sfe;
	sfe.To3DState = SFE_SDC::ES_Cancel;
	OgreEnv::GetInstance().PostFrameEventTo3D(sfe.ConvertToFrameEvent());
	//toUI
	emit DataModel::getInstance()->clickCanceled();
}


void EventWrapper::resetCamera()
{
	//to3D
	SFE_ResetCamera sfe;
	OgreEnv::GetInstance().PostFrameEventTo3D(sfe.ConvertToFrameEvent());
	//toUI
}


void EventWrapper::showAllData()
{
	//to3D
	SFE_ShowAllData sfe;
	OgreEnv::GetInstance().PostFrameEventTo3D(sfe.ConvertToFrameEvent());
	//toUI
}

void EventWrapper::editFeaturePoints()
{
	//to3D
	SFE_EditFeaturePoints sfe;
	sfe.conState = SFE_EditFeaturePoints::E_Show;
	sfe.DOM = DocumentMgr::GetInstance().GetActiveDocument();
	OgreEnv::GetInstance().PostFrameEventTo3D(sfe.ConvertToFrameEvent());
}

void EventWrapper::exitFeaturePoints()
{
	//to3D
	SFE_EditFeaturePoints sfe;
	sfe.conState = SFE_EditFeaturePoints::E_Hide;
	sfe.DOM = DocumentMgr::GetInstance().GetActiveDocument();
	OgreEnv::GetInstance().PostFrameEventTo3D(sfe.ConvertToFrameEvent());
}

#include "moc_EventWrapper.cpp"