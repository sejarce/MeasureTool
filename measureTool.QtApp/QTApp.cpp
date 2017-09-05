#include "QTApp.h"

#include "QtCore/QTimer"
#include "QtCore/QObject"
#include <QtQuick/QQuickView>
#include <QtQuick/QQuickItem>
#include <QtWidgets/QApplication>
#include <QtQml/QQmlContext>
#include <QtWidgets/QDialog>

#include "Render/OgreEnv.h"
#include "Render/OgreWndWrapper.h"

#include "Controller/PrefabControllerMgr.h"
#include "DOM/DocumentMgr.h"
#include "DOM/Document.h"

#include "QT5RenderWindow.h"															 
#include "DataWrapper.h"
#include "DataModel.h"
#include "EventWrapper.h"

#include "FrameEvent/ToolBarEvent.h"
#include "FrameEvent/ControlEvent.h"
#include "FrameEvent/FloatBarEvent.h"

#include <boost/program_options.hpp>

class	QTApp::Imp
{
public:

	using	CmdValues = std::vector<std::string>;
	using	CmdType = std::map<std::string, CmdValues>;

public:

	CmdType								CmdOptions_;

	std::unique_ptr<QApplication>	QApp_;
	std::unique_ptr<QTimer>				Timer_;
	std::unique_ptr<QQuickView>			View_;
};

QTApp::QTApp() :ImpUPtr_(std::make_unique<Imp>())
{

}

QTApp::~QTApp()
{ 

}

QTApp& QTApp::GetInstance()
{
	static QTApp sIns;
	return sIns;
}

void QTApp::Init(int argc, char** argv)
{
	auto& imp_ = *ImpUPtr_;

	OgreEnv::GetInstance().Init();

	{
		boost::program_options::options_description desc;
		auto clp = boost::program_options::command_line_parser(argc, argv).options(desc).allow_unregistered().run();
		for ( auto& cur : clp.options )
		{
			imp_.CmdOptions_.emplace(cur.string_key, cur.value);
		}
	}
	qDebug("ogre init over");
	imp_.QApp_ = std::make_unique<QApplication>(argc, argv);

	imp_.Timer_ = std::make_unique<QTimer>(imp_.QApp_.get());
	QObject::connect(imp_.Timer_.get(), &QTimer::timeout, []()
	{
		OgreEnv::GetInstance().RenderOneFrame();
	});
	imp_.Timer_->start();

	qmlRegisterSingletonType<EventWrapper>("Cloudream.Controls", 1, 0, "EventWrapper", EventWrapper::instance);	 
	qmlRegisterSingletonType<DataModel>("Cloudream.Controls", 1, 0, "DataModel", DataModel::instance);
	qmlRegisterType<DataWrapper>("Cloudream.Controls", 1, 0, "DataWrapper");
	qmlRegisterType<QT5RenderWindow>("Cloudream.Controls", 1, 0, "OgreWindow");
	qmlRegisterType<QQuickView>("Cloudream.Controls", 1, 0, "AirBtn");

	OgreEnv::GetInstance().SetUIFrameEventResponser([&](const FrameEvent& fe)
	{
		if (fe.GetEventName() == SFE_OpenFile::StaticFrameEventName())
		{
			if (fe.GetEvent<SFE_OpenFile>()->LoadingState == SFE_OpenFile::ELS_FinishBuildInfo)
			{
				emit EventWrapper::getInstance()->loadSuccess();
				SFE_OpenDocROM fe;
				fe.DocROM = DocumentMgr::GetInstance().GetActiveDocument()->GetDocumentROM();
				OgreEnv::GetInstance().PostFrameEventTo3D(fe.ConvertToFrameEvent());
				qDebug("post openEvent to 3d");
			}
		}
		else if (fe.GetEventName() == SFE_Selection::StaticFrameEventName())
		{
			auto fes = fe.GetEvent<SFE_Selection>();
			emit EventWrapper::getInstance()->selectedDOM(QString::fromStdWString(fes->SelectedDOM->GetName()));
		}
		else if ( fe.GetEventName() == SFE_SDC::StaticFrameEventName() )
		{
			auto fes = fe.GetEvent<SFE_SDC>();
			if ( fes->ConfirmUIState == SFE_SDC::EUS_ConfiromSave )
			{
				EventWrapper::getInstance()->saveFile();
			}
			else if (fes->ConfirmUIState == SFE_SDC::EUS_ConfiromDirty)			
			{
				emit EventWrapper::getInstance()->updateEditTime();
				DataModel::getInstance()->setDirty(true);
			}
		}
		else if (fe.GetEventName() == SFE_ToolBarState::StaticFrameEventName() )
		{
			auto fes = fe.GetEvent<SFE_ToolBarState>();
			emit EventWrapper::getInstance()->updateToolBarState(fes->Contate);
		}
		else if (fe.GetEventName() == SFE_FloatBarState::StaticFrameEventName())
		{
			auto fes = fe.GetEvent<SFE_FloatBarState>();
			emit EventWrapper::getInstance()->updateFloatBarState(fes->Contate);
		}
		else if (fe.GetEventName() == SFE_EditFeaturePoints::StaticFrameEventName())
		{
			auto fes = fe.GetEvent<SFE_EditFeaturePoints>();
			emit EventWrapper::getInstance()->updateFeaturepointsState(fes->conState);
		}
		else if (fe.GetEventName() == SFE_DataDirtyChange::StaticFrameEventName())
		{
			auto fes = fe.GetEvent<SFE_DataDirtyChange>();
			emit EventWrapper::getInstance()->dirtyChanged(fes->dirty);
		}
	});

	
	imp_.View_ = std::make_unique<QQuickView>();
	imp_.View_->setSource(QUrl::fromLocalFile("qml/main.qml"));
	imp_.View_->setTitle("mainWindow");
	imp_.View_->setFlags(Qt::FramelessWindowHint | Qt::Window);
	imp_.View_->setResizeMode(QQuickView::SizeRootObjectToView);
	imp_.View_->setX(300);
	imp_.View_->setY(200);
	imp_.View_->setWidth(1024);
	imp_.View_->setHeight(660);
	imp_.View_->rootContext()->setContextProperty("mainWindow", imp_.View_.get());

	auto win0 = imp_.View_->rootObject()->findChild<QT5RenderWindow*>("ogrewindow0");
	win0->SetInitFunc([&imp_](QT5RenderWindow* qtWnd, OgreWndWrapperUPtr& ogreWnd)
	{
		auto prefabs = PrefabControllerMgr::GetPrefabFrameListeners();

		auto found = false;
		for ( auto& cur : imp_.CmdOptions_ )
		{
			auto itor = prefabs.find(cur.first);
			if ( itor != prefabs.end() )
			{
				auto sl = itor->second(ogreWnd->GetRenderWindow());
				OgreEnv::GetInstance().AddFrameListener(sl);
				found = true;
				break;
			}
		}

		if ( !found )
		{
			auto sl = prefabs["default"](ogreWnd->GetRenderWindow());
			OgreEnv::GetInstance().AddFrameListener(sl);
		}
	});
	win0->setParent(imp_.View_.get());
	win0->show();

	auto airbtn = imp_.View_->rootObject()->findChild<QQuickView*>("airbtn");
	airbtn->setParent(imp_.View_.get());
	//airbtn->show();

	auto airbtn2 = imp_.View_->rootObject()->findChild<QQuickView*>("airbtn2");
	airbtn2->setParent(imp_.View_.get());
	airbtn2->show();

	auto profilewin = imp_.View_->rootObject()->findChild<QQuickView*>("profilewin");
	profilewin->setParent(imp_.View_.get());

	auto tipwin = imp_.View_->rootObject()->findChild<QQuickView*>("tipwin");
	tipwin->setParent(imp_.View_.get());

	//auto savewin = imp_.View_->rootObject()->findChild<QQuickView*>("savewin");
	//savewin->setParent(imp_.View_.get()); 

	QObject::connect(imp_.QApp_.get(), &QGuiApplication::aboutToQuit, [win0, this]()
	{
		if ( ImpUPtr_->Timer_ )
		{
			ImpUPtr_->Timer_->stop();
		}

		ImpUPtr_->Timer_.reset();

		win0->ReleaseWindow();

		OgreEnv::GetInstance().UnInit();

		ImpUPtr_->View_.reset();
	});
	imp_.View_->show();


	QIcon icon("qml/res/icon_app.png");
	imp_.QApp_->setWindowIcon(icon);
}

int QTApp::Run()
{
	auto& imp_ = *ImpUPtr_;
	return imp_.QApp_->exec();
}