#include "QT5RenderWindow.h"

#include "Render/OgreEnv.h"	
#include "Render/OgreWndWrapper.h"
#include "Render/SysEvent.h"

#include "QtWidgets/QApplication"

#include <unordered_map>

#include <boost/optional.hpp>

static boost::optional<OIS::KeyCode> QtKey2OISKey(int qtkey)
{
	static std::unordered_map<int, OIS::KeyCode> sCodeMap;
	if ( sCodeMap.empty() )
	{
		sCodeMap.bucket(108);

		sCodeMap.emplace(Qt::Key::Key_Escape, OIS::KC_ESCAPE);
		sCodeMap.emplace(Qt::Key::Key_Enter, OIS::KC_RETURN);
		sCodeMap.emplace(Qt::Key::Key_Control, OIS::KC_LCONTROL);
		sCodeMap.emplace(Qt::Key::Key_Alt, OIS::KC_LMENU);
		sCodeMap.emplace(Qt::Key::Key_Shift, OIS::KC_LSHIFT);
		sCodeMap.emplace(Qt::Key::Key_Tab, OIS::KC_TAB);
		sCodeMap.emplace(Qt::Key::Key_Space, OIS::KC_SPACE);
		sCodeMap.emplace(Qt::Key::Key_0, OIS::KC_0);
		sCodeMap.emplace(Qt::Key::Key_1, OIS::KC_1);
		sCodeMap.emplace(Qt::Key::Key_2, OIS::KC_2);
		sCodeMap.emplace(Qt::Key::Key_3, OIS::KC_3);
		sCodeMap.emplace(Qt::Key::Key_4, OIS::KC_4);
		sCodeMap.emplace(Qt::Key::Key_5, OIS::KC_5);
		sCodeMap.emplace(Qt::Key::Key_6, OIS::KC_6);
		sCodeMap.emplace(Qt::Key::Key_7, OIS::KC_7);
		sCodeMap.emplace(Qt::Key::Key_8, OIS::KC_8);
		sCodeMap.emplace(Qt::Key::Key_9, OIS::KC_9);
		sCodeMap.emplace(Qt::Key::Key_F1, OIS::KC_F1);
		sCodeMap.emplace(Qt::Key::Key_F2, OIS::KC_F2);
		sCodeMap.emplace(Qt::Key::Key_F3, OIS::KC_F3);
		sCodeMap.emplace(Qt::Key::Key_F4, OIS::KC_F4);
		sCodeMap.emplace(Qt::Key::Key_F5, OIS::KC_F5);
		sCodeMap.emplace(Qt::Key::Key_F6, OIS::KC_F6);
		sCodeMap.emplace(Qt::Key::Key_F7, OIS::KC_F7);
		sCodeMap.emplace(Qt::Key::Key_F8, OIS::KC_F8);
		sCodeMap.emplace(Qt::Key::Key_F9, OIS::KC_F9);
		sCodeMap.emplace(Qt::Key::Key_F10, OIS::KC_F10);
		sCodeMap.emplace(Qt::Key::Key_F11, OIS::KC_F11);
		sCodeMap.emplace(Qt::Key::Key_F12, OIS::KC_F12);
		sCodeMap.emplace(Qt::Key::Key_A, OIS::KC_A);
		sCodeMap.emplace(Qt::Key::Key_B, OIS::KC_B);
		sCodeMap.emplace(Qt::Key::Key_C, OIS::KC_C);
		sCodeMap.emplace(Qt::Key::Key_D, OIS::KC_D);
		sCodeMap.emplace(Qt::Key::Key_E, OIS::KC_E);
		sCodeMap.emplace(Qt::Key::Key_F, OIS::KC_F);
		sCodeMap.emplace(Qt::Key::Key_G, OIS::KC_G);
		sCodeMap.emplace(Qt::Key::Key_H, OIS::KC_H);
		sCodeMap.emplace(Qt::Key::Key_I, OIS::KC_I);
		sCodeMap.emplace(Qt::Key::Key_J, OIS::KC_J);
		sCodeMap.emplace(Qt::Key::Key_K, OIS::KC_K);
		sCodeMap.emplace(Qt::Key::Key_L, OIS::KC_L);
		sCodeMap.emplace(Qt::Key::Key_M, OIS::KC_M);
		sCodeMap.emplace(Qt::Key::Key_N, OIS::KC_N);
		sCodeMap.emplace(Qt::Key::Key_O, OIS::KC_O);
		sCodeMap.emplace(Qt::Key::Key_P, OIS::KC_P);
		sCodeMap.emplace(Qt::Key::Key_Q, OIS::KC_Q);
		sCodeMap.emplace(Qt::Key::Key_R, OIS::KC_R);
		sCodeMap.emplace(Qt::Key::Key_S, OIS::KC_S);
		sCodeMap.emplace(Qt::Key::Key_T, OIS::KC_T);
		sCodeMap.emplace(Qt::Key::Key_U, OIS::KC_U);
		sCodeMap.emplace(Qt::Key::Key_V, OIS::KC_V);
		sCodeMap.emplace(Qt::Key::Key_W, OIS::KC_W);
		sCodeMap.emplace(Qt::Key::Key_X, OIS::KC_X);
		sCodeMap.emplace(Qt::Key::Key_Y, OIS::KC_Y);
		sCodeMap.emplace(Qt::Key::Key_Z, OIS::KC_Z);
	}

	auto itor = sCodeMap.find(qtkey);
	if ( itor != sCodeMap.end() )
	{
		return itor->second;
	}
	else
	{
		return boost::none;
	}
}

QT5RenderWindow::QT5RenderWindow(QQuickView* window)
	: QQuickView(window)
	, m_initialized(false)
	, m_ogreRenderWnd(nullptr)
	, m_platformSurfaced(false)
{
	installEventFilter(this);																					    
}


QT5RenderWindow::~QT5RenderWindow()
{		
	qDebug("~QT5RenderWindow");
}

/*
Our event filter; handles the resizing of the QWindow. When the size of the QWindow changes note the
call to the Ogre3D window and camera. This keeps the Ogre3D scene looking correct.
*/
bool QT5RenderWindow::eventFilter(QObject *target, QEvent *event)
{
	//if ( target == this )
	//{
	//	if ( event->type() == QEvent::Resize )
	//	{
	//		//send msg resize
	//		if (m_ogreRenderWnd.get())
	//			m_ogreRenderWnd->Resize(this->width(), this->height());
	//	}
	//}
	return false;
}


void QT5RenderWindow::keyPressEvent(QKeyEvent * ev)
{
	//send key Press msg to camera manipulator
	if (m_ogreRenderWnd.get())
	{
		auto kc = QtKey2OISKey(ev->key());
		if ( kc )
		{
			SSysEvent sEvent;
			sEvent.EventType = SSysEvent::EET_Keyboard;
			sEvent.KeyboardEvt.Pressed = true;
			sEvent.KeyboardEvt.KeyCode = *kc;
			OgreEnv::GetInstance().PostSysEventTo3D(*m_ogreRenderWnd.get(), sEvent);
		}
	}
	QQuickView::keyPressEvent(ev);
}


void QT5RenderWindow::keyReleaseEvent(QKeyEvent * ev)
{
	//send key release msg to camera manipulator
	if (m_ogreRenderWnd.get())
	{
		auto kc = QtKey2OISKey(ev->key());
		if ( kc )
		{
			SSysEvent sEvent;
			sEvent.EventType = SSysEvent::EET_Keyboard;
			sEvent.KeyboardEvt.Pressed = false;
			sEvent.KeyboardEvt.KeyCode = *kc;
			OgreEnv::GetInstance().PostSysEventTo3D(*m_ogreRenderWnd.get(), sEvent);
		}
	}
	QQuickView::keyReleaseEvent(ev);
}


void QT5RenderWindow::mouseMoveEvent(QMouseEvent* e)
{
	//send mouse move msg to camera manipulator
	if (m_ogreRenderWnd.get())
	{
		SSysEvent sEvent;
		sEvent.EventType = SSysEvent::EET_Mouse;
		sEvent.MouseEvt.Moved = true;
		sEvent.MouseEvt.State.X.rel = e->localPos().x();
		sEvent.MouseEvt.State.Y.rel = e->localPos().y();
		OgreEnv::GetInstance().PostSysEventTo3D(*m_ogreRenderWnd.get(), sEvent);
	}
	QQuickView::mouseMoveEvent(e);
}


void QT5RenderWindow::wheelEvent(QWheelEvent* e)
{
	//send wheel msg to camera manipulator
	if (m_ogreRenderWnd.get())
	{
		SSysEvent sEvent;
		sEvent.EventType = SSysEvent::EET_Mouse;
		sEvent.MouseEvt.Wheeled = true;
		sEvent.MouseEvt.State.X.rel = e->pos().x();
		sEvent.MouseEvt.State.Y.rel = e->pos().y();
		sEvent.MouseEvt.State.Z.rel = e->delta();
		OgreEnv::GetInstance().PostSysEventTo3D(*m_ogreRenderWnd.get(), sEvent);
	}
	QQuickView::wheelEvent(e);
}


void QT5RenderWindow::mousePressEvent(QMouseEvent* e)
{
	//send mouse press msg to camera manipulator
	if (m_ogreRenderWnd.get())
	{
		SSysEvent sEvent;
		sEvent.EventType = SSysEvent::EET_Mouse;
		sEvent.MouseEvt.Pressed = true;
		switch (e->button())
		{
		case Qt::MouseButton::LeftButton:
			sEvent.MouseEvt.ButtonID = OIS::MouseButtonID::MB_Left;
			break;
		case Qt::MouseButton::MiddleButton:
			sEvent.MouseEvt.ButtonID = OIS::MouseButtonID::MB_Middle;
			break;
		case Qt::MouseButton::RightButton:
			sEvent.MouseEvt.ButtonID = OIS::MouseButtonID::MB_Right;
			break;
		default:
			break;
		}
		sEvent.MouseEvt.State.X.rel = e->localPos().x();
		sEvent.MouseEvt.State.Y.rel = e->localPos().y();
		OgreEnv::GetInstance().PostSysEventTo3D(*m_ogreRenderWnd.get(), sEvent);
	}
	QQuickView::mousePressEvent(e);
}


void QT5RenderWindow::mouseReleaseEvent(QMouseEvent* e)
{
	//send mouse release msg to camera manipulator
	if (m_ogreRenderWnd.get())
	{
		SSysEvent sEvent;
		sEvent.EventType = SSysEvent::EET_Mouse;
		sEvent.MouseEvt.Pressed = false;
		switch (e->button())
		{
		case Qt::MouseButton::LeftButton:
			sEvent.MouseEvt.ButtonID = OIS::MouseButtonID::MB_Left;
			break;
		case Qt::MouseButton::MiddleButton:
			sEvent.MouseEvt.ButtonID = OIS::MouseButtonID::MB_Middle;
			break;
		case Qt::MouseButton::RightButton:
			sEvent.MouseEvt.ButtonID = OIS::MouseButtonID::MB_Right;
			break;
		default:
			break;
		}
		sEvent.MouseEvt.State.X.rel = e->localPos().x();
		sEvent.MouseEvt.State.Y.rel = e->localPos().y();
		OgreEnv::GetInstance().PostSysEventTo3D(*m_ogreRenderWnd.get(), sEvent);
	}
	QQuickView::mouseReleaseEvent(e);
}

/*
Called after the QWindow is reopened or when the QWindow is first opened or the QWindow is resize.
*/
void QT5RenderWindow::exposeEvent(QExposeEvent *event)
{
	Q_UNUSED(event);
	if ( isExposed() && !m_initialized)
	{
		//send msg init
		m_ogreRenderWnd = OgreEnv::GetInstance().CreateRenderWindow(this->winId(), this->width(), this->height());
		if ( m_InitFunc )
		{
			m_InitFunc(this, m_ogreRenderWnd);
		}
		
		m_initialized = true;
	}
}


bool QT5RenderWindow::event(QEvent *event)
{
	
	//qDebug(std::to_string(event->type()).c_str());
	switch ( event->type() )
	{
	case QEvent::UpdateRequest:
	//send update msg to app

		return true;
	case QEvent::Destroy:
		return QQuickView::event(event);
	case QEvent::Close:
		return QQuickView::event(event);
	case QEvent::Quit:
		return QQuickView::event(event);												  
	case QEvent::Hide:
		return false;//QQuickView::event(event);
	case QEvent::PlatformSurface:
	{
		qDebug("platformSurface");
		m_platformSurfaced = !m_platformSurfaced;
		QQuickView::event(event);
	}
	case QEvent::Resize:
	{
		if (m_ogreRenderWnd && m_platformSurfaced)							   
			m_ogreRenderWnd->Resize(this->width(), this->height());													 
		QQuickView::event(event);
	}
	default:
		return QQuickView::event(event);
	}																										 
}
																												    

void QT5RenderWindow::ReleaseWindow()
{
	qDebug("releaseRenderWindow");
	m_ogreRenderWnd->Destory();
	m_ogreRenderWnd = nullptr;
}

void QT5RenderWindow::SetInitFunc(InitFunc func)
{
	m_InitFunc = func;
}

#include "moc_QT5RenderWindow.cpp"