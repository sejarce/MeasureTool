#pragma once


#include "QtGui/QWindow"
#include "QtGui/QKeyEvent"
#include "QtQuick/QQuickView"  
#include "Render/OgreWndWrapperFwd.h"

#include <functional>


class QT5RenderWindow : public QQuickView
{
	Q_OBJECT
public:

	using InitFunc = std::function<void(QT5RenderWindow*, OgreWndWrapperUPtr& wnd)>;

public:

	explicit QT5RenderWindow(QQuickView* window = 0);
	
	virtual ~QT5RenderWindow();

	/*
	We use an event filter to be able to capture keyboard/mouse events. More on this later.
	*/
	virtual bool eventFilter(QObject *target, QEvent *event);

	public Q_SLOTS:
	void ReleaseWindow();		

public:

	void	SetInitFunc(InitFunc func);

protected:
																																			 
	/*
	The below methods are what is actually fired when they keys on the keyboard are hit.
	Similar events are fired when the mouse is pressed or other events occur.
	*/
	virtual void keyPressEvent(QKeyEvent * ev);
	virtual void keyReleaseEvent(QKeyEvent * ev);
	virtual void mouseMoveEvent(QMouseEvent* e);
	virtual void wheelEvent(QWheelEvent* e);
	virtual void mousePressEvent(QMouseEvent* e);
	virtual void mouseReleaseEvent(QMouseEvent* e);
	virtual void exposeEvent(QExposeEvent *event);
	virtual bool event(QEvent *event);

protected:
	bool m_initialized;	
	bool m_platformSurfaced;
	OgreWndWrapperUPtr m_ogreRenderWnd;
	InitFunc	m_InitFunc;
};