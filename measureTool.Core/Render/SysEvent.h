#pragma once

#include "SysEventFwd.h"

#include "OISMouse.h"
#include "OISKeyboard.h"

class	SKeyboardEvent
{
public:

	OIS::KeyCode	KeyCode = OIS::KC_UNASSIGNED;
	bool			Pressed = false;
};

class	SMouseEvent
{
public:

	OIS::MouseButtonID	ButtonID;
	OIS::MouseState		State;
	bool				Pressed = false;
	bool				Moved = false;
	bool				Wheeled = false;
};

class	SSysEvent
{
public:

	enum EEventType
	{
		EET_Keyboard,
		EET_Mouse
	};

	EEventType		EventType;
	SKeyboardEvent	KeyboardEvt;
	SMouseEvent		MouseEvt;
};