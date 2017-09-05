#pragma execution_character_set("utf-8")

#include "SysEventRecorder.h"
#include "SysEvent.h"

OIS::MouseState	operator - ( const OIS::MouseState& s1, const OIS::MouseState& s2 )
{
	OIS::MouseState s;
	s.X.abs = s1.X.abs - s2.X.abs;
	s.X.rel = s1.X.rel - s2.X.rel;
	s.Y.abs = s1.Y.abs - s2.Y.abs;
	s.Y.rel = s1.Y.rel - s2.Y.rel;
	s.Z.abs = s1.Z.abs - s2.Z.abs;
	s.Z.rel = s1.Z.rel - s2.Z.rel;

	return s;
}


OIS::MouseState& SysEventRecorder::CachedMouseState()
{
	static OIS::MouseState state;
	return state;
}

void KeyEventRecorder::Reset()
{
	KeyMap_.clear();
}

void KeyEventRecorder::TransferEvent(const SKeyboardEvent& evt)
{
	SetKeyborad(evt.KeyCode, evt.Pressed);
}

void KeyEventRecorder::SetKeyborad(OIS::KeyCode kc, bool press)
{
	KeyMap_[kc] = press;
}

bool KeyEventRecorder::HasPressKeyboard(OIS::KeyCode kc) const
{
	auto itor = KeyMap_.find(kc);
	if ( itor == KeyMap_.end() )
	{
		return false;
	}

	return itor->second;
}

bool KeyEventRecorder::HasReleaseKeyboard(OIS::KeyCode kc) const
{
	auto itor = KeyMap_.find(kc);
	if ( itor == KeyMap_.end() )
	{
		return false;
	}

	return !(itor->second);
}

void MouseEventRecorder::Reset()
{
	MouseMap_.reset();
}

void MouseEventRecorder::TransferEvent(const SMouseEvent& evt)
{
	auto& mouseEvt = evt;

	if ( mouseEvt.Wheeled )
	{
		MouseMap_.set(MB_Wheel);
	}
	else if ( !mouseEvt.Moved )
	{
		SetMouseButton(mouseEvt.ButtonID, mouseEvt.Pressed);
	}
	else if ( mouseEvt.Moved )
	{
		MouseMap_.set(MB_Moved);
	}

	MouseState_ = mouseEvt.State;
}

void MouseEventRecorder::SetMouseButton(OIS::MouseButtonID id, bool press)
{
	if ( press )
	{
		MouseMap_.set(id);
	}
	else
	{
		MouseMap_.set(id + MB_OISCount);
	}
}

bool MouseEventRecorder::HasPressMouseButton(OIS::MouseButtonID id) const
{
	return MouseMap_.test(id);
}

bool MouseEventRecorder::HasReleaseMouseButton(OIS::MouseButtonID id) const
{
	return MouseMap_.test(id + MB_OISCount);
}

bool MouseEventRecorder::HasMouseMoved() const
{
	return MouseMap_.test(MB_Moved);
}

const OIS::MouseState& MouseEventRecorder::GetMouseState() const
{
	return MouseState_;
}

bool MouseEventRecorder::HasRollWheel() const
{
	return MouseMap_.test(MB_Wheel);
}


void SysEventRecorder::Reset()
{
	KeyRecorder_.Reset();
	MouseRecorder_.Reset();
}

void SysEventRecorder::SetOccurKeyborad(OIS::KeyCode kc, bool press)
{
	KeyRecorder_.SetKeyborad(kc, press);
}

void SysEventRecorder::SetOccurMouseButton(OIS::MouseButtonID id, bool press)
{
	MouseRecorder_.SetMouseButton(id, press);
}

void SysEventRecorder::TransferEvent(const SSysEvent& evt)
{
	switch (evt.EventType)
	{
	case SSysEvent::EET_Keyboard:
	{
		KeyRecorder_.TransferEvent(evt.KeyboardEvt);
	}
	break;
	case SSysEvent::EET_Mouse:
	{
		MouseRecorder_.TransferEvent(evt.MouseEvt);
	}
	break;
	default:break;
	}
	
}

bool SysEventRecorder::HasKeyPressed(OIS::KeyCode kc) const
{
	return KeyRecorder_.HasPressKeyboard(kc);
}

bool SysEventRecorder::HasKeyReleased(OIS::KeyCode kc) const
{
	return KeyRecorder_.HasReleaseKeyboard(kc);
}

bool SysEventRecorder::HasMousePressed(OIS::MouseButtonID id) const
{
	return MouseRecorder_.HasPressMouseButton(id);
}

bool SysEventRecorder::HasMouseReleased(OIS::MouseButtonID id) const
{
	return MouseRecorder_.HasReleaseMouseButton(id);
}

bool SysEventRecorder::HasMouseMoved() const
{
	return MouseRecorder_.HasMouseMoved();
}

bool SysEventRecorder::HasWheelRoll() const
{
	return MouseRecorder_.HasRollWheel();
}

const OIS::MouseState& SysEventRecorder::GetMouseState() const
{
	return MouseRecorder_.GetMouseState();
}

OIS::MouseState& SysEventRecorder::InterruptMouseState()
{
	static OIS::MouseState state;

	return state;
}