#pragma once

#include "SysEventFwd.h"

#include "OISMouse.h"
#include "OISKeyboard.h"

#include <map>
#include <bitset>

class	KeyEventRecorder
{
	typedef	std::map<OIS::KeyCode, bool>	KeyMap;

	KeyMap	KeyMap_;

public:

	void	Reset();

	void	TransferEvent(const SKeyboardEvent& evt);

	void	SetKeyborad(OIS::KeyCode kc, bool press);

	bool	HasPressKeyboard(OIS::KeyCode kc) const;

	bool	HasReleaseKeyboard(OIS::KeyCode kc) const;
};

class	MouseEventRecorder
{
	enum EOISMouseButtonWrap
	{
		MB_OISCount = OIS::MB_Button7 + 1,
		MB_Release = MB_OISCount * 2,
		MB_Wheel,
		MB_Moved,
		MB_COUNT
	};
	typedef	std::bitset<MB_COUNT>	MouseButtonMap;

	MouseButtonMap	MouseMap_;
	OIS::MouseState	MouseState_;

public:

	void	Reset();

	void	TransferEvent(const SMouseEvent& evt);

	void	SetMouseButton(OIS::MouseButtonID id, bool press);

	bool	HasPressMouseButton(OIS::MouseButtonID id) const;

	bool	HasReleaseMouseButton(OIS::MouseButtonID id) const;

	bool	HasMouseMoved() const;

	bool	HasRollWheel() const;

	const OIS::MouseState&	GetMouseState() const;
};

class	SysEventRecorder
{
	KeyEventRecorder	KeyRecorder_;
	MouseEventRecorder	MouseRecorder_;

public:

	void	Reset();

	void	SetOccurKeyborad(OIS::KeyCode kc, bool press);

	void	SetOccurMouseButton(OIS::MouseButtonID id, bool press);

	void	TransferEvent(const SSysEvent& evt);

	bool	HasKeyPressed(OIS::KeyCode kc) const;

	bool	HasKeyReleased(OIS::KeyCode kc) const;

	bool	HasMousePressed(OIS::MouseButtonID id) const;

	bool	HasMouseReleased(OIS::MouseButtonID id) const;

	bool	HasMouseMoved() const;

	bool	HasWheelRoll() const;

	const OIS::MouseState&	GetMouseState() const;

public:

	static	OIS::MouseState&	CachedMouseState();

	static	OIS::MouseState&	InterruptMouseState();

	static	bool				IsKeyDown(OIS::KeyCode kc);

	static	bool				IsKeyDown(OIS::MouseButtonID kc);
};

OIS::MouseState	operator - ( const OIS::MouseState& s1, const OIS::MouseState& s2 );