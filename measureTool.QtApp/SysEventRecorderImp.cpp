#include "Render/SysEventRecorder.h"

#include <windows.h>

#include <map>

bool SysEventRecorder::IsKeyDown(OIS::KeyCode kc)
{
	static std::map<OIS::KeyCode, int> km;
	if ( km.empty() )
	{
		auto invalidIndex = static_cast<int>( OIS::KC_MEDIASELECT ) + 1;

		km.emplace(OIS::KC_UNASSIGNED, invalidIndex++);
		km.emplace(OIS::KC_ESCAPE, VK_ESCAPE);
		km.emplace(OIS::KC_1, '1');
		km.emplace(OIS::KC_2, '2');
		km.emplace(OIS::KC_3, '3');
		km.emplace(OIS::KC_4, '4');
		km.emplace(OIS::KC_5, '5');
		km.emplace(OIS::KC_6, '6');
		km.emplace(OIS::KC_7, '7');
		km.emplace(OIS::KC_8, '8');
		km.emplace(OIS::KC_9, '9');
		km.emplace(OIS::KC_0, '0');
		km.emplace(OIS::KC_MINUS, '-');
		km.emplace(OIS::KC_EQUALS, '=');
		km.emplace(OIS::KC_BACK, VK_BACK);
		km.emplace(OIS::KC_TAB, VK_TAB);
		km.emplace(OIS::KC_Q, 'Q');
		km.emplace(OIS::KC_W, 'W');
		km.emplace(OIS::KC_E, 'E');
		km.emplace(OIS::KC_R, 'R');
		km.emplace(OIS::KC_T, 'T');
		km.emplace(OIS::KC_Y, 'Y');
		km.emplace(OIS::KC_U, 'U');
		km.emplace(OIS::KC_I, 'I');
		km.emplace(OIS::KC_O, 'O');
		km.emplace(OIS::KC_P, 'P');
		km.emplace(OIS::KC_LBRACKET, '{');
		km.emplace(OIS::KC_RBRACKET, '}');
		km.emplace(OIS::KC_RETURN, VK_RETURN);
		km.emplace(OIS::KC_LCONTROL, VK_LCONTROL);
		km.emplace(OIS::KC_A, 'A');
		km.emplace(OIS::KC_S, 'S');
		km.emplace(OIS::KC_D, 'D');
		km.emplace(OIS::KC_F, 'F');
		km.emplace(OIS::KC_G, 'G');
		km.emplace(OIS::KC_H, 'H');
		km.emplace(OIS::KC_J, 'J');
		km.emplace(OIS::KC_K, 'K');
		km.emplace(OIS::KC_L, 'L');
		km.emplace(OIS::KC_SEMICOLON, ';');
		km.emplace(OIS::KC_APOSTROPHE, '.');
		km.emplace(OIS::KC_GRAVE, 0);
		km.emplace(OIS::KC_LSHIFT, VK_LSHIFT);
		km.emplace(OIS::KC_BACKSLASH, '\\');
		km.emplace(OIS::KC_Z, 'Z');
		km.emplace(OIS::KC_X, 'X');
		km.emplace(OIS::KC_C, 'C');
		km.emplace(OIS::KC_V, 'V');
		km.emplace(OIS::KC_B, 'B');
		km.emplace(OIS::KC_N, 'N');
		km.emplace(OIS::KC_M, 'M');
		km.emplace(OIS::KC_COMMA, ',');
		km.emplace(OIS::KC_PERIOD, 0);
		km.emplace(OIS::KC_SLASH, '/');
		km.emplace(OIS::KC_RSHIFT, VK_RSHIFT);
		km.emplace(OIS::KC_MULTIPLY, VK_MULTIPLY);
		km.emplace(OIS::KC_LMENU, VK_LMENU);
		km.emplace(OIS::KC_SPACE, VK_SPACE);
		km.emplace(OIS::KC_CAPITAL, VK_CAPITAL);
		km.emplace(OIS::KC_F1, VK_F1);
		km.emplace(OIS::KC_F2, VK_F2);
		km.emplace(OIS::KC_F3, VK_F3);
		km.emplace(OIS::KC_F4, VK_F4);
		km.emplace(OIS::KC_F5, VK_F5);
		km.emplace(OIS::KC_F6, VK_F6);
		km.emplace(OIS::KC_F7, VK_F7);
		km.emplace(OIS::KC_F8, VK_F8);
		km.emplace(OIS::KC_F9, VK_F9);
		km.emplace(OIS::KC_F10, VK_F10);
		km.emplace(OIS::KC_NUMLOCK, VK_NUMLOCK);
		km.emplace(OIS::KC_SCROLL, VK_SCROLL);
		km.emplace(OIS::KC_NUMPAD7, VK_NUMPAD7);
		km.emplace(OIS::KC_NUMPAD8, VK_NUMPAD8);
		km.emplace(OIS::KC_NUMPAD9, VK_NUMPAD9);
		km.emplace(OIS::KC_SUBTRACT, VK_SUBTRACT);
		km.emplace(OIS::KC_NUMPAD4, VK_NUMPAD4);
		km.emplace(OIS::KC_NUMPAD5, VK_NUMPAD5);
		km.emplace(OIS::KC_NUMPAD6, VK_NUMPAD6);
		km.emplace(OIS::KC_ADD, VK_ADD);
		km.emplace(OIS::KC_NUMPAD1, VK_NUMPAD1);
		km.emplace(OIS::KC_NUMPAD2, VK_NUMPAD2);
		km.emplace(OIS::KC_NUMPAD3, VK_NUMPAD3);
		km.emplace(OIS::KC_NUMPAD0, VK_NUMPAD0);
		km.emplace(OIS::KC_DECIMAL, VK_DECIMAL);
		km.emplace(OIS::KC_OEM_102, invalidIndex++);
		km.emplace(OIS::KC_F11, VK_F11);
		km.emplace(OIS::KC_F12, VK_F12);
		km.emplace(OIS::KC_F13, VK_F13);
		km.emplace(OIS::KC_F14, VK_F14);
		km.emplace(OIS::KC_F15, VK_F15);
		km.emplace(OIS::KC_KANA, VK_KANA);
		km.emplace(OIS::KC_ABNT_C1, invalidIndex++);
		km.emplace(OIS::KC_CONVERT, VK_CONVERT);
		km.emplace(OIS::KC_NOCONVERT, VK_NONCONVERT);
		km.emplace(OIS::KC_YEN, invalidIndex++);
		km.emplace(OIS::KC_ABNT_C2, invalidIndex++);
		km.emplace(OIS::KC_NUMPADEQUALS, invalidIndex++);
		km.emplace(OIS::KC_PREVTRACK, invalidIndex++);
		km.emplace(OIS::KC_AT, VK_ATTN);
		km.emplace(OIS::KC_COLON, ':');
		km.emplace(OIS::KC_UNDERLINE, '_');
		km.emplace(OIS::KC_KANJI, VK_KANJI);
		km.emplace(OIS::KC_STOP, invalidIndex++);
		km.emplace(OIS::KC_AX, invalidIndex++);
		km.emplace(OIS::KC_UNLABELED, invalidIndex++);
		km.emplace(OIS::KC_NEXTTRACK, invalidIndex++);
		km.emplace(OIS::KC_NUMPADENTER, invalidIndex++);
		km.emplace(OIS::KC_RCONTROL, VK_RCONTROL);
		km.emplace(OIS::KC_MUTE, invalidIndex++);
		km.emplace(OIS::KC_CALCULATOR, invalidIndex++);
		km.emplace(OIS::KC_PLAYPAUSE, invalidIndex++);
		km.emplace(OIS::KC_MEDIASTOP, invalidIndex++);
		km.emplace(OIS::KC_VOLUMEDOWN, invalidIndex++);
		km.emplace(OIS::KC_VOLUMEUP, invalidIndex++);
		km.emplace(OIS::KC_WEBHOME, invalidIndex++);
		km.emplace(OIS::KC_NUMPADCOMMA, invalidIndex++);
		km.emplace(OIS::KC_DIVIDE, VK_DIVIDE);
		km.emplace(OIS::KC_SYSRQ, invalidIndex++);
		km.emplace(OIS::KC_RMENU, VK_RMENU);
		km.emplace(OIS::KC_PAUSE, invalidIndex++);
		km.emplace(OIS::KC_HOME, VK_HOME);
		km.emplace(OIS::KC_UP, VK_UP);
		km.emplace(OIS::KC_PGUP, invalidIndex++);
		km.emplace(OIS::KC_LEFT, VK_LEFT);
		km.emplace(OIS::KC_RIGHT, VK_RIGHT);
		km.emplace(OIS::KC_END, VK_END);
		km.emplace(OIS::KC_DOWN, VK_DOWN);
		km.emplace(OIS::KC_PGDOWN, invalidIndex++);
		km.emplace(OIS::KC_INSERT, VK_INSERT);
		km.emplace(OIS::KC_DELETE, VK_DELETE);
		km.emplace(OIS::KC_LWIN, VK_LWIN);
		km.emplace(OIS::KC_RWIN, VK_RWIN);
		km.emplace(OIS::KC_APPS, VK_APPS);
		km.emplace(OIS::KC_POWER, invalidIndex++);
		km.emplace(OIS::KC_SLEEP, VK_SLEEP);
		km.emplace(OIS::KC_WAKE, invalidIndex++);
		km.emplace(OIS::KC_WEBSEARCH, invalidIndex++);
		km.emplace(OIS::KC_WEBFAVORITES, invalidIndex++);
		km.emplace(OIS::KC_WEBREFRESH, invalidIndex++);
		km.emplace(OIS::KC_WEBSTOP, invalidIndex++);
		km.emplace(OIS::KC_WEBFORWARD, invalidIndex++);
		km.emplace(OIS::KC_WEBBACK, invalidIndex++);
		km.emplace(OIS::KC_MYCOMPUTER, invalidIndex++);
		km.emplace(OIS::KC_MAIL, invalidIndex++);
		km.emplace(OIS::KC_MEDIASELECT, invalidIndex++);
	}

	auto vk = km.find(kc);
	if ( vk != km.end() )
	{
		auto ret = ::GetAsyncKeyState(vk->second);
		return ( ret & 0x8000 ) != 0;
	}

	return false;
}

bool SysEventRecorder::IsKeyDown(OIS::MouseButtonID kc)
{
	auto vk = 0;

	switch ( kc )
	{
	case OIS::MB_Left:
	vk = VK_LBUTTON;
	break;
	case OIS::MB_Right:
	vk = VK_RBUTTON;
	break;
	case OIS::MB_Middle:
	vk = VK_MBUTTON;
	break;
	default:
	break;
	}

	auto ret = ::GetAsyncKeyState(vk);

	return ( ret & 0x8000 ) != 0;
}