#pragma once

#include <string>

class	StringUtil
{
public:
	
	static	std::wstring	UTF8ToUTF16(const std::string& utf8str);

	static	std::string		UTF16ToUTF8(const std::wstring& utf16str);
};