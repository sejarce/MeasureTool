#include "StringUtil.h"

#include <boost/locale.hpp>

std::wstring StringUtil::UTF8ToUTF16(const std::string& utf8str)
{
	return boost::locale::conv::utf_to_utf<wchar_t>(utf8str);
}

std::string StringUtil::UTF16ToUTF8(const std::wstring& utf16str)
{
	return boost::locale::conv::utf_to_utf<char>(utf16str);
}
