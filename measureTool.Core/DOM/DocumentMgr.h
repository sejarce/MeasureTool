#pragma once

#include "DocumentFwd.h"

#include <string>

class	DocumentMgr
{
	class	Imp;
	std::unique_ptr<Imp>	ImpUPtr_;

	DocumentMgr();

public:

	~DocumentMgr();

public:

	static	DocumentMgr&	GetInstance();

public:

	void			UnInit();

	DocumentSPtr	OpenDocument(const std::wstring& filePath);

	void			CloseDocument(const DocumentSPtr& doc);

	DocumentSPtr ImportOgreMesh(const std::wstring& filePath);

	DocumentSPtr	ImportPly(const std::wstring& filePath);

	void			SetActiveDocument(const DocumentSPtr& doc);

	DocumentSPtr	GetActiveDocument() const;
};