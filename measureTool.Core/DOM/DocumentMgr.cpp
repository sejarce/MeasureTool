#include "DocumentMgr.h"
#include "Document.h"

#include <vector>

class	DocumentMgr::Imp
{
public:

	std::vector<DocumentSPtr>	DocumentList_;
	DocumentSPtr					ActiveDocument_;
};

DocumentMgr::DocumentMgr() :ImpUPtr_(std::make_unique<Imp>())
{

}

DocumentMgr::~DocumentMgr()
{

}

DocumentMgr& DocumentMgr::GetInstance()
{
	static	DocumentMgr sIns;

	return sIns;
}

void DocumentMgr::UnInit()
{
	auto& imp_ = *ImpUPtr_;

	imp_.ActiveDocument_.reset();
	imp_.DocumentList_.clear();
}

DocumentSPtr DocumentMgr::OpenDocument(const std::wstring& filePath)
{
	auto& imp_ = *ImpUPtr_;

	auto newDoc = std::make_shared<Document>();
	auto ret = newDoc->OpenFile(filePath);
	if ( !ret )
	{
		return{};
	}

	imp_.DocumentList_.push_back(newDoc);
	
	return newDoc;
}

void DocumentMgr::CloseDocument(const DocumentSPtr& doc)
{
	auto& imp_ = *ImpUPtr_;

	auto itor = std::remove(imp_.DocumentList_.begin(), imp_.DocumentList_.end(), doc);
	imp_.DocumentList_.erase(itor, imp_.DocumentList_.end());

	if ( imp_.ActiveDocument_ == doc )
	{
		imp_.ActiveDocument_.reset();
	}
}

DocumentSPtr DocumentMgr::ImportOgreMesh(const std::wstring& filePath)
{
	auto& imp_ = *ImpUPtr_;

	auto newDoc = std::make_shared<Document>();
	auto ret = newDoc->ImportFile(filePath);
	if ( !ret )
	{
		return{};
	}

	imp_.DocumentList_.push_back(newDoc);

	return newDoc;
}

void DocumentMgr::SetActiveDocument(const DocumentSPtr& doc)
{
	auto& imp_ = *ImpUPtr_;

	auto itor = std::find(imp_.DocumentList_.begin(), imp_.DocumentList_.end(), doc);
	if ( itor == imp_.DocumentList_.end() )
	{
		imp_.DocumentList_.push_back(doc);
	}

	imp_.ActiveDocument_ = doc;
}

DocumentSPtr DocumentMgr::GetActiveDocument() const
{
	auto& imp_ = *ImpUPtr_;

	return imp_.ActiveDocument_;
}
