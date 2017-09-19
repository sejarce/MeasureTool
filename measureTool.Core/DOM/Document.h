#pragma once

#include "DocumentFwd.h"
#include "IDOM.h"

#include "ROM/DocumentROMFwd.h"

#include <memory>
#include <vector>

#include <boost/signals2.hpp>

namespace yzm
{
class	PointCloud;
}

class	Document : public std::enable_shared_from_this<Document>
{
	class	Imp;
	std::unique_ptr<Imp>	ImpUPtr_;

public:

	class	SListener
	{
	public:

		using	OnCreateDOMSignal = boost::signals2::signal<void(const IDOMSPtr&)>;
		using	OnRemoveDOMSignal = boost::signals2::signal<void(const IDOMSPtr&)>;

	public:

		OnCreateDOMSignal	OnCreateDOM;
		OnRemoveDOMSignal	OnRemoveDOM;
	};

public:

	Document();

	~Document();

public:

	bool	ImportFile(const std::wstring& filePath);

	bool	ImportPlyFile(const std::wstring& filePath);

	bool	OpenFile(const std::wstring& filePath);

	const std::wstring& GetFilePath() const;

	void	SaveToFile(const std::wstring& filePath);

	void	ExportCSVFile(const std::wstring& filePath);

	const yzm::PointCloud&	GetPointCloud() const;

public:

	SListener&			GetListener();

	IDOMSPtr			CreateDOM(IDOM::EDOMType itemType);

	void				RemoveDOM(const IDOMSPtr& item);

	const IDOMList&		GetDOMList() const;

	void				SetDocumentROM(const DocumentROMSPtr& docROM);

	DocumentROMSPtr		GetDocumentROM() const;
public:
	Ogre::Vector3		getCrothPoint() const;
	Ogre::Vector3		getLeftPoint() const;
	Ogre::Vector3		getRightPoint() const;
	void		setCrothPoint(const Ogre::Vector3& vec);
	void		setLeftPoint(const Ogre::Vector3& vec);
	void		setRightPoint(const Ogre::Vector3& vec);

	std::map<int, Ogre::Vector3> getFeaturePoints() const;

public:
	bool estimateData();

public:

	void	Save();

	void	Redo();

	void	Undo();
};