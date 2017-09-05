#pragma once

#include "DocumentROMFwd.h"
#include "IROMFwd.h"
#include "DOM/DocumentFwd.h"

#include "Util/PCLOctreeFwd.h"
#include "Util/PCLKdtreeFwd.h"

#include "OgrePrerequisites.h"

#include <vector>

class	DocumentROM : public std::enable_shared_from_this<DocumentROM>
{
	class	Imp;
	std::unique_ptr<Imp>	ImpUPtr_;

	class	PrivateHolder {};

public:

	using	ROMList = std::vector<IROMSPtr>;

public:

	DocumentROM(PrivateHolder);

	~DocumentROM();

public:

	static	DocumentROMSPtr	Create(const DocumentSPtr& docDOM, Ogre::SceneManager* smgr);

public:

	Ogre::MeshPtr	GetMesh() const;

	Ogre::SceneNode*	GetDocROMNode() const;

	PCLOctreeSPtr	GetOctree() const;

	PCLKdtreeSPtr	GetKdtree() const;

	const ROMList&	GetROMList() const;
};