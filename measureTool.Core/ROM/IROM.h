#pragma once

#include "IROMFwd.h"

#include "DOM/IDOMFwd.h"

#include "OgrePrerequisites.h"

#include <boost/optional.hpp>

class	TopoDS_Edge;

class	IROM
{
	class	ROMImp;

	std::unique_ptr<ROMImp>	ROMImp_;

public:

	enum EPickingState
	{
		EPS_Normal,
		EPS_Sweep,
		EPS_Select,
		EPS_COUNT
	};

	enum EDisplayMode
	{
		EDM_Browser,
		EDM_Edit,
		EDM_COUNT
	};

	using	DOMParamList = std::vector<Ogre::Vector3>;

public:

	IROM(Ogre::SceneNode* rootNode, const IDOMSPtr& dom);

	virtual ~IROM();

public:

	static	const Ogre::ColourValue&	PickingClr(EPickingState pickingState);

	static	float						PickingSize(EPickingState pickingState);

	static	IROMSPtr					GetFromDOM(const IDOMSPtr& dom);

public:

	void	SetVisible(bool val);

	bool	GetVisible() const;

	void	SetPickingState(EPickingState state);

	EPickingState	GetPickingState() const;

	void	SetDisplayMode(EDisplayMode mode);

	boost::optional<float>	GetDistanceToRay(const TopoDS_Edge& ray, float deviation);

	void	StashSave();

	void	StashPop();

public:

	IDOMSPtr	GetDOM() const;

protected:

	virtual	void	OnSetVisible(bool val) {}

	virtual	void	OnSetPickingState(EPickingState state) {}

	virtual	void	OnSetDisplayMode(EDisplayMode mode) {}

	virtual boost::optional<float>	OnGetDistanceToRay(const TopoDS_Edge& ray, float deviation)
	{
		return boost::none;
	}

	virtual void	OnStashSave() {}

	virtual	void	OnStashPop() {}

protected:

	Ogre::SceneNode*	GetRootNode() const;

	Ogre::SceneManager*	GetSmgr() const;
};

template <typename T>
class	TROM : public IROM
{
public:

	TROM(Ogre::SceneNode* rootNode, const IDOMSPtr& dom) :IROM(rootNode, dom) {}

	using	SPtr = std::shared_ptr<T>;
};