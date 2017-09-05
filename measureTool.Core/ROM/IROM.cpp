#include "IROM.h"

#include "DOM/IDOM.h"

#include "Ogre.h"

#include <vector>

class	IROM::ROMImp
{
public:
	
	using	MeasureItemDOMWPtr = std::weak_ptr<IDOM>;

public:

	MeasureItemDOMWPtr	DOM_;
	bool				Visible_{ false };
	EPickingState		PickingState_{ EPS_Normal };
	EDisplayMode		DisplayMode_;
	Ogre::SceneNode*	RootNode_{};
	Ogre::SceneManager*	Smgr_{};

public:

	static	const Ogre::ColourValue& PickingClr(EPickingState pickingState)
	{
		static	std::vector<Ogre::ColourValue> clrList;
		if ( clrList.empty() )
		{
			clrList.resize(EPS_COUNT);
			clrList[EPS_Normal] = Ogre::ColourValue(0, 0, 1);
			clrList[EPS_Sweep] = Ogre::ColourValue(0, 0, 1);
			clrList[EPS_Select] = Ogre::ColourValue(1.f, 247.f / 255, 0.f);
		}

		return clrList[pickingState];
	}

	static	float PickingSize(EPickingState pickingState)
	{
		static	std::vector<float> sizeList;
		if ( sizeList.empty() )
		{
			sizeList.resize(EPS_COUNT);
			sizeList[EPS_Normal] = 0.004f;
			sizeList[EPS_Sweep] = .008f;
			sizeList[EPS_Select] = .008f;
		}

		return sizeList[pickingState];
	}
};

IROM::IROM(Ogre::SceneNode* rootNode, const IDOMSPtr& dom) :ROMImp_(std::make_unique<ROMImp>())
{
	auto& imp_ = *ROMImp_;

	imp_.DOM_ = dom;
	imp_.RootNode_ = rootNode->createChildSceneNode();
	imp_.Smgr_ = rootNode->getCreator();

	SetVisible(false);
}

IROM::~IROM()
{
	auto& imp_ = *ROMImp_;

	imp_.RootNode_->removeAndDestroyAllChildren();
	imp_.RootNode_->getParentSceneNode()->removeChild(imp_.RootNode_);
}

const Ogre::ColourValue& IROM::PickingClr(EPickingState pickingState)
{
	return ROMImp::PickingClr(pickingState);
}

float IROM::PickingSize(EPickingState pickingState)
{
	return ROMImp::PickingSize(pickingState);
}

IROMSPtr IROM::GetFromDOM(const IDOMSPtr& dom)
{
	return dom->GetROM();
}

void IROM::SetVisible(bool val)
{
	auto& imp_ = *ROMImp_;

	OnSetVisible(val);

	imp_.Visible_ = val;
}

bool IROM::GetVisible() const
{
	auto& imp_ = *ROMImp_;

	return imp_.Visible_;
}

void IROM::SetPickingState(EPickingState state)
{
	auto& imp_ = *ROMImp_;

	OnSetPickingState(state);

	imp_.PickingState_ = state;
}

IROM::EPickingState IROM::GetPickingState() const
{
	auto& imp_ = *ROMImp_;

	return imp_.PickingState_;
}

void IROM::SetDisplayMode(EDisplayMode mode)
{
	auto& imp_ = *ROMImp_;

	OnSetDisplayMode(mode);

	imp_.DisplayMode_ = mode;
}

boost::optional<float> IROM::GetDistanceToRay(const TopoDS_Edge& ray, float deviation)
{
	auto& imp_ = *ROMImp_;

	if ( !GetVisible() )
	{
		return boost::none;
	}

	return OnGetDistanceToRay(ray, deviation);
}

void IROM::StashSave()
{
	OnStashSave();
}

void IROM::StashPop()
{
	OnStashPop();
}

IDOMSPtr IROM::GetDOM() const
{
	auto& imp_ = *ROMImp_;

	return imp_.DOM_.lock();
}

Ogre::SceneNode* IROM::GetRootNode() const
{
	auto& imp_ = *ROMImp_;

	return imp_.RootNode_;
}

Ogre::SceneManager* IROM::GetSmgr() const
{
	auto& imp_ = *ROMImp_;

	return imp_.Smgr_;
}
