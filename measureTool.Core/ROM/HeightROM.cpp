#include "HeightROM.h"

#include "DOM/HeightDOM.h"

#include "Util/MathUtil.h"

#include "Render/Extension/Line3D.h"
#include "Render/Extension/Point3D.h"
#include "Render/RenderGroupID.h"

#include "TopoDS_Edge.hxx"
#include "BRepBuilderAPI_MakeEdge.hxx"
#include "BRepExtrema_DistShapeShape.hxx"
#include "BRepAdaptor_Curve.hxx"

#include "Ogre.h"

class	HeightROM::Imp
{
public:

	Line3D*		Line_{};
	Ogre::SceneNode*	Node_{};
	TopoDS_Edge	Edge_;

	boost::optional<Ogre::Vector3>	StashP1_;
	boost::optional<Ogre::Vector3>	StashP2_;

public:

	void	SetPoint(const Ogre::Vector3& p1, const Ogre::Vector3& p2)
	{
		Line_->SetPoint(p1, p2);

		if ( p1.squaredDistance(p2) < 1e-6 )
		{
			Edge_.Nullify();
			return;
		}

		try
		{
			BRepBuilderAPI_MakeEdge me(MathUtil::ToOCCTPnt(p1), MathUtil::ToOCCTPnt(p2));
			Edge_ = me.Edge();
		}
		catch ( ... )
		{
			Edge_.Nullify();
		}
	}
};

static const auto NormalClr = Ogre::ColourValue(137.f / 255.f, 1.f, 0.0);	//89FF00
static const auto SelectClr = Ogre::ColourValue(238.f / 255.f, 103.f / 255.f, 35.f / 255.f);	//238,103,35

HeightROM::HeightROM(Ogre::SceneNode* rootNode, const IDOMSPtr& dom) :TROM(rootNode, dom)
,ImpUPtr_(std::make_unique<Imp>())
{
	auto& imp_ = *ImpUPtr_;

	imp_.Line_ = Line3DFactory::CreateInstance(GetSmgr());

	imp_.Node_ = GetRootNode()->createChildSceneNode();
	imp_.Node_->setName("HeightROM");

	imp_.Node_->attachObject(imp_.Line_);	

	imp_.Line_->setVisible(false);

	auto heiDOM = std::static_pointer_cast<HeightDOM>( dom );

	//Ogre::Vector3 pt_s(0, heiDOM->GetPoint1().y, 0);
	//Ogre::Vector3 pt_e(0, heiDOM->GetPoint2().y, 0);
	Ogre::Vector3 pt_s(heiDOM->GetPoint1());
	Ogre::Vector3 pt_e(heiDOM->GetPoint1().x, heiDOM->GetPoint2().y, heiDOM->GetPoint1().z);

	imp_.SetPoint(pt_s, pt_e);
}

HeightROM::~HeightROM()
{
	auto& imp_ = *ImpUPtr_;

	imp_.Line_->Destory();

	imp_.Node_->getParentSceneNode()->removeChild(imp_.Node_);
}

const Ogre::Vector3& HeightROM::GetStartPoint() const
{
	auto& imp_ = *ImpUPtr_;

	auto heiDOM = std::static_pointer_cast<HeightDOM>(GetDOM());
	return heiDOM->GetPoint1();
}

const Ogre::Vector3& HeightROM::GetEndPoint() const
{
	auto& imp_ = *ImpUPtr_;

	auto heiDOM = std::static_pointer_cast<HeightDOM>(GetDOM());
	return heiDOM->GetPoint2();
}

void HeightROM::SetPoint(const Ogre::Vector3& sp, const Ogre::Vector3& ep)
{
	auto& imp_ = *ImpUPtr_;

	//Ogre::Vector3 pt_s(0, sp.y, 0);
	//Ogre::Vector3 pt_e(0, ep.y, 0);
	Ogre::Vector3 pt_s(sp);
	Ogre::Vector3 pt_e(sp.x, ep.y, sp.z);

	imp_.SetPoint(pt_s, pt_e);

	auto heiDOM = std::static_pointer_cast<HeightDOM>( GetDOM() );
	heiDOM->SetPoints(sp, ep);
}

void HeightROM::OnSetVisible(bool val)
{
	auto& imp_ = *ImpUPtr_;

	imp_.Line_->setVisible(val);
}

void HeightROM::OnSetPickingState(EPickingState state)
{
	auto& imp_ = *ImpUPtr_;

	imp_.Line_->SetColor(PickingClr(state));
	imp_.Line_->SetLineWidth(PickingSize(state));

	switch ( state )
	{
	case IROM::EPS_Normal:
	{
		imp_.Line_->setRenderQueueGroup(ERG_Line);
		imp_.Line_->SetColor(NormalClr); //89FF00
		imp_.Line_->SetLineWidth(IROM::PickingSize(state));
	}
	break;
	case IROM::EPS_Sweep:
	{
		imp_.Line_->setRenderQueueGroup(ERG_Line_Sweep);
		imp_.Line_->SetColor(NormalClr); //89FF00
		imp_.Line_->SetLineWidth(IROM::PickingSize(state));
	}
	break;
	case IROM::EPS_Select:
	{
		imp_.Line_->setRenderQueueGroup(ERG_Line_Select);
		imp_.Line_->SetColor(IROM::PickingClr(state));
		imp_.Line_->SetLineWidth(IROM::PickingSize(state));
	}
	break;
	case IROM::EPS_COUNT:
	break;
	default:
	break;
	}
}


void HeightROM::OnSetDisplayMode(EDisplayMode mode)
{
	auto& imp_ = *ImpUPtr_;
	switch (mode)
	{
	case IROM::EDM_Browser:
	{
		imp_.Line_->SetColor(NormalClr); //89FF00
		imp_.Line_->SetLineWidth(IROM::PickingSize(IROM::EPS_Normal));
	}
	break;
	case IROM::EDM_Edit:
	{
		imp_.Line_->SetColor(SelectClr);
		imp_.Line_->SetLineWidth(IROM::PickingSize(IROM::EPS_Select));
	}
	break;
	default:
		break;
	}
}

boost::optional<float> HeightROM::OnGetDistanceToRay(const TopoDS_Edge& ray, float deviation)
{
	auto& imp_ = *ImpUPtr_;

	if ( imp_.Edge_.IsNull() )
	{
		return boost::none;
	}

	BRepExtrema_DistShapeShape dss(ray, imp_.Edge_);

	if ( dss.Value() > deviation )
	{
		return boost::none;
	}

	auto intPnt = dss.PointOnShape1(1);

	BRepAdaptor_Curve bc(ray);
	gp_Pnt fp;
	bc.D0(0, fp);

	return static_cast<float>( fp.Distance(intPnt) );
}

void HeightROM::OnStashSave()
{
	auto& imp_ = *ImpUPtr_;

	auto heiDOM = std::static_pointer_cast<HeightDOM>( GetDOM() );

	imp_.StashP1_ = heiDOM->GetPoint1();
	imp_.StashP2_ = heiDOM->GetPoint2();
}

void HeightROM::OnStashPop()
{
	auto& imp_ = *ImpUPtr_;

	if ( imp_.StashP1_ && imp_.StashP2_ )
	{
		SetPoint(*imp_.StashP1_, *imp_.StashP2_);
	}

	imp_.StashP1_.reset();
	imp_.StashP2_.reset();
}
