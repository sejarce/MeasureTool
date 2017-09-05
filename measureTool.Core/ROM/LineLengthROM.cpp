#include "LineLengthROM.h"

#include "DOM/LineLengthDOM.h"

#include "Util/MathUtil.h"

#include "Render/Extension/Line3D.h"
#include "Render/Extension/Point3D.h"
#include "Render/RenderGroupID.h"

#include "TopoDS_Edge.hxx"
#include "BRepBuilderAPI_MakeEdge.hxx"
#include "BRepExtrema_DistShapeShape.hxx"
#include "BRepAdaptor_Curve.hxx"

#include "Ogre.h"

class	LineLengthROM::Imp
{
public:

	Line3D*		Line_{};
	Ogre::Vector3	StartPoint_{};
	Ogre::Vector3	EndPoint_{};
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

static const auto NormalClr = Ogre::ColourValue(1.f, 0.f, 181.0f / 255.0f);	//FF00B5
static const auto SelectClr = Ogre::ColourValue(238.f / 255.f, 103.f / 255.f, 35.f / 255.f);	//238,103,35

LineLengthROM::LineLengthROM(Ogre::SceneNode* rootNode, const IDOMSPtr& dom) :TROM(rootNode, dom)
,ImpUPtr_(std::make_unique<Imp>())
{
	auto& imp_ = *ImpUPtr_;

	imp_.Line_ = Line3DFactory::CreateInstance(GetSmgr());		

	imp_.Node_ = GetRootNode()->createChildSceneNode();
	imp_.Node_->setName("LineSegmentROM");

	imp_.Node_->attachObject(imp_.Line_);	

	imp_.Line_->setVisible(false);

	auto lenDOM = std::static_pointer_cast<LineLengthDOM>( dom );

	imp_.StartPoint_ = lenDOM->GetPoint1();
	imp_.EndPoint_ = lenDOM->GetPoint2();

	imp_.SetPoint(imp_.StartPoint_, imp_.EndPoint_);
}

LineLengthROM::~LineLengthROM()
{
	auto& imp_ = *ImpUPtr_;

	imp_.Line_->Destory();

	imp_.Node_->getParentSceneNode()->removeChild(imp_.Node_);
}

const Ogre::Vector3& LineLengthROM::GetStartPoint() const
{
	auto& imp_ = *ImpUPtr_;

	return imp_.Line_->GetStartPoint();
}

const Ogre::Vector3& LineLengthROM::GetEndPoint() const
{
	auto& imp_ = *ImpUPtr_;

	return imp_.Line_->GetEndPoint();
}

void LineLengthROM::SetPoint(const Ogre::Vector3& start, const Ogre::Vector3& end)
{
	auto& imp_ = *ImpUPtr_;

	imp_.SetPoint(start, end);

	auto dom = std::static_pointer_cast<LineLengthDOM>( GetDOM() );
	dom->SetPoints(start, end);
}

void LineLengthROM::OnSetVisible(bool val)
{
	auto& imp_ = *ImpUPtr_;

	imp_.Line_->setVisible(val);
}

void LineLengthROM::OnSetPickingState(EPickingState state)
{
	auto& imp_ = *ImpUPtr_;

	imp_.Line_->SetColor(PickingClr(state));

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
	
void LineLengthROM::OnSetDisplayMode(EDisplayMode mode)
{
	auto& imp_ = *ImpUPtr_;
	switch ( mode )
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

boost::optional<float> LineLengthROM::OnGetDistanceToRay(const TopoDS_Edge& ray, float deviation)
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

void LineLengthROM::OnStashSave()
{
	auto& imp_ = *ImpUPtr_;

	auto heiDOM = std::static_pointer_cast<LineLengthDOM>( GetDOM() );

	imp_.StashP1_ = heiDOM->GetPoint1();
	imp_.StashP2_ = heiDOM->GetPoint2();
}

void LineLengthROM::OnStashPop()
{
	auto& imp_ = *ImpUPtr_;

	if ( imp_.StashP1_ && imp_.StashP2_ )
	{
		SetPoint(*imp_.StashP1_, *imp_.StashP2_);
	}

	imp_.StashP1_.reset();
	imp_.StashP2_.reset();
}
