#include "DimensionROM.h"

#include "DOM/DimensionDOM.h"

#include "Render/RenderGroupID.h"
#include "Render/Extension/DynamicCurve.h"

#include "Util/MathUtil.h"
#include "Util/MeshUtil.h"

#include "BRepExtrema_DistShapeShape.hxx"
#include "BRepAdaptor_Curve.hxx"

#include "Ogre.h"

class	DimensionROM::Imp
{
public:

	DynamicCurve*		DynamicCurve_{};
	DynamicCurve*		ThickDynamicCurve_{};
	Ogre::SceneNode*	Node_{};
	Ogre::SceneManager*	Smgr_{};

	std::vector<Ogre::Vector3>	FittingPntList_;
	Ogre::Vector3		PlaneNormal_;
	Ogre::Vector3		Center_;
	float				XDeg_;
	float				ZDeg_;
	boost::signals2::connection	Conn_;

	boost::optional<std::vector<Ogre::Vector3>> StashPlnList_;
	boost::optional<Ogre::Vector3>				StashPlnNormal_;
};

static const auto MaxQueryMeshSize = 3000;

static const auto NormalClr = Ogre::ColourValue(1.f, 118.f / 255, 0.f);
static const auto SelectClr = Ogre::ColourValue(238.f / 255.f, 103.f / 255.f, 35.f / 255.f);	//238,103,35

DimensionROM::DimensionROM(Ogre::SceneNode* rootNode, const IDOMSPtr& dom) :TROM(rootNode, dom)
,ImpUPtr_(std::make_unique<Imp>())
{
	auto& imp_ = *ImpUPtr_;
	imp_.Smgr_ = GetRootNode()->getCreator();
	imp_.Node_ = GetRootNode()->createChildSceneNode();
	imp_.Node_->setName("CurveROM");

	imp_.DynamicCurve_ = DynamicCurveFactory::CreateInstance(imp_.Smgr_);
	{
		imp_.Node_->attachObject(imp_.DynamicCurve_);
		imp_.DynamicCurve_->SetColor(NormalClr);
	}

	imp_.ThickDynamicCurve_ = DynamicCurveFactory::CreateInstance(imp_.Smgr_);

	auto curveDOM = std::static_pointer_cast<DimensionDOM>( dom );

	imp_.PlaneNormal_ = curveDOM->GetPlnNormal();
	imp_.Center_ = curveDOM->GetPlnPos();

	if ( !curveDOM->GetPntList().empty() )
	{
		Update(curveDOM->GetPntList(), curveDOM->GetPlnNormal());
	}
}

DimensionROM::~DimensionROM()
{
	auto& imp_ = *ImpUPtr_;

	imp_.DynamicCurve_->Destory();
	imp_.ThickDynamicCurve_->Destory();
	
	imp_.Node_->removeAndDestroyAllChildren();
	imp_.Node_->getParentSceneNode()->removeChild(imp_.Node_);

	imp_.Conn_.disconnect();
}

void DimensionROM::Update(const std::vector<Ogre::Vector3>& plnList, const Ogre::Vector3& plnNormal)
{
	auto& imp_ = *ImpUPtr_;

	imp_.PlaneNormal_ = plnNormal;

	auto dom = std::static_pointer_cast<DimensionDOM>( GetDOM() );
	dom->UpdateCurve(imp_.PlaneNormal_, plnList);
	
	imp_.Center_ = dom->GetPlnPos();
	imp_.FittingPntList_ = dom->GetPntList();

	imp_.DynamicCurve_->UpdateCurve(imp_.FittingPntList_, 6, PickingSize(IROM::EPS_Normal) / 2, true, false);
	imp_.ThickDynamicCurve_->UpdateCurve(imp_.FittingPntList_, 6, PickingSize(IROM::EPS_Sweep) / 2, true, false);
}

const std::vector<Ogre::Vector3>& DimensionROM::GetFittingList() const
{
	auto& imp_ = *ImpUPtr_;

	return imp_.FittingPntList_;
}

const Ogre::Vector3& DimensionROM::GetCenter() const
{
	auto& imp_ = *ImpUPtr_;

	return imp_.Center_;
}

const Ogre::Vector3& DimensionROM::GetNormal() const
{
	auto& imp_ = *ImpUPtr_;

	return imp_.PlaneNormal_;
}

std::tuple<float, float> DimensionROM::GetRotation() const
{
	auto& imp_ = *ImpUPtr_;

	return std::make_tuple(imp_.XDeg_, imp_.ZDeg_);
}

void DimensionROM::SetRotation(float xdeg, float zdeg)
{
	auto& imp_ = *ImpUPtr_;

	imp_.XDeg_ = xdeg;
	imp_.ZDeg_ = zdeg;
}

void DimensionROM::SaveToFile(const std::wstring& fileName)
{
	auto& imp_ = *ImpUPtr_;

	imp_.DynamicCurve_->SaveToFile(fileName);
}

void DimensionROM::OnSetVisible(bool val)
{
	auto& imp_ = *ImpUPtr_;

	imp_.Node_->setVisible(val);
}

void DimensionROM::OnSetPickingState(EPickingState state)
{
	auto& imp_ = *ImpUPtr_;

	imp_.DynamicCurve_->detachFromParent();
	imp_.ThickDynamicCurve_->detachFromParent();

	switch ( state )
	{
	case IROM::EPS_Normal:
	{
		imp_.Node_->attachObject(imp_.DynamicCurve_);
		imp_.DynamicCurve_->SetColor(NormalClr);
		imp_.DynamicCurve_->setRenderQueueGroup(ERG_Curve);
	}
	break;
	case IROM::EPS_Sweep:
	{
		imp_.Node_->attachObject(imp_.ThickDynamicCurve_);
		imp_.ThickDynamicCurve_->SetColor(NormalClr);
		imp_.DynamicCurve_->setRenderQueueGroup(ERG_Curve_Sweep);
	}
	break;
	case IROM::EPS_Select:
	{
		imp_.Node_->attachObject(imp_.ThickDynamicCurve_);
		imp_.ThickDynamicCurve_->SetColor(PickingClr(IROM::EPS_Select));
		imp_.DynamicCurve_->setRenderQueueGroup(ERG_Curve_Select);
	}
	break;
	case IROM::EPS_COUNT:
	break;
	default:
	break;
	}
}

void DimensionROM::OnSetDisplayMode(EDisplayMode mode)
{
	auto& imp_ = *ImpUPtr_;

	imp_.DynamicCurve_->detachFromParent();
	imp_.ThickDynamicCurve_->detachFromParent();
	imp_.Node_->attachObject(imp_.DynamicCurve_);

	switch (mode)
	{
	case IROM::EDM_Browser:
	{	
		imp_.DynamicCurve_->SetColor(NormalClr);
		imp_.DynamicCurve_->setRenderQueueGroup(ERG_Curve);
	}
	break;
	case IROM::EDM_Edit:
	{
		imp_.DynamicCurve_->SetColor(IROM::PickingClr(IROM::EPS_Select));
		imp_.DynamicCurve_->setRenderQueueGroup(ERG_Aux_FittingFace);
	}
	break;
	default:
		break;
	}
}

boost::optional<float> DimensionROM::OnGetDistanceToRay(const TopoDS_Edge& ray, float deviation)
{
	auto& imp_ = *ImpUPtr_;

	auto dom = std::static_pointer_cast<DimensionDOM>( GetDOM() );
	auto edge = dom->GetEdge();
	if ( edge.IsNull() )
	{
		return boost::none;
	}

	BRepExtrema_DistShapeShape dss(ray, edge);

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

void DimensionROM::OnStashSave()
{
	auto& imp_ = *ImpUPtr_;

	auto dom = std::static_pointer_cast<DimensionDOM>( GetDOM() );

	imp_.StashPlnList_ = dom->GetPntList();
	imp_.StashPlnNormal_ = dom->GetPlnNormal();
}

void DimensionROM::OnStashPop()
{
	auto& imp_ = *ImpUPtr_;

	auto dom = std::static_pointer_cast<DimensionDOM>( GetDOM() );

	if ( imp_.StashPlnList_ && imp_.StashPlnNormal_ )
	{
		Update(*imp_.StashPlnList_, *imp_.StashPlnNormal_);
	}

	imp_.StashPlnList_.reset();
	imp_.StashPlnNormal_.reset();
}
