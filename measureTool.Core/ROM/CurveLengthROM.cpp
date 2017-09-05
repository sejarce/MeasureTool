#include "CurveLengthROM.h"

#include "DOM/CurveLengthDOM.h"

#include "Render/RenderGroupID.h"
#include "Render/Extension/DynamicCurve.h"

#include "Util/MathUtil.h"
#include "Util/MeshUtil.h"

#include "TopoDS_Edge.hxx"
#include "BRepBuilderAPI_MakeEdge.hxx"
#include "BRepExtrema_DistShapeShape.hxx"
#include "BRepAdaptor_Curve.hxx"

#include "GProp_GProps.hxx"
#include "GeomAPI_PointsToBSpline.hxx"
#include "Geom_BSplineCurve.hxx"
#include "BrepTools.hxx"
#include "BRep_Builder.hxx"
#include "BRepGProp.hxx"
#include "BRepBuilderAPI_MakeVertex.hxx"


#include "Ogre.h"

class	CurveLengthROM::Imp
{
public:

	DynamicCurve*		DynamicCurve_{};
	DynamicCurve*		ThickDynamicCurve_{};
	Ogre::SceneNode*	Node_{};
	Ogre::SceneManager*	Smgr_{};

	std::vector<Ogre::Vector3>	FittingPntList_;
	boost::optional<Pnt3D>		StartPos_;
	boost::optional<Pnt3D>		EndPos_;
	boost::signals2::connection	Conn_;
};

static const auto MaxQueryMeshSize = 3000;

static const auto NormalClr = Ogre::ColourValue(1.f, 118.f / 255.f, 0.5f);
static const auto SelectClr = Ogre::ColourValue(238.f / 255.f, 103.f / 255.f, 35.f / 255.f);	//238,103,35

CurveLengthROM::CurveLengthROM(Ogre::SceneNode* rootNode, const IDOMSPtr& dom) :TROM(rootNode, dom)
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
	//{
	//	imp_.Node_->attachObject(imp_.ThickDynamicCurve_);
	//	imp_.ThickDynamicCurve_->SetColor(NormalClr);
	//}

	auto opencurveDOM = std::static_pointer_cast<CurveLengthDOM>(dom);

	imp_.StartPos_ = opencurveDOM->GetStartPos();
	imp_.EndPos_ = opencurveDOM->GetEndPos();
	{
		auto edge = opencurveDOM->GetEdge();
		if (!edge.IsNull())
		{
			Standard_Real f, l;
			auto curve = BRep_Tool::Curve(edge, f, l);
			auto FittingCurve = Handle(Geom_BSplineCurve)::DownCast(curve);
			const int buildCount = 300;
			auto step = 1.f / (buildCount - 1);
			gp_Pnt pnt;
			for (auto index = 0; index < buildCount; ++index)
			{
				FittingCurve->D0(step*index, pnt);
				Ogre::Vector3 v{ static_cast<float>(pnt.X()), static_cast<float>(pnt.Y()), static_cast<float>(pnt.Z()) };
				imp_.FittingPntList_.emplace_back(v);
			}
		}
	}

	if (!opencurveDOM->GetPntList().empty())
	{
		//Update(opencurveDOM->GetPntList(), opencurveDOM->GetPlnNormal());
		Update(imp_.StartPos_.get(), imp_.EndPos_.get(), imp_.FittingPntList_);
	}
}

CurveLengthROM::~CurveLengthROM()
{
	auto& imp_ = *ImpUPtr_;

	imp_.DynamicCurve_->Destory();
	imp_.ThickDynamicCurve_->Destory();
	
	imp_.Node_->removeAndDestroyAllChildren();
	imp_.Node_->getParentSceneNode()->removeChild(imp_.Node_);

	imp_.Conn_.disconnect();
}

void CurveLengthROM::Update(const Pnt3D& startpos,
	const Pnt3D& endpos,
	const std::vector<Ogre::Vector3>& plnList)
{
	auto& imp_ = *ImpUPtr_;
	imp_.StartPos_ = startpos;
	imp_.EndPos_ = endpos;

	auto dom = std::static_pointer_cast<CurveLengthDOM>(GetDOM());
	dom->UpdateCurve(startpos, endpos, plnList);

	imp_.FittingPntList_ = dom->GetPntList();


	imp_.DynamicCurve_->UpdateCurve(imp_.FittingPntList_, 6, PickingSize(IROM::EPS_Normal) / 2, false, false);
	imp_.ThickDynamicCurve_->UpdateCurve(imp_.FittingPntList_, 6, PickingSize(IROM::EPS_Sweep) / 2, false, false);
}

const std::vector<Ogre::Vector3>& CurveLengthROM::GetFittingList() const
{
	auto& imp_ = *ImpUPtr_;

	return imp_.FittingPntList_;
}

const CurveLengthROM::Pnt3D& CurveLengthROM::GetStartPos() const
{
	auto& imp_ = *ImpUPtr_;

	return imp_.StartPos_.get();
}

const CurveLengthROM::Pnt3D& CurveLengthROM::GetEndPos() const
{
	auto& imp_ = *ImpUPtr_;

	return imp_.EndPos_.get();
}

void CurveLengthROM::SaveToFile(const std::wstring& fileName)
{
	auto& imp_ = *ImpUPtr_;

	imp_.DynamicCurve_->SaveToFile(fileName);
}

void CurveLengthROM::OnSetVisible(bool val)
{
	auto& imp_ = *ImpUPtr_;

	imp_.Node_->setVisible(val);
}

void CurveLengthROM::OnSetPickingState(EPickingState state)
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

void CurveLengthROM::OnSetDisplayMode(EDisplayMode mode)
{
	auto& imp_ = *ImpUPtr_;
	imp_.DynamicCurve_->detachFromParent();
	imp_.ThickDynamicCurve_->detachFromParent();
	imp_.Node_->attachObject(imp_.DynamicCurve_);
	imp_.Node_->attachObject(imp_.ThickDynamicCurve_);

	switch (mode)
	{
	case IROM::EDM_Browser:
	{
		imp_.DynamicCurve_->SetColor(NormalClr);
		imp_.DynamicCurve_->setRenderQueueGroup(ERG_Curve);
		//imp_.Line_->SetLineWidth(0.002f);
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

boost::optional<float> CurveLengthROM::OnGetDistanceToRay(const TopoDS_Edge& ray, float deviation)
{
	auto& imp_ = *ImpUPtr_;

	auto dom = std::static_pointer_cast<CurveLengthDOM>(GetDOM());
	auto edge = dom->GetEdge();
	if (edge.IsNull())
	{
		return boost::none;
	}

	BRepExtrema_DistShapeShape dss(ray, edge);

	if (dss.Value() > deviation)
	{
		return boost::none;
	}

	auto intPnt = dss.PointOnShape1(1);

	BRepAdaptor_Curve bc(ray);
	gp_Pnt fp;
	bc.D0(0, fp);

	return static_cast<float>(fp.Distance(intPnt));
}

void CurveLengthROM::OnStashSave()
{
	auto& imp_ = *ImpUPtr_;

	auto dom = std::static_pointer_cast<CurveLengthDOM>(GetDOM());

	imp_.StartPos_ = dom->GetStartPos();
	imp_.EndPos_ = dom->GetEndPos();
}

void CurveLengthROM::OnStashPop()
{
	auto& imp_ = *ImpUPtr_;

	if (imp_.StartPos_ && imp_.EndPos_)
	{
		Update(imp_.StartPos_.get(), imp_.EndPos_.get(), imp_.FittingPntList_);
	}

	imp_.StartPos_.reset();
	imp_.EndPos_.reset();
}
