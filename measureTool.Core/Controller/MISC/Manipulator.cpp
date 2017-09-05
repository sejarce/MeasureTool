#include "Manipulator.h"

#include "Render/RenderGroupID.h"
#include "Render/PrefabResourceMgr.h"
#include "Render/Extension/Circle3D.h"
#include "Render/Extension/DynamicCurve.h"

#include "Util/MeshUtil.h"
#include "Util/MathUtil.h"

#include "gp_Circ.hxx"
#include "gp_Quaternion.hxx"
#include "gce_MakeLin.hxx"
#include "BRepPrimAPI_MakeCylinder.hxx"
#include "BRepBuilderAPI_MakeEdge.hxx"
#include "BRepExtrema_DistShapeShape.hxx"
#include "BRepTools.hxx"

static const Ogre::ColourValue ZClr(80.f / 255, 227.f / 255, 194.f / 255, .5f);
static const Ogre::ColourValue XClr(1.f, 0, 0, .5f);

class	Manipulator::Imp
{
public:



	Ogre::SceneManager*		Smgr_{};
	Ogre::SceneNode*		Node_{};
	

	Ogre::BillboardSet*		Arrow_{};
	Ogre::Billboard*		ArrowBoard_{};
	Ogre::SceneNode*		ArrowNode_{};

	Ogre::SceneNode*		RingNode_{};
	Ogre::SceneNode*		FaceRadiusNode_{};
	Ogre::SceneNode*		ZFaceInversionNode_{};
	Ogre::SceneNode*		ZFaceAdjNode_{};
	Ogre::SceneNode*		XRingRotNode_{};
	Ogre::SceneNode*		XFaceInversionNode_{};
	Ogre::SceneNode*		XFaceAdjNode_{};
	
	Circle3D*				ZFace_{};
	Circle3D*				XFace_{};
	DynamicCurve*			ZRing_{};
	DynamicCurve*			XRing_{};
	DynamicCurve*			ZBigRing_{};
	DynamicCurve*			XBigRing_{};

	TopoDS_Shape			BaseArrowShape_;
	TopoDS_Shape			BaseZCircleShape_;
	TopoDS_Shape			BaseXCircleShape_;
	TopoDS_Shape			ArrowShape_;
	TopoDS_Shape			ZCircleShape_;
	TopoDS_Shape			XCircleShape_;

	gp_Trsf					ShapeTrans_;
	gp_Trsf					ShapeRot_;

	Ogre::Degree			XDeg_{};
	Ogre::Degree			ZDeg_{};

public:

	void SetState(EObjType obj, IROM::EPickingState state)
	{
		switch ( obj )
		{
		case Manipulator::EOT_Arrow:
		{
			switch ( state )
			{
			case IROM::EPS_Normal:
			{
				Arrow_->setMaterial(PrefabResourceMgr::GetInstance().GetMateiral(PrefabResourceMgr::EPMAT_Manipulator_Arrow));
				ArrowBoard_->setDimensions(.05f, .2f);
			}
			break;
			case IROM::EPS_Sweep:
			{
				Arrow_->setMaterial(PrefabResourceMgr::GetInstance().GetMateiral(PrefabResourceMgr::EPMAT_Manipulator_Arrow));
				ArrowBoard_->setDimensions(.08f, .2f);
			}
			break;
			case IROM::EPS_Select:
			{
				Arrow_->setMaterial(PrefabResourceMgr::GetInstance().GetMateiral(PrefabResourceMgr::EPMAT_Manipulator_Arrow));
				ArrowBoard_->setDimensions(.08f, .2f);
			}
			break;
			case IROM::EPS_COUNT:
			break;
			default:
			break;
			}
		}
		break;
		case Manipulator::EOT_ZRing:
		{
			switch ( state )
			{
			case IROM::EPS_Normal:
			ZRing_->SetColor(ZClr);
			ZRing_->detachFromParent();
			ZBigRing_->detachFromParent();
			RingNode_->attachObject(ZRing_);
			break;
			case IROM::EPS_Sweep:
			ZBigRing_->SetColor(ZClr);
			ZRing_->detachFromParent();
			ZBigRing_->detachFromParent();
			RingNode_->attachObject(ZBigRing_);
			break;
			case IROM::EPS_Select:
			ZBigRing_->SetColor(Ogre::ColourValue::White);
			ZRing_->detachFromParent();
			ZBigRing_->detachFromParent();
			RingNode_->attachObject(ZBigRing_);
			break;
			case IROM::EPS_COUNT:
			break;
			default:
			break;
			}
		}
		break;
		case Manipulator::EOT_XRing:
		{
			switch ( state )
			{
			case IROM::EPS_Normal:
			XRing_->SetColor(XClr);
			XRing_->detachFromParent();
			XBigRing_->detachFromParent();
			RingNode_->attachObject(XRing_);
			break;
			case IROM::EPS_Sweep:
			XBigRing_->SetColor(XClr);
			XRing_->detachFromParent();
			XBigRing_->detachFromParent();
			RingNode_->attachObject(XBigRing_);
			break;
			case IROM::EPS_Select:
			XBigRing_->SetColor(Ogre::ColourValue::White);
			XRing_->detachFromParent();
			XBigRing_->detachFromParent();
			RingNode_->attachObject(XBigRing_);
			break;
			case IROM::EPS_COUNT:
			break;
			default:
			break;
			}
		}
		break;
		default:
		break;
		}
	}

	void	UpdateShape()
	{
		auto trsf = ShapeTrans_ * ShapeRot_;

		auto p = gp::Origin().Transformed(trsf);
		
		ArrowShape_ = BaseArrowShape_.Moved(ShapeTrans_);
		XCircleShape_ = BaseXCircleShape_.Moved(trsf);
		ZCircleShape_ = BaseZCircleShape_.Moved(trsf);
	}

	void	UpdateRotation(const Ogre::Quaternion& curRot)
	{
		auto curX = curRot * Ogre::Vector3::UNIT_X;
		auto curY = curRot * Ogre::Vector3::UNIT_Y;
		auto curZ = curRot * Ogre::Vector3::UNIT_Z;

		RingNode_->setOrientation(curRot);
		ShapeRot_.SetRotation(MathUtil::ToOCCT(curRot));
		UpdateShape();

		auto degxi = static_cast<int>( XDeg_.valueDegrees() + .5f );
		auto degzi = static_cast<int>( ZDeg_.valueDegrees() + .5f );
		
		SetFaceInfo(degxi, degzi);
	}

	void SetFaceInfo(int xDegree, int zDegree)
	{
		XFace_->SetDegree(std::abs(xDegree));
		ZFace_->SetDegree(std::abs(zDegree));

		auto xAdjRot = Ogre::Quaternion::IDENTITY;
		xAdjRot.FromAngleAxis(Ogre::Degree(-xDegree), Ogre::Vector3::UNIT_Z);
		XFaceAdjNode_->setOrientation(xAdjRot);

		auto zAdjRot = Ogre::Quaternion::IDENTITY;
		zAdjRot.FromAngleAxis(Ogre::Degree(-zDegree), Ogre::Vector3::UNIT_Z);
		ZFaceAdjNode_->setOrientation(zAdjRot);

		if ( xDegree > 0 )
		{
			XFaceInversionNode_->setOrientation(Ogre::Quaternion::IDENTITY);
		}
		else
		{
			XFaceInversionNode_->setOrientation(Ogre::Vector3::UNIT_Z.getRotationTo(Ogre::Vector3::NEGATIVE_UNIT_Z));
		}

		if ( zDegree > 0 )
		{
			ZFaceInversionNode_->setOrientation(Ogre::Quaternion::IDENTITY);
		}
		else
		{
			ZFaceInversionNode_->setOrientation(Ogre::Vector3::UNIT_Z.getRotationTo(Ogre::Vector3::NEGATIVE_UNIT_Z));
		}
	}
};

Manipulator::Manipulator(Ogre::SceneNode* parentNode) :ImpUPtr_(std::make_unique<Imp>())
{
	auto& imp_ = *ImpUPtr_;

	imp_.Smgr_ = parentNode->getCreator();
	imp_.Node_ = parentNode->createChildSceneNode();

	{//arrow
		imp_.ArrowNode_ = imp_.Node_->createChildSceneNode();

		imp_.Arrow_ = imp_.Smgr_->createBillboardSet();
		imp_.Arrow_->setBillboardType(Ogre::BBT_ORIENTED_COMMON);
		imp_.Arrow_->setRenderQueueGroup(ERG_Manipulator_Arrow);
		imp_.Arrow_->setMaterial(PrefabResourceMgr::GetInstance().GetMateiral(PrefabResourceMgr::EPMAT_Manipulator_Arrow));
		imp_.Arrow_->setCommonDirection(Ogre::Vector3::UNIT_Y);
		imp_.ArrowBoard_ = imp_.Arrow_->createBillboard(Ogre::Vector3::ZERO);
		imp_.ArrowBoard_->setDimensions(.05f, .2f);

		imp_.ArrowNode_->setPosition(0, imp_.ArrowBoard_->getOwnHeight() / 2, 0);
		imp_.ArrowNode_->attachObject(imp_.Arrow_);
	}

	auto ringRadius = .05f;
	auto curveWidth = .002f;
	auto curveSeg = 8;

	{//node
		imp_.RingNode_ = imp_.Node_->createChildSceneNode();
		imp_.FaceRadiusNode_ = imp_.RingNode_->createChildSceneNode();

		imp_.ZFaceAdjNode_ = imp_.FaceRadiusNode_->createChildSceneNode();
		imp_.ZFaceInversionNode_ = imp_.ZFaceAdjNode_->createChildSceneNode();

		imp_.XRingRotNode_ = imp_.FaceRadiusNode_->createChildSceneNode();
		imp_.XFaceAdjNode_ = imp_.XRingRotNode_->createChildSceneNode();
		imp_.XFaceInversionNode_ = imp_.XFaceAdjNode_->createChildSceneNode();

		imp_.FaceRadiusNode_->setScale(ringRadius * 2, ringRadius * 2, ringRadius * 2);
		imp_.XRingRotNode_->setOrientation(Ogre::Vector3::UNIT_Z.getRotationTo(Ogre::Vector3::UNIT_X));
	}

	{//z ring
		std::vector<Ogre::Vector3> pntList;
		{
			for ( auto deg = 0; deg < 360; deg += 10 )
			{
				Ogre::Degree od(deg);
				pntList.emplace_back(std::cos(od.valueRadians()) * ringRadius, std::sin(od.valueRadians()) * ringRadius, 0.f);
			}
		}

		imp_.ZRing_ = DynamicCurveFactory::CreateInstance(imp_.Smgr_);
		imp_.ZRing_->UpdateCurve(pntList, curveSeg, curveWidth, true, true);
		imp_.ZRing_->setRenderQueueGroup(ERG_Manipulator_Ring);
		
		imp_.RingNode_->attachObject(imp_.ZRing_);

		imp_.ZBigRing_ = DynamicCurveFactory::CreateInstance(imp_.Smgr_);
		imp_.ZBigRing_->UpdateCurve(pntList, curveSeg, curveWidth * 2, true, true);
		imp_.ZBigRing_->setRenderQueueGroup(ERG_Manipulator_Ring);
	}

	{//x ring
		std::vector<Ogre::Vector3> pntList;
		{
			for ( auto deg = 0; deg < 360; deg += 10 )
			{
				Ogre::Degree od(deg);
				pntList.emplace_back(0.f, std::sin(od.valueRadians()) * ringRadius, -std::cos(od.valueRadians()) * ringRadius);
			}
		}

		imp_.XRing_ = DynamicCurveFactory::CreateInstance(imp_.Smgr_);
		imp_.XRing_->UpdateCurve(pntList, curveSeg, curveWidth, true, true);
		imp_.XRing_->setRenderQueueGroup(ERG_Manipulator_Ring);

		imp_.RingNode_->attachObject(imp_.XRing_);

		imp_.XBigRing_ = DynamicCurveFactory::CreateInstance(imp_.Smgr_);
		imp_.XBigRing_->UpdateCurve(pntList, curveSeg, curveWidth * 2, true, true);
		imp_.XBigRing_->setRenderQueueGroup(ERG_Manipulator_Ring);
	}

	{//z face
		imp_.ZFace_ = Circle3DFactory::CreateInstance(imp_.Smgr_);
		imp_.ZFaceInversionNode_->attachObject(imp_.ZFace_);
		
	}

	{//x face
		imp_.XFace_ = Circle3DFactory::CreateInstance(imp_.Smgr_);
		imp_.XFaceInversionNode_->attachObject(imp_.XFace_);
	}

	auto arrowWidth = imp_.ArrowBoard_->getOwnWidth();
	auto arrowHeight = imp_.ArrowBoard_->getOwnHeight();

	BRepPrimAPI_MakeCylinder mc(gp_Ax2(gp::Origin(), gp::DY()), arrowWidth / 4, arrowHeight * 0.8);
	imp_.BaseArrowShape_ = mc.Shape();

	{//z circle
		gp_Ax2 ax(gp::Origin(), gp::DZ());
		gp_Circ c(ax, ringRadius);

		imp_.BaseZCircleShape_ = BRepBuilderAPI_MakeEdge(c).Edge();
	}

	{//z circle
		gp_Ax2 ax(gp::Origin(), gp::DX());
		gp_Circ c(ax, ringRadius);

		imp_.BaseXCircleShape_ = BRepBuilderAPI_MakeEdge(c).Edge();
	}

	imp_.UpdateShape();

	imp_.ZFace_->SetColor(ZClr);
	imp_.XFace_->SetColor(XClr);

	SetRotation(0, 0);

	SetState(EOT_Arrow, IROM::EPS_Normal);
	SetState(EOT_ZRing, IROM::EPS_Normal);
	SetState(EOT_XRing, IROM::EPS_Normal);
	SetVisible(false);
}

Manipulator::~Manipulator()
{
	auto& imp_ = *ImpUPtr_;

	imp_.ZRing_->Destory();
	imp_.ZBigRing_->Destory();
	imp_.XRing_->Destory();
	imp_.XBigRing_->Destory();
	imp_.ZFace_->Destory();
	imp_.XFace_->Destory();

	imp_.Node_->removeAndDestroyAllChildren();
	imp_.Node_->getParentSceneNode()->removeChild(imp_.Node_);
}

void Manipulator::SetPosition(const Ogre::Vector3& pos)
{
	auto& imp_ = *ImpUPtr_;

	imp_.Node_->setPosition(pos);

	imp_.ShapeTrans_.SetTranslationPart(MathUtil::ToOCCTVec(pos));

	imp_.UpdateShape();
}

Ogre::Vector3 Manipulator::GetPosition() const
{
	auto& imp_ = *ImpUPtr_;

	return imp_.Node_->getPosition();
}

Ogre::Quaternion Manipulator::GetOrientation() const
{
	auto& imp_ = *ImpUPtr_;

	return imp_.RingNode_->getOrientation();
}

void Manipulator::SetVisible(bool val)
{
	auto& imp_ = *ImpUPtr_;

	imp_.Node_->setVisible(val);
}

void Manipulator::SetRotation(float xDegree, float zDegree)
{
	auto& imp_ = *ImpUPtr_;

	Ogre::Quaternion rotx;
	rotx.FromAngleAxis(Ogre::Degree(xDegree), Ogre::Vector3::UNIT_X);
	
	auto curZ = rotx * Ogre::Vector3::UNIT_Z;
	
	Ogre::Quaternion rotz;
	rotz.FromAngleAxis(Ogre::Degree(zDegree), curZ);

	auto curRot = rotz * rotx;

	imp_.XDeg_ = xDegree;
	imp_.ZDeg_ = zDegree;

	imp_.UpdateRotation(curRot);
}

std::tuple<float, float> Manipulator::GetRotation() const
{
	auto& imp_ = *ImpUPtr_;

	return std::make_tuple(imp_.XDeg_.valueDegrees(), imp_.ZDeg_.valueDegrees());
}

float Manipulator::GetXDegree() const
{
	auto& imp_ = *ImpUPtr_;

	return ( imp_.XDeg_.valueDegrees() + .5f );
}

float Manipulator::GetZDegree() const
{
	auto& imp_ = *ImpUPtr_;

	return ( imp_.ZDeg_.valueDegrees() + .5f );
}

void Manipulator::RotateX(float rad)
{
	auto& imp_ = *ImpUPtr_;

	auto curRot = imp_.RingNode_->getOrientation();
	auto curAxe = curRot * Ogre::Vector3::UNIT_X;

	Ogre::Quaternion qua;
	qua.FromAngleAxis(Ogre::Radian(rad), curAxe);

	auto rot = qua * curRot;

	imp_.XDeg_ += Ogre::Radian(rad);

	imp_.UpdateRotation(rot);
}

void Manipulator::RotateZ(float rad)
{
	auto& imp_ = *ImpUPtr_;

	auto curRot = imp_.RingNode_->getOrientation();
	auto curAxe = curRot * Ogre::Vector3::UNIT_Z;

	Ogre::Quaternion qua;
	qua.FromAngleAxis(Ogre::Radian(rad), curAxe);

	auto rot = qua * curRot;

	imp_.ZDeg_ += Ogre::Radian(rad);

	imp_.UpdateRotation(rot);
}

void Manipulator::SetState(EObjType obj, IROM::EPickingState state)
{
	auto& imp_ = *ImpUPtr_;

	imp_.SetState(obj, state);
}

boost::optional<Manipulator::EObjType> Manipulator::QueryRay(const Ogre::Ray& ray)
{
	auto& imp_ = *ImpUPtr_;

	gce_MakeLin gml(MathUtil::ToOCCTPnt(ray.getOrigin()), MathUtil::ToOCCTDir(ray.getDirection()));
	BRepBuilderAPI_MakeEdge me(gml.Value());
	auto rayEdge = me.Edge();

	
	BRepExtrema_DistShapeShape dss2(imp_.ZCircleShape_, rayEdge);
	BRepExtrema_DistShapeShape dss3(imp_.XCircleShape_, rayEdge);

	std::multimap<float, EObjType> mm;

	{
		BRepExtrema_DistShapeShape dss(imp_.ArrowShape_, rayEdge);
		if ( dss.Value() < 5e-3 )
		{
			auto p1 = dss.PointOnShape2(1);
			mm.emplace(MathUtil::ToOgre(p1).squaredDistance(ray.getOrigin()), EOT_Arrow);
		}
	}

	{
		BRepExtrema_DistShapeShape dss(imp_.ZCircleShape_, rayEdge);
		if ( dss.Value() < 15e-3 )
		{
			auto p1 = dss.PointOnShape2(1);
			mm.emplace(MathUtil::ToOgre(p1).squaredDistance(ray.getOrigin()), EOT_ZRing);
		}
	}

	{
		BRepExtrema_DistShapeShape dss(imp_.XCircleShape_, rayEdge);
		if ( dss.Value() < 15e-3 )
		{
			auto p1 = dss.PointOnShape2(1);
			mm.emplace(MathUtil::ToOgre(p1).squaredDistance(ray.getOrigin()), EOT_XRing);
		}
	}

	if ( mm.empty() )
	{
		return boost::none;
	}

	return mm.begin()->second;
}
