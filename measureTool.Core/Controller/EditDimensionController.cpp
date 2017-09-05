#include "EditDimensionController.h"

#include "Render/Camera/MayaCamera.h"
#include "Render/VisibilityFlag.h"
#include "Render/QueryUtil.h"
#include "Render/RenderGroupID.h"
#include "Render/PrefabResourceMgr.h"
#include "Render/Extension/Circle3D.h"
#include "Render/Extension/Torus.h"
#include "Render/Extension/DynamicPlane.h"
#include "Render/Extension/DynamicPointSet.h"

#include "FrameEvent/ToolBarEvent.h"
#include "FrameEvent/ControlEvent.h"

#include "ROM/DocumentROM.h"
#include "ROM/DimensionROM.h"
#include "ROM/IROM.h"
#include "DOM/IDOM.h"

#include "Util/MathUtil.h"
#include "Util/CpuTimer.h"
#include "Util/PCLOctree.h"
#include "Util/DebugUtil.h"
#include "Util/MeshUtil.h"

#include "MISC/Manipulator.h"

#include "Ogre.h"
#include "OgreMeshSerializer.h"

#include "gp_Circ.hxx"
#include "gce_MakeLin.hxx"
#include "BRepPrimAPI_MakeCylinder.hxx"
#include "BRepBuilderAPI_MakeEdge.hxx"
#include "BRepExtrema_DistShapeShape.hxx"
#include "BRepTools.hxx"

class 	EditDimensionController::Imp
{
public:

	enum EState
	{
		ES_Init,
		ES_Create,
		ES_Edit_Browser,
		ES_Edit_MoveArrow,
		ES_Edit_Ring,
		ES_Edit_Rot
	};

public:

	EditDimensionController*	ThisPtr_{};
	EState					State_ = ES_Init;
	bool					NeedQuery_{ true };

	Ogre::RenderWindow*		RT_{};
	MayaCameraSPtr 			CameraListener_{};
	Ogre::SceneManager*		Smgr_{};

	Ogre::SceneNode*		HumanNode_{};
	Ogre::Matrix4			HumanNodeInvTrans_;
	Ogre::Quaternion		HumanNodeInvRot_;

	PCLOctreeSPtr			Octree_;
	//DimensionROM::SPtr			Rom_;
	std::shared_ptr<DimensionROM>	Rom_;

	Ogre::SceneNode*		EditNode_{};

	Ogre::SceneNode*		DbgQueryPointNode_{};
	DynamicPointSet*		DbgQueryPointSet_{};

	Ogre::SceneNode*		DbgSamplerPointNode_{};
	DynamicPointSet*		DbgFittingPointSet_{};

	DynamicPlane*			QueryPlane_{};

	std::unique_ptr<Manipulator>	Manipulator_;
	boost::optional<Manipulator::EObjType>	Sweep_;

	Ogre::Vector3				LastDragPnt_;

	std::vector<Ogre::Vector3>	SampleDirList_;

	std::vector<Ogre::Vector3>	CurQueryList_;
	OIS::MouseState				LastMouseState_;



public:

	void	UpdatePlnData(const Ogre::Plane& pln, const Ogre::Vector3& plnPos)
	{
		Rom_->SetVisible(false);
		QueryPlane_->setVisible(false);

		auto plnList = Octree_->PlaneQuery(pln, plnPos, 9e-3);
		if ( plnList.empty() )
		{
			return;
		}

		DbgQueryPointSet_->Update(plnList);

		static const auto sampleEps = 5e-3;
		auto sqSampleEps = sampleEps * sampleEps;

		Ogre::Vector3 centroid = Ogre::Vector3::ZERO;
		for ( auto& cur : plnList )
		{
			centroid += cur;
		}
		centroid /= plnList.size();

		std::vector<Ogre::Vector3> frontList, backList;

		std::vector<Ogre::Vector3> tmpFrontList, tmpBackList;
		Ogre::Vector3 tmpFrontCentroid = Ogre::Vector3::ZERO;
		Ogre::Vector3 tmpBackCentroid = Ogre::Vector3::ZERO;

		for ( auto& curSampleDir : SampleDirList_ )
		{
			tmpFrontList.clear();
			tmpBackList.clear();
			tmpFrontCentroid = Ogre::Vector3::ZERO;
			tmpBackCentroid = Ogre::Vector3::ZERO;

			auto curAdjSampleDir = Ogre::Vector3::UNIT_Y.getRotationTo(pln.normal) * curSampleDir;

			for ( auto& curPlnPnt : plnList )
			{
				auto projPnt = MathUtil::ProjectPointOnLine(curPlnPnt, centroid, curAdjSampleDir);
				auto sd = curPlnPnt.squaredDistance(projPnt);
				if ( sd > sqSampleEps )
				{
					continue;
				}

				auto curDir = projPnt - centroid;
				if ( curDir.dotProduct(curAdjSampleDir) > 0 )
				{
					tmpFrontList.push_back(projPnt);
					tmpFrontCentroid += projPnt;
				}
				else
				{
					tmpBackList.push_back(projPnt);
					tmpBackCentroid += projPnt;
				}
			}

			if ( !tmpFrontList.empty() )
			{
				//取中间点
				//tmpFrontCentroid /= tmpFrontList.size();
				//取外层点（最远点）
				float maxSd = 0.0;
				for (auto curr : tmpFrontList)
				{
					auto sd = curr.squaredDistance(centroid);
					if (sd > maxSd)
					{
						maxSd = sd;
						tmpFrontCentroid = curr;
					}
				}
				frontList.push_back(tmpFrontCentroid);
			}

			if ( !tmpBackList.empty() )
			{
				//取中间点
				//tmpBackCentroid /= tmpBackList.size();
				//取外层点（最远点）
				float maxSd = 0.0;
				for (auto curr : tmpBackList)
				{
					auto sd = curr.squaredDistance(centroid);
					if (sd > maxSd)
					{
						maxSd = sd;
						tmpBackCentroid = curr;
					}
				}
				backList.push_back(tmpBackCentroid);
			}
		}

		if ( frontList.empty() && backList.empty() )
		{
			return;
		}

		CurQueryList_.swap(frontList);
		std::copy(backList.begin(), backList.end(), std::back_inserter(CurQueryList_));


		Rom_->Update(CurQueryList_, pln.normal);

		auto& fittingList = Rom_->GetFittingList();

		DbgFittingPointSet_->Update(fittingList);

		QueryPlane_->UpdatePln(fittingList);

		QueryPlane_->setVisible(true);
		Rom_->SetVisible(true);
	}

	boost::optional<Ogre::Vector3>	GetRayOnArrowPln(const Ogre::Ray& ray, OIS::MouseState& mouse)
	{
		Ogre::Plane arrowPln(CameraListener_->GetDir(), Manipulator_->GetPosition());
		auto curInt = Ogre::Math::intersects(ray, arrowPln);
		if ( !curInt.first )
		{
			return boost::none;
		}

		return ray.getPoint(curInt.second);
	}
};

EditDimensionController::EditDimensionController(Ogre::RenderWindow* rt
											   , const MayaCameraSPtr& camera
											   , Ogre::SceneNode* node
											   , PCLOctreeSPtr octree) :ImpUPtr_(new Imp)
{
	auto& imp_ = *ImpUPtr_;

	imp_.ThisPtr_ = this;
	imp_.RT_ = rt;
	imp_.CameraListener_ = camera;
	imp_.Smgr_ = camera->GetCamera()->getSceneManager();
	imp_.HumanNode_ = node;
	imp_.Octree_ = octree;

	imp_.HumanNodeInvTrans_ = imp_.HumanNode_->_getFullTransformUpdated().inverse();
	imp_.HumanNodeInvRot_ = imp_.HumanNodeInvTrans_.extractQuaternion();

	imp_.EditNode_ = imp_.Smgr_->getRootSceneNode()->createChildSceneNode();
	imp_.EditNode_->setPosition(imp_.HumanNode_->getPosition());

	imp_.DbgQueryPointNode_ = imp_.EditNode_->createChildSceneNode();
	{
		imp_.DbgQueryPointSet_ = DynamicPointSetFactory::CreateInstance(imp_.Smgr_);
		imp_.DbgQueryPointSet_->SetColor(Ogre::ColourValue::Red);

		imp_.DbgQueryPointNode_->attachObject(imp_.DbgQueryPointSet_);

		imp_.DbgQueryPointNode_->setVisible(false);
	}

	imp_.DbgSamplerPointNode_ = imp_.EditNode_->createChildSceneNode();
	{
		imp_.DbgFittingPointSet_ = DynamicPointSetFactory::CreateInstance(imp_.Smgr_);
		imp_.DbgFittingPointSet_->SetColor(Ogre::ColourValue::Green);

		imp_.DbgSamplerPointNode_->attachObject(imp_.DbgFittingPointSet_);

		imp_.DbgSamplerPointNode_->setVisible(false);
	}

	imp_.QueryPlane_ = DynamicPlaneFactory::CreateInstance(imp_.Smgr_);
	{
		imp_.EditNode_->attachObject(imp_.QueryPlane_);
		auto& clr = IROM::PickingClr(IROM::EPS_Select);
		imp_.QueryPlane_->SetColor(Ogre::ColourValue(clr.r, clr.g, clr.b, clr.a / 2));
		imp_.QueryPlane_->setVisible(false);
	}

	auto step = 3;
	auto haflCount = 180 / step;
	std::vector<Ogre::Vector3> frontSamples, backSamples;
	frontSamples.reserve(haflCount);
	backSamples.reserve(haflCount);

	for ( auto deg = 0; deg < haflCount; ++deg )
	{
		auto curDir = deg * step;
		Ogre::Quaternion qua;
		qua.FromAngleAxis(Ogre::Radian(Ogre::Degree(deg * 3)), Ogre::Vector3::UNIT_Y);
		auto dir = qua * Ogre::Vector3::UNIT_X;

		imp_.SampleDirList_.push_back(dir);
	}

	imp_.Manipulator_ = std::make_unique<Manipulator>(imp_.EditNode_);
}

EditDimensionController::~EditDimensionController()
{
	Unload();

	auto& imp_ = *ImpUPtr_;

	imp_.DbgQueryPointSet_->Destory();
	imp_.DbgFittingPointSet_->Destory();
	imp_.QueryPlane_->Destory();

	imp_.Manipulator_.reset();

	imp_.EditNode_->removeAndDestroyAllChildren();
	imp_.EditNode_->getParentSceneNode()->removeAndDestroyChild(imp_.EditNode_);

	imp_.Rom_->SetDisplayMode(IROM::EDM_Browser);
}

void EditDimensionController::_FrameQueue(const Ogre::FrameEvent& fevt)
{
	auto& imp_ = *ImpUPtr_;

	auto& evtRecorder = GetSysEventRecorder();

	if ( evtRecorder.HasKeyPressed(OIS::KC_F1) )
	{
		static auto dbg = true;
		imp_.DbgQueryPointNode_->setVisible(dbg);
		dbg = !dbg;
	}

	if ( evtRecorder.HasKeyPressed(OIS::KC_F2) )
	{
		static auto dbg = true;
		imp_.DbgSamplerPointNode_->setVisible(dbg);
		dbg = !dbg;
	}

	if ( evtRecorder.HasKeyPressed(OIS::KC_F3) )
	{
		MeshUtil::SaveToPly(L"query.ply", imp_.CurQueryList_, {});
		MeshUtil::SaveToPly(L"fitting.ply", imp_.Rom_->GetFittingList(), {});
	}

	auto cursorRay = QueryUtil::GetRayFromCamera(imp_.CameraListener_->GetCamera(), SysEventRecorder::CachedMouseState(), imp_.RT_);
	Ogre::Ray adjRay(imp_.HumanNodeInvTrans_ * cursorRay.getOrigin(), imp_.HumanNodeInvRot_ * cursorRay.getDirection());

	switch ( imp_.State_ )
	{
	case Imp::ES_Init:
	{
		auto evt = PopFrameEvent<SFE_EditItem>();

		imp_.Rom_ = std::static_pointer_cast<DimensionROM>( IROM::GetFromDOM(evt->DOM.lock()) );
		imp_.Rom_->SetDisplayMode(IROM::EDM_Edit);

		if ( evt->Status == SFE_EditItem::ES_CreateMode )
		{
			imp_.State_ = Imp::ES_Create;
		}
		else
		{
			imp_.Manipulator_->SetPosition(imp_.Rom_->GetCenter());
			auto rot = imp_.Rom_->GetRotation();
			imp_.Manipulator_->SetRotation(std::get<0>(rot), std::get<1>(rot));

			imp_.State_ = Imp::ES_Edit_Browser;
		}

	}
	break;
	case Imp::ES_Create:
	{
		if ( evtRecorder.HasMousePressed(OIS::MB_Left) )
		{
			if ( !imp_.CurQueryList_.empty() )
			{
				Ogre::AxisAlignedBox box;
				for ( auto& cur : imp_.CurQueryList_ )
				{
					box.merge(cur);
				}

				imp_.Manipulator_->SetPosition(box.getCenter());

				imp_.State_ = Imp::ES_Edit_Browser;
				break;
			}
		}

		if ( evtRecorder.HasMouseMoved() )
		{
			imp_.NeedQuery_ = true;
		}

		if ( !imp_.NeedQuery_ )
		{
			return;
		}

		imp_.NeedQuery_ = false;

		auto queryPnt = imp_.Octree_->RayQuery(adjRay, 5e-3);
		if (!std::get<0>(queryPnt))
		{
			return;
		}

		Ogre::Plane pln(Ogre::Vector3::UNIT_Y, std::get<2>(queryPnt));

		imp_.UpdatePlnData(pln, std::get<2>(queryPnt));

		auto dirty = imp_.Rom_->GetDOM()->isDirty();
		imp_.Rom_->GetDOM()->Dirty();
		imp_.Rom_->GetDOM()->UpdateEditTime();
		if (!dirty)
		{
			//通知ui
			SFE_DataDirtyChange sfe;
			sfe.dirty = true;
			PostFrameEventToUI(sfe.ConvertToFrameEvent());
		}
	}
	break;
	case Imp::ES_Edit_Browser:
	{
		imp_.Manipulator_->SetVisible(true);

		if ( evtRecorder.HasMousePressed(OIS::MB_Left) && imp_.Sweep_ )
		{
			if ( *imp_.Sweep_ == Manipulator::EOT_Arrow )
			{
				imp_.State_ = Imp::ES_Edit_MoveArrow;
				imp_.LastMouseState_ = SysEventRecorder::CachedMouseState();
				imp_.LastDragPnt_ = *imp_.GetRayOnArrowPln(adjRay, imp_.LastMouseState_);
				break;
			}
			else
			{
				imp_.State_ = Imp::ES_Edit_Ring;
				imp_.LastMouseState_ = SysEventRecorder::CachedMouseState();
				break;
			}
		}

		if ( evtRecorder.HasKeyPressed(OIS::KC_F4) )
		{
			auto pos = imp_.HumanNodeInvTrans_ * Ogre::Vector3::ZERO;
			pos.y = 1.224577f;
			Ogre::Plane pln(Ogre::Vector3::UNIT_Y, pos);
			imp_.UpdatePlnData(pln, pos);

			Ogre::AxisAlignedBox box;
			for ( auto& cur : imp_.CurQueryList_ )
			{
				box.merge(cur);
			}

			imp_.Manipulator_->SetPosition(box.getCenter());
		}

		if ( imp_.Sweep_ )
		{
			imp_.Manipulator_->SetState(*imp_.Sweep_, IROM::EPS_Normal);
			imp_.Sweep_.reset();
		}

		imp_.Sweep_ = imp_.Manipulator_->QueryRay(adjRay);
		if ( imp_.Sweep_ )
		{
			imp_.Manipulator_->SetState(*imp_.Sweep_, IROM::EPS_Sweep);
		}

	}
	break;
	case Imp::ES_Edit_MoveArrow:
	{
		if ( evtRecorder.HasMouseReleased(OIS::MB_Left) )
		{
			imp_.State_ = Imp::ES_Edit_Browser;
			break;
		}

		if ( !evtRecorder.HasMouseMoved() )
		{
			break;
		}

		auto curMouseState = SysEventRecorder::CachedMouseState();
		auto offset = curMouseState - imp_.LastMouseState_;

		if ( offset.X.rel == 0 && offset.Y.rel == 0 )
		{
			break;
		}

		auto curDragPnt = imp_.GetRayOnArrowPln(adjRay, curMouseState);
		if ( !curDragPnt )
		{
			break;
		}

		auto delta = *curDragPnt - imp_.LastDragPnt_;

		auto curRot = imp_.Manipulator_->GetOrientation();
		auto pos = imp_.Manipulator_->GetPosition() + delta;
		Ogre::Plane pln(curRot * Ogre::Vector3::UNIT_Y, pos);

		imp_.UpdatePlnData(pln, pos);

		Ogre::AxisAlignedBox box;
		for ( auto& cur : imp_.CurQueryList_ )
		{
			box.merge(cur);
		}

		imp_.Manipulator_->SetPosition(box.getCenter());
		imp_.LastDragPnt_ = *curDragPnt;
		imp_.LastMouseState_ = curMouseState;

		auto dirty = imp_.Rom_->GetDOM()->isDirty();
		imp_.Rom_->GetDOM()->Dirty();
		imp_.Rom_->GetDOM()->UpdateEditTime();
		if (!dirty)
		{
			//通知ui
			SFE_DataDirtyChange sfe;
			sfe.dirty = true;
			PostFrameEventToUI(sfe.ConvertToFrameEvent());
		}
	}
	break;
	case Imp::ES_Edit_Ring:
	{
		if ( evtRecorder.HasMouseReleased(OIS::MB_Left) )
		{
			imp_.State_ = Imp::ES_Edit_Browser;
			break;
		}

		auto curMouseState = SysEventRecorder::CachedMouseState();
		auto offset = curMouseState - imp_.LastMouseState_;

		if ( offset.X.rel == 0 && offset.Y.rel == 0 )
		{
			break;
		}

		auto mpPos = imp_.Manipulator_->GetPosition();

		Ogre::Plane ringPln;
		{
			auto mpRot = imp_.Manipulator_->GetOrientation();

			if ( *imp_.Sweep_ == Manipulator::EOT_XRing )
			{
				auto dir = mpRot * Ogre::Vector3::UNIT_X;
				ringPln.redefine(dir, mpPos);
			}
			else
			{
				auto dir = mpRot * Ogre::Vector3::UNIT_Z;
				ringPln.redefine(dir, mpPos);
			}
		}

		auto lastRay = QueryUtil::GetRayFromCamera(imp_.CameraListener_->GetCamera(), imp_.LastMouseState_, imp_.RT_);
		Ogre::Ray lastAdjRay(imp_.HumanNodeInvTrans_ * lastRay.getOrigin(), imp_.HumanNodeInvRot_ * lastRay.getDirection());

		auto curInt = Ogre::Math::intersects(adjRay, ringPln);
		if ( !curInt.first )
		{
			break;
		}

		auto lastInt = Ogre::Math::intersects(lastAdjRay, ringPln);
		if ( !lastInt.first )
		{
			break;
		}

		auto curPnt = adjRay.getPoint(curInt.second);
		auto lastPnt = lastAdjRay.getPoint(lastInt.second);

		auto curDir = ( curPnt - mpPos ).normalisedCopy();
		auto lastDir = ( lastPnt - mpPos ).normalisedCopy();

		auto angle = curDir.angleBetween(lastDir);

		auto testDir = lastDir.crossProduct(curDir);
		if ( testDir.dotProduct(ringPln.normal) < 0 )
		{
			angle = -angle;
		}

		auto needUpdate = false;

		if ( *imp_.Sweep_ == Manipulator::EOT_XRing )
		{
			auto curDeg = imp_.Manipulator_->GetXDegree();
			auto tmp = curDeg + angle.valueDegrees();
			if ( tmp <= 45.f && tmp >= -45.f )
			{
				imp_.Manipulator_->RotateX(angle.valueRadians());

				needUpdate = true;
			}
		}
		else
		{
			auto curDeg = imp_.Manipulator_->GetZDegree();
			auto tmp = curDeg + angle.valueDegrees();
			if ( tmp <= 45.f && tmp >= -45.f )
			{
				imp_.Manipulator_->RotateZ(angle.valueRadians());

				needUpdate = true;
			}
		}

		imp_.LastMouseState_ = curMouseState;

		if ( !needUpdate )
		{
			break;
		}

		auto curRot = imp_.Manipulator_->GetOrientation();
		auto newPos = imp_.Manipulator_->GetPosition();
		auto curDeg = imp_.Manipulator_->GetRotation();

		imp_.Rom_->SetRotation(std::get<0>(curDeg), std::get<1>(curDeg));

		Ogre::Plane pln(curRot * Ogre::Vector3::UNIT_Y, newPos);
		imp_.UpdatePlnData(pln, newPos);

		auto dirty = imp_.Rom_->GetDOM()->isDirty();
		imp_.Rom_->GetDOM()->Dirty();
		imp_.Rom_->GetDOM()->UpdateEditTime();
		if (!dirty)
		{
			//通知ui
			SFE_DataDirtyChange sfe;
			sfe.dirty = true;
			PostFrameEventToUI(sfe.ConvertToFrameEvent());
		}
	}
	break;
	default:
	break;
	}
}