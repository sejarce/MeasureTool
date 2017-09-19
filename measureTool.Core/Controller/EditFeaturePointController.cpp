#include "EditFeaturePointController.h"

#include "Render/Camera/MayaCamera.h"
#include "Render/QueryUtil.h"

#include "Render/Extension/Point3D.h"

#include "FrameEvent/FloatBarEvent.h"
#include "FrameEvent/ControlEvent.h"

#include "Util/MathUtil.h"

#include "DOM/Document.h"

#include "Render/PrefabResourceMgr.h"

#include "Ogre.h"

#include "Render/OgreEnv.h"


class EditFeaturePointController::Imp
{
public:
	enum EState
	{
		ES_Init,		
		ES_Edit,
		ES_Select,
		Es_DragX,
		Es_DragY,
		Es_DragZ,
		ES_PreExit,
		ES_Count
	};
public:
	Ogre::RenderWindow*			RT_{};
	MayaCameraSPtr 				CameraListener_{};
	Ogre::SceneManager*			Smgr_{};

	Ogre::SceneNode*			HumanNode_{};

	Ogre::SceneNode*			Node_{};

	EState						State_ = ES_Init;

	Point3D*					CrothPoint_{};
	Point3D*					LeftPoint_{};
	Point3D*					RightPoint_{};

	std::vector<Point3D*>		FeaturePoints_{};

	Ogre::SceneNode*			AuxNode_{};
	Ogre::SceneNode*			AuxXNode_{};
	Ogre::SceneNode*			AuxYNode_{};
	Ogre::SceneNode*			AuxZNode_{};
	Ogre::Entity*				AuxXEntity_{};
	Ogre::MeshPtr				AuxXMesh_;
	Ogre::Entity*				AuxYEntity_{};
	Ogre::MeshPtr				AuxYMesh_;
	Ogre::Entity*				AuxZEntity_{};
	Ogre::MeshPtr				AuxZMesh_;

	DocumentSPtr				Doc_{};
public:

	Ogre::Ray getCurrCameraRay() const
	{
		auto cursorRay = QueryUtil::GetRayFromCamera(CameraListener_->GetCamera(), SysEventRecorder::CachedMouseState(), RT_);

		Ogre::Vector3 cursorAdjOrigin, cursorAdjDir, cursorAdjEndPnt;
		auto transform = HumanNode_->_getFullTransformUpdated();
		auto transformInv = transform.inverse();
		auto rot = transformInv.extractQuaternion();

		cursorAdjOrigin = transformInv * cursorRay.getOrigin();
		cursorAdjDir = rot * cursorRay.getDirection();

		cursorAdjEndPnt = cursorAdjOrigin + cursorAdjDir;

		decltype(cursorRay) adjCursorRay(cursorAdjOrigin, cursorAdjDir);

		return adjCursorRay;
	}

	float distanceRay_Point(const Ogre::Vector3& pt)
	{
		auto adjCursorRay = getCurrCameraRay();

		auto pc = MathUtil::ProjectPointOnLine(pt, adjCursorRay.getOrigin(), adjCursorRay.getDirection());
		return (pc - pt).length();
	}

	float distanceRay_Ray(const Ogre::Ray& ray)
	{
		auto adjCursorRay = getCurrCameraRay();

		//distance of line_line
		auto nor = adjCursorRay.getDirection().crossProduct(ray.getDirection());
		nor.normalise();
		auto l = adjCursorRay.getOrigin() - ray.getOrigin();
		return abs(l.dotProduct(nor));
	}

	bool queryRay_Plane(Ogre::Vector3& pt, /*const Ogre::Vector3& planeNor,*/ 
		const Ogre::Vector3& planeline,
		const Ogre::Vector3& planePos)
	{
		auto adjCursorRay = getCurrCameraRay();

		auto v = planeline.crossProduct(adjCursorRay.getDirection());
		auto planeNor = v.crossProduct(planeline);
		
		auto t = planeNor.dotProduct(planePos) - planeNor.dotProduct(adjCursorRay.getOrigin());

		auto p = planeNor.dotProduct(adjCursorRay.getDirection());
		if (p != 0)
		{
			t /= p;
			if (t >= 0)
			{
				pt = adjCursorRay.getOrigin() + t*adjCursorRay.getDirection();
				return true;
			}
		}
		return false;
	}
};

EditFeaturePointController::EditFeaturePointController(Ogre::RenderWindow* rt
	, const MayaCameraSPtr& camera
	, Ogre::SceneNode* node
	, DocumentSPtr& doc) : ImpUPtr_(new Imp)
{
	auto& imp_ = *ImpUPtr_;

	imp_.RT_ = rt;
	imp_.CameraListener_ = camera;
	imp_.Smgr_ = camera->GetCamera()->getSceneManager();
	imp_.HumanNode_ = node;

	imp_.CrothPoint_ = Point3DFactory::CreateInstance(imp_.Smgr_);
	imp_.LeftPoint_ = Point3DFactory::CreateInstance(imp_.Smgr_);
	imp_.RightPoint_ = Point3DFactory::CreateInstance(imp_.Smgr_);

	imp_.Node_ = imp_.HumanNode_->createChildSceneNode();
	imp_.Node_->attachObject(imp_.CrothPoint_);
	imp_.Node_->attachObject(imp_.LeftPoint_);
	imp_.Node_->attachObject(imp_.RightPoint_);

	imp_.CrothPoint_->setVisible(false);
	imp_.LeftPoint_->setVisible(false);
	imp_.RightPoint_->setVisible(false);

	for (auto cur : doc->getFeaturePoints())
	{
		auto point3d = Point3DFactory::CreateInstance(imp_.Smgr_);
		point3d->SetPoint(cur.second);
		imp_.Node_->attachObject(point3d);
		point3d->setVisible(false);
		imp_.FeaturePoints_.emplace_back(point3d);
	}

	imp_.AuxNode_ = imp_.HumanNode_->createChildSceneNode();

	imp_.AuxXNode_ = imp_.AuxNode_->createChildSceneNode();
	imp_.AuxXMesh_ = PrefabResourceMgr::GetInstance().GetMesh(PrefabResourceMgr::EPM_AuxY);
	imp_.AuxXMesh_->load();
	imp_.AuxXEntity_ = imp_.Smgr_->createEntity(imp_.AuxXMesh_);
	imp_.AuxXNode_->attachObject(imp_.AuxXEntity_);

	imp_.AuxYNode_ = imp_.AuxNode_->createChildSceneNode();
	imp_.AuxYMesh_ = PrefabResourceMgr::GetInstance().GetMesh(PrefabResourceMgr::EPM_AuxZ);
	imp_.AuxYMesh_->load();
	imp_.AuxYEntity_ = imp_.Smgr_->createEntity(imp_.AuxYMesh_);
	imp_.AuxYNode_->attachObject(imp_.AuxYEntity_);

	imp_.AuxZNode_ = imp_.AuxNode_->createChildSceneNode();
	imp_.AuxZMesh_ = PrefabResourceMgr::GetInstance().GetMesh(PrefabResourceMgr::EPM_AuxX);
	imp_.AuxZMesh_->load();
	imp_.AuxZEntity_ = imp_.Smgr_->createEntity(imp_.AuxZMesh_);
	imp_.AuxZNode_->attachObject(imp_.AuxZEntity_);

	imp_.AuxNode_->setVisible(false);

	imp_.Doc_ = doc;
}

EditFeaturePointController::~EditFeaturePointController()
{
	auto& imp_ = *ImpUPtr_;

	imp_.CrothPoint_->Destory();
	imp_.LeftPoint_->Destory();
	imp_.RightPoint_->Destory();

	for (int i = 0; i < imp_.FeaturePoints_.size(); i++)
	{
		imp_.FeaturePoints_[i]->Destory();
	}
	imp_.FeaturePoints_.clear();
	std::vector<Point3D*>().swap(imp_.FeaturePoints_);

	imp_.Node_->removeAllChildren();
	imp_.Node_->getParentSceneNode()->removeChild(imp_.Node_);

	imp_.AuxXMesh_->unload();
	imp_.AuxXEntity_->detachFromParent();
	imp_.Smgr_->destroyMovableObject(imp_.AuxXEntity_);
	imp_.AuxYMesh_->unload();
	imp_.AuxYEntity_->detachFromParent();
	imp_.Smgr_->destroyMovableObject(imp_.AuxYEntity_);
	imp_.AuxZMesh_->unload();
	imp_.AuxZEntity_->detachFromParent();
	imp_.Smgr_->destroyMovableObject(imp_.AuxZEntity_);

	imp_.AuxNode_->removeAllChildren();
	imp_.AuxNode_->getParentSceneNode()->removeChild(imp_.AuxNode_);

	Unload();

	SFE_EditFeaturePoints sfe;
	sfe.conState = SFE_EditFeaturePoints::E_Hide;
	PostFrameEventToUI(sfe.ConvertToFrameEvent());
}

void EditFeaturePointController::_FrameQueue(const Ogre::FrameEvent& fevt)
{
	auto& imp_ = *ImpUPtr_;
	
	auto doc = imp_.Doc_;
	auto& evtRecorder = GetSysEventRecorder();

	static Ogre::Vector3 selectPos{};
	static Ogre::Vector3 queryPos{};

	auto fe = PopFrameEvent<SFE_EditFeaturePoints>();
	if (fe && fe->conState == SFE_EditFeaturePoints::E_Hide)
	{
		SFE_Finish fe;
		GetParent()->HandleFrameEventImmediately(fe.ConvertToFrameEvent());
		RemoveLazy();
	}

	switch (imp_.State_)
	{
	case Imp::ES_Init:
	{
		imp_.CrothPoint_->SetPoint(imp_.Doc_->getCrothPoint());
		imp_.LeftPoint_->SetPoint(imp_.Doc_->getLeftPoint());
		imp_.RightPoint_->SetPoint(imp_.Doc_->getRightPoint());
		imp_.CrothPoint_->setVisible(true);
		imp_.LeftPoint_->setVisible(true);
		imp_.RightPoint_->setVisible(true);

		for (auto cur : imp_.FeaturePoints_)
		{
			cur->setVisible(true);
		}

		imp_.State_ = Imp::ES_Edit;
	}
	break;	
	case Imp::ES_Edit:
	{
		if (evtRecorder.HasMousePressed(OIS::MB_Left))
		{
			imp_.CrothPoint_->SetColor(Ogre::ColourValue::Red);
			imp_.CrothPoint_->SetRadius(0.01);
			imp_.LeftPoint_->SetColor(Ogre::ColourValue::Red);
			imp_.LeftPoint_->SetRadius(0.01);
			imp_.RightPoint_->SetColor(Ogre::ColourValue::Red);
			imp_.RightPoint_->SetRadius(0.01);
			auto d = imp_.distanceRay_Point(imp_.CrothPoint_->GetPoint());

			if (d < 0.005)
			{
				imp_.CrothPoint_->SetColor(Ogre::ColourValue::Green);
				imp_.CrothPoint_->SetRadius(0.02);
				selectPos = imp_.CrothPoint_->GetPoint();
				imp_.State_ = Imp::ES_Select;
			}
			else
			{
				d = imp_.distanceRay_Point(imp_.LeftPoint_->GetPoint());

				if (d < 0.005)
				{
					imp_.LeftPoint_->SetColor(Ogre::ColourValue::Green);
					imp_.LeftPoint_->SetRadius(0.02);
					selectPos = imp_.LeftPoint_->GetPoint();
					imp_.State_ = Imp::ES_Select;
				}
				else
				{
					d = imp_.distanceRay_Point(imp_.RightPoint_->GetPoint());

					if (d < 0.005)
					{
						imp_.RightPoint_->SetColor(Ogre::ColourValue::Green);
						imp_.RightPoint_->SetRadius(0.02);
						selectPos = imp_.RightPoint_->GetPoint();
						imp_.State_ = Imp::ES_Select;
					}
				}
			}
		}
		else if (evtRecorder.HasMouseMoved())
		{
			imp_.CrothPoint_->SetColor(Ogre::ColourValue::Red);
			imp_.CrothPoint_->SetRadius(0.01);
			imp_.LeftPoint_->SetColor(Ogre::ColourValue::Red);
			imp_.LeftPoint_->SetRadius(0.01);
			imp_.RightPoint_->SetColor(Ogre::ColourValue::Red);
			imp_.RightPoint_->SetRadius(0.01);
			auto d = imp_.distanceRay_Point(imp_.CrothPoint_->GetPoint());
			if (d < 0.005)
			{
				imp_.CrothPoint_->SetColor(Ogre::ColourValue::Green);
				imp_.CrothPoint_->SetRadius(0.02);
			}
			else
			{
				d = imp_.distanceRay_Point(imp_.LeftPoint_->GetPoint());
				if (d < 0.005)
				{
					imp_.LeftPoint_->SetColor(Ogre::ColourValue::Green);
					imp_.LeftPoint_->SetRadius(0.02);
				}
				else
				{
					d = imp_.distanceRay_Point(imp_.RightPoint_->GetPoint());
					if (d < 0.005)
					{
						imp_.RightPoint_->SetColor(Ogre::ColourValue::Green);
						imp_.RightPoint_->SetRadius(0.02);
					}
				}
			}
		}
	}
	break;
	case Imp::ES_Select:
	{
		//show aux
		imp_.AuxNode_->setPosition(selectPos);
		imp_.AuxNode_->setVisible(true);

		if (evtRecorder.HasMouseReleased(OIS::MB_Right))
		{
			imp_.AuxXNode_->setScale(1.0, 1.0, 1.0);
			imp_.AuxYNode_->setScale(1.0, 1.0, 1.0);
			imp_.AuxZNode_->setScale(1.0, 1.0, 1.0);
			imp_.AuxNode_->setVisible(false);
			imp_.State_ = Imp::ES_Edit;
			break;
		}

		//query
		//x÷·
		Ogre::Ray xray{ selectPos, Ogre::Vector3(1, 0, 0) };
		Ogre::Ray yray{ selectPos, Ogre::Vector3(0, 1, 0) };
		Ogre::Ray zray{ selectPos, Ogre::Vector3(0, 0, 1) };
		if (evtRecorder.HasMousePressed(OIS::MB_Left))
		{
			auto d = imp_.distanceRay_Ray(xray);
			if (d < 0.005)
			{
				imp_.AuxYNode_->setScale(1.0, 1.0, 1.0);
				imp_.AuxZNode_->setScale(1.0, 1.0, 1.0);
				imp_.AuxXNode_->setScale(1.0, 3.0, 3.0);

				if (imp_.queryRay_Plane(queryPos, xray.getDirection(), selectPos))
					imp_.State_ = Imp::Es_DragX;
			}
			else
			{
				d = imp_.distanceRay_Ray(yray);
				if (d < 0.005)
				{					
					imp_.AuxXNode_->setScale(1.0, 1.0, 1.0);
					imp_.AuxZNode_->setScale(1.0, 1.0, 1.0);
					imp_.AuxYNode_->setScale(3.0, 1.0, 3.0);

					if (imp_.queryRay_Plane(queryPos, yray.getDirection(), selectPos))
						imp_.State_ = Imp::Es_DragY;
				}
				else
				{
					d = imp_.distanceRay_Ray(zray);
					if (d < 0.005)
					{
						imp_.AuxXNode_->setScale(1.0, 1.0, 1.0);
						imp_.AuxYNode_->setScale(1.0, 1.0, 1.0);
						imp_.AuxZNode_->setScale(3.0, 3.0, 1.0);

						if (imp_.queryRay_Plane(queryPos, zray.getDirection(), selectPos))
							imp_.State_ = Imp::Es_DragZ;
					}
				}
			}
		}
		else if (evtRecorder.HasMouseMoved())
		{
			auto d = imp_.distanceRay_Ray(xray);
			if (d < 0.005)
			{
				imp_.AuxYNode_->setScale(1.0, 1.0, 1.0);
				imp_.AuxZNode_->setScale(1.0, 1.0, 1.0);
				imp_.AuxXNode_->setScale(1.0, 3.0, 3.0);
			}
			else
			{
				d = imp_.distanceRay_Ray(yray);
				if (d < 0.005)
				{
					imp_.AuxXNode_->setScale(1.0, 1.0, 1.0);
					imp_.AuxZNode_->setScale(1.0, 1.0, 1.0);
					imp_.AuxYNode_->setScale(3.0, 1.0, 3.0);
				}
				else
				{
					d = imp_.distanceRay_Ray(zray);
					if (d < 0.005)
					{
						imp_.AuxXNode_->setScale(1.0, 1.0, 1.0);
						imp_.AuxYNode_->setScale(1.0, 1.0, 1.0);
						imp_.AuxZNode_->setScale(3.0, 3.0, 1.0);
					}
					else
					{
						imp_.AuxXNode_->setScale(1.0, 1.0, 1.0);
						imp_.AuxYNode_->setScale(1.0, 1.0, 1.0);
						imp_.AuxZNode_->setScale(1.0, 1.0, 1.0);
					}
				}
			}
		}
	}
	break;
	case Imp::Es_DragX:
	{
		if (evtRecorder.HasMouseReleased(OIS::MB_Left))
			imp_.State_ = Imp::ES_Select;
		Ogre::Ray xray{ selectPos, Ogre::Vector3(1, 0, 0) };
		if (evtRecorder.HasMouseMoved())
		{
			Ogre::Vector3 pos;
			if (imp_.queryRay_Plane(pos, xray.getDirection(), selectPos))
			{
				auto v = pos - queryPos;
				auto x = v.dotProduct(xray.getDirection());
				queryPos = pos;
				selectPos.x += x;
				imp_.AuxNode_->setPosition(selectPos);
				if (imp_.CrothPoint_->GetColor() == Ogre::ColourValue::Green)
				{
					imp_.CrothPoint_->SetPoint(selectPos);
					imp_.Doc_->setCrothPoint(selectPos);
				}
				else if (imp_.LeftPoint_->GetColor() == Ogre::ColourValue::Green)
				{
					imp_.LeftPoint_->SetPoint(selectPos);
					imp_.Doc_->setLeftPoint(selectPos);
				}
				else if (imp_.RightPoint_->GetColor() == Ogre::ColourValue::Green)
				{
					imp_.RightPoint_->SetPoint(selectPos);
					imp_.Doc_->setRightPoint(selectPos);
				}
			}
		}
	}
	break;
	case Imp::Es_DragY:
	{
		if (evtRecorder.HasMouseReleased(OIS::MB_Left))
			imp_.State_ = Imp::ES_Select;
		Ogre::Ray yray{ selectPos, Ogre::Vector3(0, 1, 0) };
		if (evtRecorder.HasMouseMoved())
		{
			Ogre::Vector3 pos;
			if (imp_.queryRay_Plane(pos, yray.getDirection(), selectPos))
			{
				auto v = pos - queryPos;
				auto y = v.dotProduct(yray.getDirection());
				queryPos = pos;
				selectPos.y += y;
				imp_.AuxNode_->setPosition(selectPos);
				if (imp_.CrothPoint_->GetColor() == Ogre::ColourValue::Green)
				{
					imp_.CrothPoint_->SetPoint(selectPos);
					imp_.Doc_->setCrothPoint(selectPos);
				}
				else if (imp_.LeftPoint_->GetColor() == Ogre::ColourValue::Green)
				{
					imp_.LeftPoint_->SetPoint(selectPos);
					imp_.Doc_->setLeftPoint(selectPos);
				}
				else if (imp_.RightPoint_->GetColor() == Ogre::ColourValue::Green)
				{
					imp_.RightPoint_->SetPoint(selectPos);
					imp_.Doc_->setRightPoint(selectPos);
				}
			}
		}
	}
	break;
	case Imp::Es_DragZ:
	{
		if (evtRecorder.HasMouseReleased(OIS::MB_Left))
		{
			imp_.AuxXNode_->setScale(1.0, 1.0, 1.0);
			imp_.AuxYNode_->setScale(1.0, 1.0, 1.0);
			imp_.AuxZNode_->setScale(1.0, 1.0, 1.0);
			imp_.State_ = Imp::ES_Select;
			break;
		}
		Ogre::Ray zray{ selectPos, Ogre::Vector3(0, 0, 1) };
		if (evtRecorder.HasMouseMoved())
		{
			Ogre::Vector3 pos;
			if (imp_.queryRay_Plane(pos, zray.getDirection(), selectPos))
			{
				auto v = pos - queryPos;
				auto z = v.dotProduct(zray.getDirection());
				queryPos = pos;
				selectPos.z += z;
				imp_.AuxNode_->setPosition(selectPos);
				if (imp_.CrothPoint_->GetColor() == Ogre::ColourValue::Green)
				{
					imp_.CrothPoint_->SetPoint(selectPos);
					imp_.Doc_->setCrothPoint(selectPos);
				}
				else if (imp_.LeftPoint_->GetColor() == Ogre::ColourValue::Green)
				{
					imp_.LeftPoint_->SetPoint(selectPos);
					imp_.Doc_->setLeftPoint(selectPos);
				}
				else if (imp_.RightPoint_->GetColor() == Ogre::ColourValue::Green)
				{
					imp_.RightPoint_->SetPoint(selectPos);
					imp_.Doc_->setRightPoint(selectPos);
				}
			}
		}
	}
	break;
	default:
		break;
	}
}

