#include "EditLineLengthController.h"

#include "Render/Camera/MayaCamera.h"
#include "Render/VisibilityFlag.h"
#include "Render/QueryUtil.h"

#include "Render/Extension/Line3D.h"
#include "Render/Extension/Point3D.h"

#include "ROM/LineLengthROM.h"
#include "ROM/IROM.h"
#include "DOM/IDOM.h"

#include "FrameEvent/ToolBarEvent.h"
#include "FrameEvent/ControlEvent.h"

#include "Util/MathUtil.h"

#include "Ogre.h"
#include "OgreMeshSerializer.h"	 

#include "Render/OgreEnv.h"

#include <boost/timer/timer.hpp>


class EditLineLengthController::Imp
{
public:
	enum EState
	{
		ES_Init,
		ES_Create,
		ES_Edit,
		ES_Drag,
		ES_PreExit,
		ES_Count
	};
public:
	Ogre::RenderWindow*			RT_{};
	MayaCameraSPtr 				CameraListener_{};
	Ogre::SceneManager*			Smgr_{};

	Ogre::SceneNode*			HumanNode_{};

	PCLOctreeSPtr				Octree_;

	EState						State_ = ES_Init;

	std::shared_ptr<LineLengthROM>	Rom_;

	Ogre::SceneNode*			Node_ = NULL;
	Point3D*					StartPoint_ = NULL;
	Point3D*					EndPoint_ = NULL;

	Ogre::Vector3	SavedStartPoint_{};
	Ogre::Vector3	SavedEndPoint_{};

public:
	~Imp()
	{}

	bool QueryPicking(Ogre::Vector3& pt)
	{
		boost::timer::cpu_timer ct;

		auto cursorRay = QueryUtil::GetRayFromCamera(CameraListener_->GetCamera(), SysEventRecorder::CachedMouseState(), RT_);

		Ogre::Vector3 cursorAdjOrigin, cursorAdjDir, cursorAdjEndPnt;
		{
			auto transform = HumanNode_->_getFullTransformUpdated();
			auto transformInv = transform.inverse();
			auto rot = transformInv.extractQuaternion();

			cursorAdjOrigin = transformInv * cursorRay.getOrigin();
			cursorAdjDir = rot * cursorRay.getDirection();

			cursorAdjEndPnt = cursorAdjOrigin + cursorAdjDir;
		}

		decltype( cursorRay ) adjCursorRay(cursorAdjOrigin, cursorAdjDir);

		auto queryPnt = Octree_->RayQuery(adjCursorRay, 5e-3);

		if ( std::get<0>(queryPnt) )
		{
			pt = std::get<2>(queryPnt);
			return true;
		}
		else
		{
			return false;
		}
	}

	void createLine(const Ogre::Vector3& start, const Ogre::Vector3& end)
	{
		Rom_->SetVisible(true);
		//Rom_->GetLine()->SetColor(Ogre::ColourValue::Blue);
		//Rom_->GetLine()->SetLineWidth(0.004f);
		Rom_->SetPoint(start, end);
	}

	float distanceRay_Point(const Ogre::Vector3& pt)
	{
		boost::timer::cpu_timer ct;
		auto cursorRay = QueryUtil::GetRayFromCamera(CameraListener_->GetCamera(), SysEventRecorder::CachedMouseState(), RT_);
		Ogre::Vector3 cursorAdjOrigin, cursorAdjDir, cursorAdjEndPnt;
		{
			auto transform = HumanNode_->_getFullTransformUpdated();
			auto transformInv = transform.inverse();
			auto rot = transformInv.extractQuaternion();

			cursorAdjOrigin = transformInv * cursorRay.getOrigin();
			cursorAdjDir = rot * cursorRay.getDirection();

			cursorAdjEndPnt = cursorAdjOrigin + cursorAdjDir;
		}

		decltype( cursorRay ) adjCursorRay(cursorAdjOrigin, cursorAdjDir);

		auto pc = MathUtil::ProjectPointOnLine(pt, adjCursorRay.getOrigin(), adjCursorRay.getDirection());
		return ( pc - pt ).length();
	}
};


EditLineLengthController::EditLineLengthController(Ogre::RenderWindow* rt, const MayaCameraSPtr& camera, Ogre::SceneNode* node, PCLOctreeSPtr octree)
	:ImpUPtr_(new Imp)
{
	auto& imp_ = *ImpUPtr_;

	imp_.RT_ = rt;
	imp_.CameraListener_ = camera;
	imp_.Smgr_ = camera->GetCamera()->getSceneManager();
	imp_.HumanNode_ = node;
	imp_.Octree_ = octree;

	imp_.StartPoint_ = Point3DFactory::CreateInstance(imp_.Smgr_);
	imp_.EndPoint_ = Point3DFactory::CreateInstance(imp_.Smgr_);

	imp_.Node_ = imp_.HumanNode_->createChildSceneNode();
	imp_.Node_->attachObject(imp_.StartPoint_);
	imp_.Node_->attachObject(imp_.EndPoint_);

	imp_.StartPoint_->setVisible(false);
	imp_.EndPoint_->setVisible(false);
}


EditLineLengthController::~EditLineLengthController()
{
	auto& imp_ = *ImpUPtr_;
	imp_.Rom_.reset();

	imp_.StartPoint_->Destory();
	imp_.EndPoint_->Destory();

	imp_.Node_->removeAllChildren();
	imp_.Node_->getParentSceneNode()->removeChild(imp_.Node_);

	Unload();
}


void EditLineLengthController::_FrameQueue(const Ogre::FrameEvent& fevt)
{
	auto& imp_ = *ImpUPtr_;

	auto& evtRecorder = GetSysEventRecorder();

	switch ( imp_.State_ )
	{
	case Imp::ES_Init:
	{
		auto fe = PopFrameEvent<SFE_EditItem>();
		imp_.Rom_ = std::static_pointer_cast<LineLengthROM>( IROM::GetFromDOM(fe->DOM.lock()) );

		switch ( fe->Status )
		{
		case SFE_EditItem::ES_CreateMode:
		{
			imp_.State_ = Imp::ES_Create;
		}
		break;
		case SFE_EditItem::ES_EditMode:
		{
			imp_.State_ = Imp::ES_Edit;

			imp_.SavedStartPoint_ = imp_.Rom_->GetStartPoint();
			imp_.SavedEndPoint_ = imp_.Rom_->GetEndPoint();

			imp_.StartPoint_->setVisible(true);
			imp_.StartPoint_->SetPoint(imp_.Rom_->GetStartPoint());
			imp_.EndPoint_->setVisible(true);
			imp_.EndPoint_->SetPoint(imp_.Rom_->GetEndPoint());
			imp_.createLine(imp_.Rom_->GetStartPoint(), imp_.Rom_->GetEndPoint());

			imp_.Rom_->SetDisplayMode(IROM::EDM_Edit);
		}
		break;
		imp_.State_ = Imp::ES_Count;
		break;
		}
	}
	break;
	case Imp::ES_Create:
	{
		if ( !imp_.StartPoint_->getVisible() )
		{
			if ( evtRecorder.HasMousePressed(OIS::MB_Left) )
			{
				Ogre::Vector3 pt;
				if ( imp_.QueryPicking(pt) )
				{
					imp_.StartPoint_->setVisible(true);
					imp_.StartPoint_->SetPoint(pt);
				}
			}
		}
		else
		{
			if ( evtRecorder.HasMousePressed(OIS::MB_Left) )
			{
				Ogre::Vector3 pt;
				if ( imp_.QueryPicking(pt) )
				{
					//绘制一个点	
					imp_.EndPoint_->setVisible(true);
					imp_.EndPoint_->SetPoint(pt);

					//绘制一根线
					imp_.createLine(imp_.StartPoint_->GetPoint(), imp_.EndPoint_->GetPoint());
					//计算2点直线距离传给UI,并通知UI已经进入编辑状态	
					auto line = imp_.StartPoint_->GetPoint() - imp_.EndPoint_->GetPoint();

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

					imp_.State_ = Imp::ES_Edit;
					imp_.Rom_->SetDisplayMode(IROM::EDM_Edit);
				}
			}
			else if ( evtRecorder.HasMouseMoved() )
			{
				Ogre::Vector3 pt;
				if ( imp_.QueryPicking(pt) )
				{
					//绘制一个点
					imp_.EndPoint_->setVisible(true);
					imp_.EndPoint_->SetPoint(pt);
					//绘制一根线
					imp_.createLine(imp_.StartPoint_->GetPoint(), imp_.EndPoint_->GetPoint());
					//计算2点直线距离传给UI,并通知UI已经进入编辑状态	
					auto line = imp_.StartPoint_->GetPoint() - imp_.EndPoint_->GetPoint();
				}
			}
		}
	}
	break;
	case Imp::ES_Edit:
	{
		if ( evtRecorder.HasMousePressed(OIS::MB_Left) )
		{
			imp_.StartPoint_->SetRadius(0.01);
			imp_.EndPoint_->SetRadius(0.01);
			imp_.StartPoint_->SetColor(Ogre::ColourValue::Red);
			imp_.EndPoint_->SetColor(Ogre::ColourValue::Red);

			auto d = imp_.distanceRay_Point(imp_.StartPoint_->GetPoint());
			if ( d < 0.005 )
			{
				//设置选中第一个点
				imp_.StartPoint_->SetColor(Ogre::ColourValue::Green);
				imp_.StartPoint_->SetRadius(0.02);
				imp_.State_ = Imp::ES_Drag;
			}
			else
			{
				d = imp_.distanceRay_Point(imp_.EndPoint_->GetPoint());
				if ( d < 0.005 )
				{
					//设置选中第二个点
					imp_.EndPoint_->SetColor(Ogre::ColourValue::Green);
					imp_.EndPoint_->SetRadius(0.02);
					imp_.State_ = Imp::ES_Drag;
				}
			}
		}
		else if ( evtRecorder.HasMouseMoved() )
		{
			imp_.StartPoint_->SetRadius(0.01);
			imp_.EndPoint_->SetRadius(0.01);
			imp_.StartPoint_->SetColor(Ogre::ColourValue::Red);
			imp_.EndPoint_->SetColor(Ogre::ColourValue::Red);
			auto d = imp_.distanceRay_Point(imp_.StartPoint_->GetPoint());
			if ( d < 0.005 )
			{
				//设置选中第一个点
				imp_.StartPoint_->SetColor(Ogre::ColourValue::Green);
				imp_.StartPoint_->SetRadius(0.02);
			}
			else
			{
				d = imp_.distanceRay_Point(imp_.EndPoint_->GetPoint());
				if ( d < 0.005 )
				{
					//设置选中第二个点
					imp_.EndPoint_->SetColor(Ogre::ColourValue::Green);
					imp_.EndPoint_->SetRadius(0.02);
				}
			}
		}
	}
	break;
	case Imp::ES_Drag:
	{
		if ( evtRecorder.HasMouseReleased(OIS::MB_Left) )
		{
			imp_.StartPoint_->SetColor(Ogre::ColourValue::Red);
			imp_.StartPoint_->SetRadius(0.01);
			imp_.EndPoint_->SetColor(Ogre::ColourValue::Red);
			imp_.EndPoint_->SetRadius(0.01);
			imp_.State_ = Imp::ES_Edit;
		}
		else if ( evtRecorder.HasMouseMoved() )
		{
			//第一个点
			if ( imp_.StartPoint_->GetColor() == Ogre::ColourValue::Green )
			{
				//设置当前点新位置
				Ogre::Vector3 pt;
				if ( imp_.QueryPicking(pt) )
				{
					imp_.StartPoint_->setVisible(true);
					imp_.StartPoint_->SetPoint(pt);
				}
			}
			//第二个点
			if ( imp_.EndPoint_->GetColor() == Ogre::ColourValue::Green )
			{
				//设置当前点新位置
				Ogre::Vector3 pt;
				if ( imp_.QueryPicking(pt) )
				{
					imp_.EndPoint_->setVisible(true);
					imp_.EndPoint_->SetPoint(pt);
				}
			}
			//绘制一根线
			imp_.createLine(imp_.StartPoint_->GetPoint(), imp_.EndPoint_->GetPoint());

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
	}
	break;
	default:
	break;
	}
}
