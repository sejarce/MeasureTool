#include "EditCurveLengthController.h"

#include "Render/Camera/MayaCamera.h"
#include "Render/VisibilityFlag.h"
#include "Render/QueryUtil.h"

#include "Render/Extension/Point3D.h"
#include "Render/Extension/DynamicPlane.h"
#include "Render/Extension/DynamicPointSet.h"

#include "ROM/CurveLengthROM.h"
#include "ROM/IROM.h"
#include "DOM/IDOM.h"

#include "FrameEvent/ToolBarEvent.h"
#include "FrameEvent/ControlEvent.h"

#include "Util/MathUtil.h"

#include "Ogre.h"
#include "OgreMeshSerializer.h"	 

#include "Render/OgreEnv.h"

class EditCurveLengthController::Imp
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

	Ogre::RenderWindow*			RT_{};
	MayaCameraSPtr 				CameraListener_{};
	Ogre::SceneManager*			Smgr_{};

	Ogre::SceneNode*			HumanNode_{};
	Ogre::Matrix4				HumanNodeInvTrans_;
	Ogre::Quaternion			HumanNodeInvRot_;

	PCLOctreeSPtr				Octree_;
	PCLKdtreeSPtr				Kdtree_;

	EState						State_ = ES_Init;

	std::shared_ptr<CurveLengthROM>	Rom_;

	Ogre::SceneNode*			Node_ = NULL;

	Ogre::SceneNode*		DbgQueryPointNode_{};
	DynamicPointSet*		DbgQueryPointSet_{};

	Ogre::SceneNode*		DbgSamplerPointNode_{};
	DynamicPointSet*		DbgFittingPointSet_{};

	std::pair<int, Point3D*>	StartPoint_;
	std::pair<int, Point3D*>	EndPoint_;
	
	//std::vector<Point3D*>		PointLists_;

	std::pair<int, Ogre::Vector3>	SavedStartPoint_{};
	std::pair<int, Ogre::Vector3>	SavedEndPoint_{};
	std::vector<Ogre::Vector3> SaveplnList_;

public:

	bool QueryPicking(int& vertexIndex, Ogre::Vector3& pt)
	{
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

		if (std::get<0>(queryPnt))
		{
			vertexIndex = std::get<1>(queryPnt);
			pt = std::get<2>(queryPnt);
			return true;
		}
		else
		{
			return false;
		}
	}

	float distanceRay_Point(const Ogre::Vector3& pt)
	{
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

	void computerShortestPath(const std::pair<int, Ogre::Vector3>& s, const std::pair<int, Ogre::Vector3>& e)
	{
		auto si = s.first;
		auto ei = e.first;
		if (si < 0)
		{
			si = Kdtree_->findPointIndex(s.second);
		}
		if (ei < 0)
		{
			ei = Kdtree_->findPointIndex(e.second);
		}

		PCLKdtree::PointPath pp = Kdtree_->computerShortestPath(si, ei);

		//for (auto curr : PointLists_)
		//{
		//	curr->Destory();
		//}
		//PointLists_.clear();
		//std::vector<Point3D*>().swap(PointLists_);

		if (pp.size() > 1)
		{
			std::vector<Ogre::Vector3> plnList;
			std::pair<int, Ogre::Vector3> ss = pp.back();
			std::pair<int, Ogre::Vector3> ee = pp.front();
			//int index = 0;
			while (!pp.empty())
			{
				//std::cout << "index:" << index++ << " pos:" << pp.back().second.x << "," << pp.back().second.y << "," << pp.back().second.z << std::endl;
				//index++;
				//auto point3d = Point3DFactory::CreateInstance(Smgr_);
				//Node_->attachObject(point3d);
				//point3d->setVisible(false);
				//point3d->SetPoint(pp.back().second);
				//point3d->setVisible(true);
				//point3d->SetColor(Ogre::ColourValue::Blue);
				//point3d->SetRadius(0.002);
				//PointLists_.emplace_back(point3d);
				plnList.emplace_back(pp.back().second);
				pp.pop_back();
			}
			Rom_->SetVisible(true);
			Rom_->Update(ss, ee, plnList);
		}
	}
};


EditCurveLengthController::EditCurveLengthController(Ogre::RenderWindow* rt,
													   const MayaCameraSPtr& camera,
													   Ogre::SceneNode* node,
													   PCLOctreeSPtr octree,
													   PCLKdtreeSPtr kdtree)
													   :ImpUPtr_(new Imp)
{
	auto& imp_ = *ImpUPtr_;

	imp_.RT_ = rt;
	imp_.CameraListener_ = camera;
	imp_.Smgr_ = camera->GetCamera()->getSceneManager();
	imp_.HumanNode_ = node;
	imp_.Octree_ = octree;
	imp_.Kdtree_ = kdtree;

	imp_.HumanNodeInvTrans_ = imp_.HumanNode_->_getFullTransformUpdated().inverse();
	imp_.HumanNodeInvRot_ = imp_.HumanNodeInvTrans_.extractQuaternion();

	imp_.StartPoint_.second = Point3DFactory::CreateInstance(imp_.Smgr_);
	imp_.EndPoint_.second = Point3DFactory::CreateInstance(imp_.Smgr_);

	imp_.Node_ = imp_.HumanNode_->createChildSceneNode();
	imp_.Node_->attachObject(imp_.StartPoint_.second);
	imp_.Node_->attachObject(imp_.EndPoint_.second);

	imp_.StartPoint_.second->setVisible(false);
	imp_.EndPoint_.second->setVisible(false);

	imp_.DbgQueryPointNode_ = imp_.Node_->createChildSceneNode();
	{
		imp_.DbgQueryPointSet_ = DynamicPointSetFactory::CreateInstance(imp_.Smgr_);
		imp_.DbgQueryPointSet_->SetColor(Ogre::ColourValue::Red);

		imp_.DbgQueryPointNode_->attachObject(imp_.DbgQueryPointSet_);

		imp_.DbgQueryPointNode_->setVisible(false);
	}

	imp_.DbgSamplerPointNode_ = imp_.Node_->createChildSceneNode();
	{
		imp_.DbgFittingPointSet_ = DynamicPointSetFactory::CreateInstance(imp_.Smgr_);
		imp_.DbgFittingPointSet_->SetColor(Ogre::ColourValue::Green);

		imp_.DbgSamplerPointNode_->attachObject(imp_.DbgFittingPointSet_);

		imp_.DbgSamplerPointNode_->setVisible(false);
	}
}

EditCurveLengthController::~EditCurveLengthController()
{
	auto& imp_ = *ImpUPtr_;
	imp_.Rom_.reset();

	imp_.StartPoint_.second->Destory();
	imp_.EndPoint_.second->Destory();

	imp_.DbgQueryPointSet_->Destory();
	imp_.DbgFittingPointSet_->Destory();

	imp_.Node_->removeAllChildren();
	imp_.Node_->getParentSceneNode()->removeChild(imp_.Node_);

	Unload();
	//imp_.Rom_->SetPickingState(IROM::EPS_Normal);	
}

void EditCurveLengthController::_FrameQueue(const Ogre::FrameEvent& fevt)
{
	auto& imp_ = *ImpUPtr_;

	auto& evtRecorder = GetSysEventRecorder();

	auto sfe = PopFrameEvent<SFE_SDC>();
	if ( sfe )
	{
		switch ( sfe->To3DState )
		{
		case SFE_SDC::ES_Save:
		{
			imp_.SavedStartPoint_ = imp_.Rom_->GetStartPos();
			imp_.SavedEndPoint_ = imp_.Rom_->GetEndPos();
			imp_.SaveplnList_ = imp_.Rom_->GetFittingList();

			imp_.Rom_->SetDisplayMode(IROM::EDM_Browser);

			SFE_Finish fe;
			GetParent()->HandleFrameEventImmediately(fe.ConvertToFrameEvent());
			RemoveLazy();
		}
		break;
		case SFE_SDC::ES_Delete:
		{
			imp_.Rom_->SetDisplayMode(IROM::EDM_Browser);

			SFE_Finish fe;
			GetParent()->HandleFrameEventImmediately(fe.ConvertToFrameEvent());
			RemoveLazy();
		}
		break;
		case SFE_SDC::ES_Cancel:
		{
			imp_.Rom_->Update(imp_.SavedStartPoint_, imp_.SavedEndPoint_, imp_.SaveplnList_);

			imp_.StartPoint_.second->SetPoint(imp_.Rom_->GetStartPos().second);
			imp_.StartPoint_.first = imp_.Rom_->GetStartPos().first;
			imp_.EndPoint_.second->SetPoint(imp_.Rom_->GetEndPos().second);
			imp_.EndPoint_.first = imp_.Rom_->GetEndPos().first;

			//求2个点的最短路径
			imp_.computerShortestPath(imp_.Rom_->GetStartPos(), imp_.Rom_->GetEndPos());

			imp_.Rom_->SetDisplayMode(IROM::EDM_Browser);
			SFE_Finish fe;
			GetParent()->HandleFrameEventImmediately(fe.ConvertToFrameEvent());
			RemoveLazy();
		}
		default:
		break;
		}
	}

	switch ( imp_.State_ )
	{
	case Imp::ES_Init:
	{
		auto fe = PopFrameEvent<SFE_EditItem>();
		imp_.Rom_ = std::static_pointer_cast<CurveLengthROM>( IROM::GetFromDOM(fe->DOM.lock()) );
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

			imp_.SavedStartPoint_ = imp_.Rom_->GetStartPos();
			imp_.SavedEndPoint_ = imp_.Rom_->GetEndPos();
			imp_.SaveplnList_ = imp_.Rom_->GetFittingList();

			imp_.StartPoint_.second->SetPoint(imp_.Rom_->GetStartPos().second);
			imp_.StartPoint_.first = imp_.Rom_->GetStartPos().first;
			imp_.EndPoint_.second->SetPoint(imp_.Rom_->GetEndPos().second);
			imp_.EndPoint_.first = imp_.Rom_->GetEndPos().first;
			imp_.StartPoint_.second->setVisible(true);
			imp_.EndPoint_.second->setVisible(true);

			//求2个点的最短路径
			imp_.computerShortestPath(imp_.Rom_->GetStartPos(), imp_.Rom_->GetEndPos());

			imp_.Rom_->SetDisplayMode(IROM::EDM_Edit);
		}
		break;
		default:
		imp_.State_ = Imp::ES_Count;
		break;
		}
	}
	break;
	case Imp::ES_Create:
	{
		if ( !imp_.StartPoint_.second->getVisible() )
		{
			if ( evtRecorder.HasMousePressed(OIS::MB_Left) )
			{
				int verIndex;
				Ogre::Vector3 pt;
				if (imp_.QueryPicking(verIndex, pt))
				{
					imp_.StartPoint_.second->setVisible(true);
					imp_.StartPoint_.second->SetPoint(pt);
					imp_.StartPoint_.first = verIndex;
				}
			}
		}
		else
		{
			if ( evtRecorder.HasMousePressed(OIS::MB_Left) )
			{
				int verIndex;
				Ogre::Vector3 pt;
				if (imp_.QueryPicking(verIndex, pt))
				{
					imp_.EndPoint_.second->setVisible(true);
					imp_.EndPoint_.second->SetPoint(pt);
					imp_.EndPoint_.first = verIndex;
					//求2个点的最短路径	
					std::pair<int, Ogre::Vector3> sp = { imp_.StartPoint_.first, imp_.StartPoint_.second->GetPoint() };
					std::pair<int, Ogre::Vector3> ep = { imp_.EndPoint_.first, imp_.EndPoint_.second->GetPoint() };
					imp_.computerShortestPath(sp,ep);

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
				int verIndex;
				Ogre::Vector3 pt;
				if ( imp_.QueryPicking(verIndex, pt) )
				{
					//求2个点的最短路径
					std::pair<int, Ogre::Vector3> sp = { imp_.StartPoint_.first, imp_.StartPoint_.second->GetPoint() };
					std::pair<int, Ogre::Vector3> ep = { verIndex, pt };
					imp_.computerShortestPath(sp, ep);
					imp_.Rom_->SetDisplayMode(IROM::EDM_Edit);
				}
			}
		}
	}
	break;
	case Imp::ES_Edit:
	{
		if ( evtRecorder.HasMousePressed(OIS::MB_Left) )
		{
			imp_.StartPoint_.second->SetRadius(0.01);
			imp_.EndPoint_.second->SetRadius(0.01);
			imp_.StartPoint_.second->SetColor(Ogre::ColourValue::Red);
			imp_.EndPoint_.second->SetColor(Ogre::ColourValue::Red);

			auto d = imp_.distanceRay_Point(imp_.StartPoint_.second->GetPoint());
			if ( d < 0.005 )
			{
				//设置选中startpos
				imp_.StartPoint_.second->SetColor(Ogre::ColourValue::Green);
				imp_.StartPoint_.second->SetRadius(0.02);
				imp_.State_ = Imp::ES_Drag;
			}
			else
			{
				d = imp_.distanceRay_Point(imp_.EndPoint_.second->GetPoint());
				if (d < 0.005)
				{
					//设置选中endpos
					imp_.EndPoint_.second->SetColor(Ogre::ColourValue::Green);
					imp_.EndPoint_.second->SetRadius(0.02);
					imp_.State_ = Imp::ES_Drag;
				}				
			}
		}
		else if ( evtRecorder.HasMouseMoved() )
		{
			imp_.StartPoint_.second->SetRadius(0.01);
			imp_.EndPoint_.second->SetRadius(0.01);

			imp_.StartPoint_.second->SetColor(Ogre::ColourValue::Red);
			imp_.EndPoint_.second->SetColor(Ogre::ColourValue::Red);

			auto d = imp_.distanceRay_Point(imp_.StartPoint_.second->GetPoint());
			if ( d < 0.005 )
			{
				//设置选中startpos
				imp_.StartPoint_.second->SetColor(Ogre::ColourValue::Green);
				imp_.StartPoint_.second->SetRadius(0.02);
			}
			else
			{
				d = imp_.distanceRay_Point(imp_.EndPoint_.second->GetPoint());
				if (d < 0.005)
				{
					//设置选中endpos
					imp_.EndPoint_.second->SetColor(Ogre::ColourValue::Green);
					imp_.EndPoint_.second->SetRadius(0.02);

				}
			}
		}
	}
	break;
	case Imp::ES_Drag:
	{
		if ( evtRecorder.HasMouseReleased(OIS::MB_Left) )
		{
			imp_.StartPoint_.second->SetRadius(0.01);
			imp_.EndPoint_.second->SetRadius(0.01);
			imp_.StartPoint_.second->SetColor(Ogre::ColourValue::Red);
			imp_.EndPoint_.second->SetColor(Ogre::ColourValue::Red);
			imp_.State_ = Imp::ES_Edit;
		}
		else if ( evtRecorder.HasMouseMoved() )
		{
			if (imp_.StartPoint_.second->GetColor() == Ogre::ColourValue::Green)
			{
				int verIndex;
				Ogre::Vector3 pt;
				if ( imp_.QueryPicking(verIndex, pt) )
				{
					imp_.StartPoint_.second->setVisible(true);
					imp_.StartPoint_.second->SetPoint(pt);
					imp_.StartPoint_.first = verIndex;
				}
			}
			if (imp_.EndPoint_.second->GetColor() == Ogre::ColourValue::Green)
			{
				int verIndex;
				Ogre::Vector3 pt;
				if (imp_.QueryPicking(verIndex, pt))
				{
					imp_.EndPoint_.second->setVisible(true);
					imp_.EndPoint_.second->SetPoint(pt);
					imp_.EndPoint_.first = verIndex;
				}
			}

			//求2个点的最短路径		
			std::pair<int, Ogre::Vector3> sp = { imp_.StartPoint_.first, imp_.StartPoint_.second->GetPoint() };
			std::pair<int, Ogre::Vector3> ep = { imp_.EndPoint_.first, imp_.EndPoint_.second->GetPoint() };
			imp_.computerShortestPath(sp, ep);

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
