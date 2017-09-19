#include "BrowserController.h"

#include "EditHeightController.h"
#include "EditDimensionController.h"
#include "EditLineLengthController.h"
#include "EditCurveLengthController.h"
#include "EditFeaturePointController.h"

#include "Render/Camera/MayaCamera.h"
#include "Render/VisibilityFlag.h"
#include "Render/QueryUtil.h"
#include "Render/OgreEnv.h"

#include "FrameEvent/ToolBarEvent.h"
#include "FrameEvent/FloatBarEvent.h"
#include "FrameEvent/ControlEvent.h"

#include "ROM/DocumentROM.h"
#include "ROM/IROM.h"

#include "DOM/Document.h"
#include "DOM/IDOM.h"
#include "DOM/DocumentMgr.h"

#include "Util/MathUtil.h"
#include "Util/CpuTimer.h"
#include "Util/PCLOctree.h"
#include "Util/PCLKdtree.h"
#include "Util/DebugUtil.h"

#include "Ogre.h"
#include "OgreMeshSerializer.h"

#include "BRepBuilderAPI_MakeEdge.hxx"
#include "gce_MakeLin.hxx"
#include "TopoDS_Edge.hxx"

#include <boost/filesystem/fstream.hpp>
#include <boost/optional.hpp>
#include <boost/timer/timer.hpp>
#include <boost/chrono.hpp>

static auto const MaxSelectedSize = 5000U;

class 	BrowserController::Imp
{
public:

	enum EState
	{
		ES_Init,
		ES_Broswer,
		ES_Edit,
		ES_WaitChildExit,
	};

public:

	BrowserController*	ThisPtr_{};
	EState					State_ = ES_Init;

	Ogre::RenderWindow*		RT_{};
	MayaCameraSPtr 			CameraListener_{};
	Ogre::SceneManager*		Smgr_{};

	Ogre::SceneNode*		HumanNode_{};
	Ogre::MeshPtr			HummanMesh_{};
	Ogre::Entity*			HummanEntity_{};
	Ogre::Matrix4			HumanNodeInvTrans_;
	Ogre::Quaternion		HumanNodeInvRot_;

	PCLOctreeSPtr			Octree_;
	PCLKdtreeSPtr			Kdtree_;

	SFE_EditItem::SPtr		CurEditItem_;
	DocumentROMSPtr			DocRom_;

	IROMWPtr				Sweep_;
	IROMWPtr				Select_;

	bool					ShowAllData_{ false };

public:

	void	ResetBrowserInfo()
	{
		auto select = Select_.lock();
		if ( select )
		{
			select->SetPickingState(IROM::EPS_Normal);
			if ( !ShowAllData_ )
			{
				select->SetVisible(false);
			}
		}

		auto sweep = Sweep_.lock();
		if ( sweep )
		{
			sweep->SetPickingState(IROM::EPS_Normal);
			if ( !ShowAllData_ )
			{
				sweep->SetVisible(false);
			}
		}

		Select_.reset();
		Sweep_.reset();
	}

	void	SetSelection(const IROMSPtr& rom)
	{
		auto selection = Select_.lock();
		if ( selection )
		{
			selection->SetPickingState(IROM::EPS_Normal);

			if ( !ShowAllData_ )
			{
				selection->SetVisible(false);
			}
		}
		Select_ = rom;
		if (rom.get()) {
			rom->SetPickingState(IROM::EPS_Select);
			rom->SetVisible(true);
		}
	}

	void	EditItem(const std::shared_ptr<SFE_EditItem>& fe)
	{
		if ( !fe )
		{
			return;
		}

		CurEditItem_ = fe;

		ResetBrowserInfo();

		auto rom = IROM::GetFromDOM(fe->DOM.lock());

		if ( fe->Status == SFE_EditItem::ES_EditMode )
		{
			rom->StashSave();
		}

		auto& wholeList = DocRom_->GetROMList();
		for ( auto& cur : wholeList )
		{
			if ( cur != rom )
			{
				cur->SetDisplayMode(IROM::EDM_Browser);
				cur->SetPickingState(IROM::EPS_Normal);
				cur->SetVisible(false);
			}
		}

		ShowAllData_ = false;

		switch ( fe->Type )
		{
		case SFE_EditItem::EIT_Height:
		{
			auto child = std::make_shared<EditHeightController>(RT_, CameraListener_, HumanNode_, Octree_);
			child->HandleFrameEvent(fe->ConvertToFrameEvent());
			ThisPtr_->AddChild(child);
			State_ = Imp::ES_Edit;
		}
		break;
		case SFE_EditItem::EIT_LineLength:
		{
			auto child = std::make_shared<EditLineLengthController>(RT_, CameraListener_, HumanNode_, Octree_);
			child->HandleFrameEvent(fe->ConvertToFrameEvent());
			ThisPtr_->AddChild(child);
			State_ = Imp::ES_Edit;
		}
		break;
		case SFE_EditItem::EIT_Dimension:
		{
			auto child = std::make_shared<EditDimensionController>(RT_, CameraListener_, HumanNode_, Octree_);
			child->HandleFrameEvent(fe->ConvertToFrameEvent());
			ThisPtr_->AddChild(child);
			State_ = Imp::ES_Edit;
		}
		break;
		case SFE_EditItem::EIT_ConvexDimension:
		{
			auto child = std::make_shared<EditDimensionController>(RT_, CameraListener_, HumanNode_, Octree_);
			child->HandleFrameEvent(fe->ConvertToFrameEvent());
			ThisPtr_->AddChild(child);
			State_ = Imp::ES_Edit;
		}
		break;
		case SFE_EditItem::EIT_CurveLength:
		{
			auto child = std::make_shared<EditCurveLengthController>(RT_, CameraListener_, HumanNode_, Octree_, Kdtree_);
			child->HandleFrameEvent(fe->ConvertToFrameEvent());
			ThisPtr_->AddChild(child);
			State_ = Imp::ES_Edit;
		}
		break;
		default:
		break;
		}
	}
};

BrowserController::BrowserController(Ogre::RenderWindow* rt, const MayaCameraSPtr& camera) :ImpUPtr_(new Imp)
{
	auto& imp_ = *ImpUPtr_;

	imp_.ThisPtr_ = this;
	imp_.RT_ = rt;
	imp_.CameraListener_ = camera;
	imp_.Smgr_ = camera->GetCamera()->getSceneManager();
	imp_.HumanNode_ = imp_.Smgr_->getRootSceneNode()->createChildSceneNode();
}

BrowserController::~BrowserController()
{
	auto& imp_ = *ImpUPtr_;

	Unload();

	imp_.HumanNode_->getParentSceneNode()->removeAndDestroyChild(imp_.HumanNode_);

	imp_.Smgr_->destroyEntity(imp_.HummanEntity_);
}

void BrowserController::_FrameQueue(const Ogre::FrameEvent& fevt)
{
	auto& imp_ = *ImpUPtr_;

	auto& evtRecorder = GetSysEventRecorder();

	switch ( imp_.State_ )
	{
	case Imp::ES_Init:
	{	
		auto fe = PopFrameEvent<SFE_OpenDocROM>();
		assert(fe);

		if (DocumentMgr::GetInstance().GetActiveDocument()->estimateData())
		{
			SFE_EstimateData fe;
			OgreEnv::GetInstance().PostFrameEventToUI(fe.ConvertToFrameEvent());
		}

		auto rom = DocumentMgr::GetInstance().GetActiveDocument()->GetDocumentROM();//fe->DocROM;
		imp_.HummanMesh_ = rom->GetMesh();
		imp_.Octree_ = rom->GetOctree();
		imp_.Kdtree_ = rom->GetKdtree();

		imp_.HummanEntity_ = imp_.Smgr_->createEntity(imp_.HummanMesh_);
		imp_.HummanEntity_->setMaterialName("Mat/Base/VertexColor");

		imp_.HumanNode_->attachObject(imp_.HummanEntity_);
		{
			auto bounds = imp_.HummanMesh_->getBounds();
			auto size = bounds.getSize();
			auto center = bounds.getCenter();

			auto deltaY = -(center.y - size.y / 2);

			imp_.HumanNode_->setPosition({ -center.x, deltaY, -center.z });
		}
		imp_.HumanNodeInvTrans_ = imp_.HumanNode_->_getFullTransformUpdated().inverse();
		imp_.HumanNodeInvRot_ = imp_.HumanNodeInvTrans_.extractQuaternion();

		rom->GetDocROMNode()->setPosition(imp_.HumanNode_->getPosition());
		imp_.DocRom_ = rom;

		imp_.CameraListener_->SetPosAndTarget({ 0.f, 1.5f, 3.f }, { 0.f, 1.f, 0.f });

		if (imp_.DocRom_->GetROMList().size() > 0)
		{
			imp_.Select_ = imp_.DocRom_->GetROMList().front();
		}
		
		imp_.State_ = Imp::ES_Broswer;
	}
	break;
	case Imp::ES_Broswer:
	{
		auto select = imp_.Select_.lock();
		if ( select )
		{
			select->SetVisible(true);
			select->SetPickingState(IROM::EPS_Select);
		}

		auto sweep = imp_.Sweep_.lock();

		if ( evtRecorder.HasKeyPressed(OIS::KC_LCONTROL) || evtRecorder.HasKeyPressed(OIS::KC_S) )
		{
			if ( SysEventRecorder::IsKeyDown(OIS::KC_LCONTROL) && SysEventRecorder::IsKeyDown(OIS::KC_S) )
			{
				SFE_SDC fe;
				fe.ConfirmUIState = SFE_SDC::EUS_ConfiromSave;
				PostFrameEventToUI(fe.ConvertToFrameEvent());
			}
		}

		if ( evtRecorder.HasMousePressed(OIS::MB_Left) )
		{
			if ( sweep && sweep != select )
			{
				if ( select )
				{
					select->SetPickingState(IROM::EPS_Normal);
				}

				imp_.Select_ = sweep;
				sweep->SetPickingState(IROM::EPS_Select);

				SFE_Selection fe;
				fe.SelectedDOM = sweep->GetDOM();
				PostFrameEventToUI(fe.ConvertToFrameEvent());
				break;
			}
		}

		{//selection
			auto fe = PopFrameEvent<SFE_Selection>();
			if ( fe )
			{
				imp_.ResetBrowserInfo();
				auto rom = IROM::GetFromDOM(fe->SelectedDOM);

				imp_.SetSelection(rom);
				rom->SetVisible(true);
				rom->SetPickingState(IROM::EPS_Select);
			}
		}

		{//show all data
			auto fe = PopFrameEvent<SFE_ShowAllData>();
			if ( fe )
			{
				imp_.ShowAllData_ = !imp_.ShowAllData_;

				SFE_FloatBarState sfe;
				if (imp_.ShowAllData_)
					sfe.Contate = SFE_FloatBarState::ETBS_ConAllSee;
				else
					sfe.Contate = SFE_FloatBarState::ETBS_None;
				PostFrameEventToUI(sfe.ConvertToFrameEvent());
			}
		}

		{//Edit
			auto fe = PopFrameEvent<SFE_EditItem>();
			if (fe)
			{
				imp_.EditItem(fe);

				SFE_FloatBarState sfe;
				sfe.Contate = SFE_FloatBarState::ETBS_None;
				PostFrameEventToUI(sfe.ConvertToFrameEvent());
			}
		}

		{//FeaturePoints
			auto fe = PopFrameEvent<SFE_EditFeaturePoints>();
			if (fe && fe->conState == SFE_EditFeaturePoints::E_Show)
			{
				auto child = std::make_shared<EditFeaturePointController>(imp_.RT_, imp_.CameraListener_, imp_.HumanNode_, fe->DOM.lock());
				child->HandleFrameEvent(fe->ConvertToFrameEvent());
				imp_.ThisPtr_->AddChild(child);
				imp_.State_ = Imp::ES_Edit;
			}
		}

		auto& wholeList = imp_.DocRom_->GetROMList();
		for ( auto& curRom : wholeList )
		{
			if ( imp_.ShowAllData_ )
			{
				curRom->SetVisible(true);
			}
			else
			{
				if ( curRom != select && curRom != sweep )
				{
					curRom->SetVisible(false);
				}
			}
		}

		if ( sweep )
		{
			if ( sweep != select )
			{
				sweep->SetPickingState(IROM::EPS_Normal);
			}
			imp_.Sweep_.reset();
		}

		auto cursorRay = QueryUtil::GetRayFromCamera(imp_.CameraListener_->GetCamera(), SysEventRecorder::CachedMouseState(), imp_.RT_);
		Ogre::Ray adjRay(imp_.HumanNodeInvTrans_ * cursorRay.getOrigin(), imp_.HumanNodeInvRot_ * cursorRay.getDirection());

		gce_MakeLin ml(MathUtil::ToOCCTPnt(adjRay.getOrigin()), MathUtil::ToOCCTDir(adjRay.getDirection()));
		auto rayEdge = BRepBuilderAPI_MakeEdge(ml.Value()).Edge();

		std::multimap<float, IROMSPtr> pickingRomList;
		for ( auto& curRom : wholeList )
		{
			if ( !curRom->GetVisible() )
			{
				continue;
			}

			auto curDis = curRom->GetDistanceToRay(rayEdge, 1e-2);
			if ( curDis )
			{
				pickingRomList.emplace(*curDis, curRom);
			}
		}

		if ( pickingRomList.empty() )
		{
			return;
		}

		imp_.Sweep_ = pickingRomList.begin()->second;
		sweep = imp_.Sweep_.lock();

		if ( sweep != select )
		{
			sweep->SetPickingState(IROM::EPS_Sweep);
		}
	}
	break;
	case Imp::ES_Edit:
	{
		{//edit
			auto fe = PopFrameEvent<SFE_EditItem>();
			if ( fe )
			{
				RemoveChildrenLazy();
				imp_.State_ = Imp::ES_WaitChildExit;
				imp_.CurEditItem_ = fe;
				break;
			}
		}

		{//sdc
			auto fe = PopFrameEvent<SFE_SDC>();
			if ( fe )
			{
				RemoveChildrenLazy();
				imp_.State_ = Imp::ES_Broswer;

				if (!imp_.CurEditItem_.get() || !imp_.CurEditItem_->DOM.lock().get())
					break;
				auto rom = IROM::GetFromDOM(imp_.CurEditItem_->DOM.lock());
				if (!rom.get())
					break;
				rom->SetDisplayMode(IROM::EDM_Browser);

				switch ( fe->To3DState )
				{
				case SFE_SDC::ES_Save:
				{
					imp_.Select_ = rom;

					auto dirty = rom->GetDOM()->isDirty();
					rom->GetDOM()->Dirty(false);
					if (dirty)
					{
						//֪ͨui
						SFE_DataDirtyChange sfe;
						sfe.dirty = false;
						PostFrameEventToUI(sfe.ConvertToFrameEvent());
					}

					SFE_SDC sfe;
					sfe.ConfirmUIState = SFE_SDC::EUS_ConfiromDirty;
					PostFrameEventToUI(sfe.ConvertToFrameEvent());
				}
				break;
				case SFE_SDC::ES_Delete:
				{
					auto dirty = rom->GetDOM()->isDirty();
					rom->GetDOM()->Dirty(false);
					if (dirty)
					{
						//֪ͨui
						SFE_DataDirtyChange sfe;
						sfe.dirty = false;
						PostFrameEventToUI(sfe.ConvertToFrameEvent());
					}
				}
				break;
				case SFE_SDC::ES_Cancel:
				{
					rom->StashPop();
					imp_.Select_ = rom;

					auto dirty = rom->GetDOM()->isDirty();
					rom->GetDOM()->Dirty(false);
					if (dirty)
					{
						//֪ͨui
						SFE_DataDirtyChange sfe;
						sfe.dirty = false;
						PostFrameEventToUI(sfe.ConvertToFrameEvent());
					}
				}
				break;
				default:
				break;
				}
			}
		}

		{//selection
			auto fe = PopFrameEvent<SFE_Selection>();
			if (fe)
			{
				imp_.ResetBrowserInfo();
				auto rom = IROM::GetFromDOM(fe->SelectedDOM);

				rom->SetVisible(true);
				imp_.SetSelection(rom);
				rom->SetPickingState(IROM::EPS_Select);
				imp_.Select_ = rom;
			}
		}
	}
	break;
	case Imp::ES_WaitChildExit:
	{
		imp_.EditItem(imp_.CurEditItem_);
	}
	break;
	default:
	break;
	}
}

void BrowserController::_HandleFrameEventImmediately(const FrameEvent& frameEvent)
{
	auto& imp_ = *ImpUPtr_;

	if ( SFE_Finish::StaticFrameEventName() == frameEvent.GetEventName() )
	{
		switch ( imp_.State_ )
		{
		case Imp::ES_Edit:
		{
			imp_.State_ = Imp::ES_Broswer;
		}
		break;
		case Imp::ES_WaitChildExit:
		{
			if ( imp_.CurEditItem_ )
			{
				imp_.EditItem(imp_.CurEditItem_);
			}
		}
		break;
		default:
		break;
		}
	}
}

void BrowserController::_FrameStart(const Ogre::FrameEvent& fevt)
{
	auto& imp_ = *ImpUPtr_;

	auto fe = PopFrameEvent<SFE_ResetCamera>();
	if ( fe )
	{
		imp_.CameraListener_->SetPosAndTarget({ 0.f, 1.5f, 3.f }, { 0.f, 1.f, 0.f });
	}
}

