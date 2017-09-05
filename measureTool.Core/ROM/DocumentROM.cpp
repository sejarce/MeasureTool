#include "DocumentROM.h"

#include "HeightROM.h"
#include "LineLengthROM.h"
#include "DimensionROM.h"
#include "CurveLengthROM.h"

#include "DOM/Document.h"
#include "DOM/IDOM.h"

#include "Util/MeshUtil.h"

#include "FrameEvent/ToolBarEvent.h"
#include "Render/OgreEnv.h"

#include "Util/PCLOctree.h"
#include "Util/PCLKdtree.h"

#include "Ogre.h"

class	DocumentROM::Imp
{
public:

	using	DocumentDOMWPtr = std::weak_ptr<Document>;
	using	DocumentROMWPtr = std::weak_ptr<DocumentROM>;

public:

	Ogre::SceneManager*		Smgr_{};
	Ogre::SceneNode*		Node_{};

	DocumentDOMWPtr			DocDOM_;
	DocumentROMWPtr			Self_;
	ROMList					ROMList_;
	boost::signals2::connection	OnCreateDOMConn_;
	boost::signals2::connection	OnRemoveDOMConn_;

	Ogre::MeshPtr			LoadedMesh_;
	PCLOctreeSPtr			Octree_;
	PCLKdtreeSPtr			Kdtree_;

public:

	void	CreateROM(const IDOMSPtr& item, bool regSignal)
	{
		IROMSPtr rom;
		switch ( item->GetType() )
		{
		case IDOM::EDT_Height:
		{
			rom = std::make_shared<HeightROM>(Node_, item);
		}
		break;
		case IDOM::EDT_LineLength:
		{
			rom = std::make_shared<LineLengthROM>(Node_, item);
		}
		break;
		case IDOM::EDT_Dimension:
		{
			rom = std::make_shared<DimensionROM>(Node_, item);
		}
		break;
		case IDOM::EDT_ConvexDimension:
		{
			rom = std::make_shared<DimensionROM>(Node_, item);
		}
		break;
		case IDOM::EDT_CurveLength:
		{
			rom = std::make_shared<CurveLengthROM>(Node_, item);
		}
		break;
		default:
		break;
		}

		item->SetROM(rom);
		auto self = Self_;
		std::weak_ptr<IDOM> wpDom = item;

		if ( regSignal )
		{
			item->GetListener().OnChangeState.connect([self, wpDom](IDOM::EDOMState state, bool val)
			{
				auto spSelf = self.lock();
				if ( !spSelf )
				{
					return;
				}

				auto spDom = wpDom.lock();

				auto& imp_ = *( spSelf->ImpUPtr_ );

				if ( state == IDOM::EDS_Delete )
				{
					if ( val )
					{
						imp_.DeleteROM(spDom);
					}
					else
					{
						imp_.CreateROM(spDom, false);
					}
				}
			});
		}
		
		ROMList_.push_back(rom);
	}

	void	DeleteROM(const IDOMSPtr& item)
	{
		auto rom = item->GetROM();
		auto itor = std::remove(ROMList_.begin(), ROMList_.end(), rom);
		ROMList_.erase(itor, ROMList_.end());
	}

	void	Init(DocumentROMWPtr self, const DocumentSPtr& docDOM, Ogre::SceneManager* smgr)
	{
		docDOM->SetDocumentROM(self.lock());

		Smgr_ = smgr;
		Self_ = self;
		DocDOM_ = docDOM;
		Node_ = Smgr_->getRootSceneNode()->createChildSceneNode();

		OnCreateDOMConn_ = docDOM->GetListener().OnCreateDOM.connect([self, this](const IDOMSPtr& item)
		{
			auto sp = self.lock();
			if ( !sp )
			{
				return;
			}

			CreateROM(item, true);
		});

		OnRemoveDOMConn_ = docDOM->GetListener().OnRemoveDOM.connect([self, this](const IDOMSPtr& item)
		{
			auto sp = self.lock();
			if ( !sp )
			{
				return;
			}

			DeleteROM(item);
		});

		auto& pc = docDOM->GetPointCloud();
		auto id = reinterpret_cast<int>( static_cast<void*>( this ) );
		LoadedMesh_ = MeshUtil::BuildMesh(pc, "HumanMesh/" + std::to_string(id));

		for ( auto& cur : docDOM->GetDOMList() )
		{
			CreateROM(cur, true);
		}

		if ( !ROMList_.empty() )
		{
			ROMList_.front()->SetDisplayMode(IROM::EDM_Browser);
			ROMList_.front()->SetPickingState(IROM::EPS_Normal);
			ROMList_.front()->SetVisible(true);
		}

		auto ftr = [&]()
		{
			Octree_ = std::make_shared<PCLOctree>();
			Octree_->Build(LoadedMesh_, 0.005f, 0.02f);
			Octree_->SetFeaturePoint({ 0.157283f, 1.22729f, -0.144646f }, { -0.160695f, 1.23482f, -0.119898f }, { -0.0084175f, 0.74448f, -0.10326f });
//////////////////////////////////////////////////////////////////////////
			Kdtree_ = std::make_shared<PCLKdtree>();
			Kdtree_->Build(LoadedMesh_);
//////////////////////////////////////////////////////////////////////////
						
			SFE_OpenFile fe;
			fe.LoadingState = SFE_OpenFile::ELS_FinishBuildInfo;
			OgreEnv::GetInstance().PostFrameEventToUI(fe.ConvertToFrameEvent());
		};

		OgreEnv::GetInstance().PostThreadTask(ftr);
	}
};

DocumentROM::DocumentROM(PrivateHolder) :ImpUPtr_(std::make_unique<Imp>())
{
	auto& imp_ = *ImpUPtr_;
}

DocumentROM::~DocumentROM()
{
	auto& imp_ = *ImpUPtr_;

	imp_.OnCreateDOMConn_.disconnect();
	imp_.OnRemoveDOMConn_.disconnect();

	imp_.ROMList_.clear();

	imp_.Node_->removeAndDestroyAllChildren();
	imp_.Node_->getParentSceneNode()->removeChild(imp_.Node_);

	Ogre::MeshManager::getSingleton().remove(imp_.LoadedMesh_.staticCast<Ogre::Resource>());
}

DocumentROMSPtr DocumentROM::Create(const DocumentSPtr& docDOM, Ogre::SceneManager* smgr)
{
	auto ret = std::make_shared<DocumentROM>(PrivateHolder());
	ret->ImpUPtr_->Init(ret, docDOM, smgr);

	return ret;
}

Ogre::MeshPtr DocumentROM::GetMesh() const
{
	auto& imp_ = *ImpUPtr_;

	return imp_.LoadedMesh_;
}

Ogre::SceneNode* DocumentROM::GetDocROMNode() const
{
	auto& imp_ = *ImpUPtr_;

	return imp_.Node_;
}

PCLOctreeSPtr DocumentROM::GetOctree() const
{
	auto& imp_ = *ImpUPtr_;

	return imp_.Octree_;
}

PCLKdtreeSPtr DocumentROM::GetKdtree() const
{
	auto& imp_ = *ImpUPtr_;

	return imp_.Kdtree_;
}

const DocumentROM::ROMList& DocumentROM::GetROMList() const
{
	auto& imp_ = *ImpUPtr_;

	return imp_.ROMList_;
}
