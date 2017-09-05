#include "RootController.h"

#include "BrowserController.h"

#include "Render/Camera/MayaCamera.h"
#include "FrameEvent/ToolBarEvent.h"
#include "FrameEvent/ControlEvent.h"

#include "Render/OgreEnv.h"

#include "Ogre.h"
#include "Compositor/OgreCompositorManager2.h"

class 	RootController::Imp
{

public:

	enum EState
	{
		ES_Init,
		ES_Broswer,
		ES_WaitBrowser,
	};

public:

	EState						State_ = ES_Init;
	Ogre::RenderWindow*			RT_{};
	MayaCameraSPtr 				CameraListener_{};
	Ogre::SceneManager*			Smgr_{};
	Ogre::CompositorWorkspace*	WorkSpce_{};
	SFE_OpenDocROM::SPtr		OpenInfo_;

public:

};

RootController::RootController(Ogre::RenderWindow *rt) :ImpUPtr_(new Imp)
{
	auto& imp_ = *ImpUPtr_;

	imp_.RT_ = rt;

	{//smgr
		imp_.Smgr_ = OgreEnv::GetInstance().GetGlobalSmgr();
	}

	{//camera
		auto camera = imp_.Smgr_->createCamera("MainCamera");
		auto mayaCamera = std::make_shared<MayaCamera>(camera, imp_.RT_);

		mayaCamera->SetPosAndTarget({ 0.f, 1.5f, 3.f }, { 0.f, 1.f, 0.f });
		//camera->setProjectionType(Ogre::PT_ORTHOGRAPHIC);
		//camera->setOrthoWindowHeight(3);
		camera->setFOVy(Ogre::Degree(45));
		camera->setNearClipDistance(0.1f);
		camera->setFarClipDistance(10.f);

		imp_.CameraListener_ = mayaCamera;
	}

	auto camera = imp_.CameraListener_->GetCamera();

	{//compositor
		auto comMgr = Ogre::Root::getSingletonPtr()->getCompositorManager2();

		imp_.WorkSpce_ = comMgr->addWorkspace(imp_.Smgr_, imp_.RT_, camera, "measureWorkspace", true);
		
		//auto bgMat = Ogre::MaterialManager::getSingleton().getByName("measureTool/ToneMapping");
		auto bgMat = Ogre::MaterialManager::getSingleton().getByName("measureTool/Background");
		auto texUnit = bgMat->getTechnique(0)->getPass(0)->getTextureUnitState(0);
		Ogre::Image bgImage;
		{
			std::ifstream ifs("qml/res/bg.png", std::ios::binary);
			auto fhds = new Ogre::FileStreamDataStream(&ifs, false);

			Ogre::DataStreamPtr dataPtr(fhds);

			bgImage.load(dataPtr, "png");
		}

		auto texture = Ogre::TextureManager::getSingletonPtr()->createManual("BGTexture", "General",
																			 Ogre::TEX_TYPE_2D,
																			 bgImage.getWidth(), bgImage.getHeight(), 0,
																			 bgImage.getFormat(), Ogre::TU_STATIC);
		texture->getBuffer()->blitFromMemory(bgImage.getPixelBox());
		texUnit->setTexture(texture);
	}

	{//light
		auto light = imp_.Smgr_->createLight();
		imp_.Smgr_->setAmbientLight(Ogre::ColourValue(1.f, 1.f, 1.f));
		auto lightNode = imp_.Smgr_->getRootSceneNode()->createChildSceneNode();
		lightNode->setPosition({ 2000.f, 8000.f, 5000.f });
		lightNode->attachObject(light);
		light->setDirection({ -1.f, -1.f, -1.f });
		light->setDiffuseColour(1.f, 1.f, 1.f);
	}

	{//grid
		auto gridMesh = Ogre::MeshManager::getSingleton().createManual("grid", "General");
		{
			gridMesh->sharedVertexData = new Ogre::VertexData;
			auto subMesh = gridMesh->createSubMesh();
			subMesh->operationType = Ogre::RenderOperation::OT_TRIANGLE_FAN;
			subMesh->useSharedVertices = true;

			auto decl = gridMesh->sharedVertexData->vertexDeclaration;

			auto offset = 0;
			auto& posEle = decl->addElement(0, 0, Ogre::VET_FLOAT3, Ogre::VES_POSITION);
			offset += static_cast<decltype( offset )>( Ogre::VertexElement::getTypeSize(Ogre::VET_FLOAT3) );
			auto& texEle = decl->addElement(0, offset, Ogre::VET_FLOAT2, Ogre::VES_TEXTURE_COORDINATES);

			std::vector<std::tuple<Ogre::Vector3, Ogre::Vector2>> vList;
			auto halfSize = 0.5f;
			auto texScale = 25.f;
			vList.emplace_back(Ogre::Vector3(-halfSize, 0, -halfSize), Ogre::Vector2(0.f, texScale));
			vList.emplace_back(Ogre::Vector3(-halfSize, 0, halfSize), Ogre::Vector2(0.f, 0.f));
			vList.emplace_back(Ogre::Vector3(halfSize, 0, halfSize), Ogre::Vector2(texScale, 0.f));
			vList.emplace_back(Ogre::Vector3(halfSize, 0, -halfSize), Ogre::Vector2(texScale, texScale));

			auto vBuf = Ogre::HardwareBufferManager::getSingleton().createVertexBuffer(decl->getVertexSize(0), vList.size(), Ogre::HardwareBuffer::HBU_STATIC_WRITE_ONLY, true);
			{
				auto pBuf = static_cast<uint8_t*>( vBuf->lock(Ogre::HardwareBuffer::HBL_WRITE_ONLY) );
				float* pVal;

				for ( auto index = 0; index < vList.size(); ++index )
				{
					auto& curPnt = vList[index];
					auto& curPos = std::get<0>(curPnt);
					auto& curTex = std::get<1>(curPnt);

					posEle.baseVertexPointerToElement(pBuf, &pVal);
					*reinterpret_cast<Ogre::Vector3*>( pVal ) = curPos;

					texEle.baseVertexPointerToElement(pBuf, &pVal);
					*reinterpret_cast<Ogre::Vector2*>( pVal ) = curTex;

					pBuf += vBuf->getVertexSize();
				}
				vBuf->unlock();
			}

			gridMesh->sharedVertexData->vertexBufferBinding->setBinding(0, vBuf);
			gridMesh->sharedVertexData->vertexCount = vList.size();
			gridMesh->_updateBoundsFromVertexBuffers();
		}

		auto gridEn = imp_.Smgr_->createEntity(gridMesh);
		gridEn->setMaterialName("measureTool/Grid");

		auto node = imp_.Smgr_->getRootSceneNode()->createChildSceneNode();
		node->attachObject(gridEn);
		node->setScale(2.f, 1.f, 2.f);
	}
}

RootController::~RootController()
{
	auto& imp_ = *ImpUPtr_;

	Unload();

	imp_.CameraListener_.reset();

	auto comMgr = Ogre::Root::getSingletonPtr()->getCompositorManager2();
	comMgr->removeWorkspace(imp_.WorkSpce_);
}

void RootController::_FrameQueue(const Ogre::FrameEvent& fevt)
{
	auto& imp_ = *ImpUPtr_;

	auto& evtRecorder = GetSysEventRecorder();

	switch ( imp_.State_ )
	{
	case Imp::ES_Init:
	{
		auto fe = PopFrameEvent<SFE_OpenDocROM>();
		if ( fe )
		{
			auto child = std::make_shared<BrowserController>(imp_.RT_, imp_.CameraListener_);
			child->HandleFrameEvent(fe->ConvertToFrameEvent());

			AddChild(child);

			imp_.State_ = Imp::ES_Broswer;
		}
	}
	break;
	case Imp::ES_Broswer:
	{
		auto fe = PopFrameEvent<SFE_OpenDocROM>();
		if ( fe )
		{
			imp_.OpenInfo_ = fe;

			RemoveChildrenLazy();
			imp_.State_ = Imp::ES_WaitBrowser;
		}
	}
	break;
	case Imp::ES_WaitBrowser:
	{
		auto child = std::make_shared<BrowserController>(imp_.RT_, imp_.CameraListener_);
		child->HandleFrameEvent(imp_.OpenInfo_->ConvertToFrameEvent());
		AddChild(child);

		imp_.OpenInfo_.reset();
		imp_.State_ = Imp::ES_Broswer;
	}
	break;
	default:
	break;
	}
}

void RootController::_HandleFrameEventImmediately(const FrameEvent& frameEvent)
{
	auto& imp_ = *ImpUPtr_;

	switch ( imp_.State_ )
	{
	case Imp::ES_Broswer:
	{
		if ( frameEvent.GetEventName() == SFE_Finish::StaticFrameEventName() )
		{
			imp_.State_ = Imp::ES_Init;
		}
	}
	break;
	default:
	break;
	}
}
