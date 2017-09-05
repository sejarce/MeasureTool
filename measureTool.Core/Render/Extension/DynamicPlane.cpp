#include "DynamicPlane.h"

#include "Render/RenderGroupID.h"
#include "Render/QueryMask.h"
#include "Render/PrefabResourceMgr.h"

#include "Ogre.h"

static auto maxSize = 3000;

DynamicPlane::DynamicPlane(const Ogre::IdType& id, Ogre::ObjectMemoryManager *objectMemoryManager)
	:MovableObject(id, objectMemoryManager, ERG_Aux_FittingFace)
{
	Material_ = PrefabResourceMgr::GetInstance().GetMateiral(PrefabResourceMgr::EPMAT_Aux_DynamicFace);

	RO_.vertexData = new Ogre::VertexData;
	RO_.useIndexes = false;
	RO_.operationType = Ogre::RenderOperation::OT_TRIANGLE_FAN;

	auto decl = RO_.vertexData->vertexDeclaration;
	auto& posEle = decl->addElement(0, 0, Ogre::VET_FLOAT3, Ogre::VES_POSITION);

	auto vBuf = Ogre::HardwareBufferManager::getSingleton().createVertexBuffer(decl->getVertexSize(0), maxSize, Ogre::HardwareBuffer::HBU_DYNAMIC_WRITE_ONLY, true);
	RO_.vertexData->vertexBufferBinding->setBinding(0, vBuf);
	RO_.vertexData->vertexCount = maxSize;

	SetColor(Ogre::ColourValue::White);

	Ogre::AxisAlignedBox box(Ogre::Vector3::ZERO, Ogre::Vector3::UNIT_SCALE);
	SetBound(box);
}

DynamicPlane::~DynamicPlane()
{
	delete RO_.vertexData;
}

const Ogre::String& DynamicPlane::getMovableType(void) const
{
	return DynamicPlaneFactory::GetFactoryName();
}

const Ogre::LightList& DynamicPlane::getLights(void) const
{
	return queryLights();
}

void DynamicPlane::getRenderOperation(Ogre::RenderOperation& op)
{
	op = RO_;
}

void DynamicPlane::getWorldTransforms(Ogre::Matrix4* xform) const
{	
	*xform = getParentNode()->_getFullTransform();
}

void DynamicPlane::_updateRenderQueue(Ogre::RenderQueue* queue, Ogre::Camera *camera, const Ogre::Camera *lodCamera)
{
	queue->addRenderable(this, mRenderQueueID, mRenderQueuePriority);
}

void DynamicPlane::visitRenderables(Renderable::Visitor* visitor, bool debugRenderables /*= false*/)
{
	visitor->visit(this, 0, false);
}

const Ogre::MaterialPtr& DynamicPlane::getMaterial(void) const
{
	return Material_;
}

Ogre::Real DynamicPlane::getSquaredViewDepth(const Ogre::Camera* cam) const
{
	auto diff = (getParentNode()->_getFullTransform()).getTrans() - cam->getDerivedPosition();
	return diff.squaredLength();
}

const Ogre::UserObjectBindings& DynamicPlane::GetUserObjectBindings() const
{
	return Ogre::MovableObject::getUserObjectBindings();
}

Ogre::UserObjectBindings& DynamicPlane::GetUserObjectBindings()
{
	return Ogre::MovableObject::getUserObjectBindings();
}

void DynamicPlane::SetSmgr(Ogre::SceneManager* smgr)
{
	Smgr_ = smgr;
}

Ogre::SceneManager* DynamicPlane::GetSmgr() const
{
	return Smgr_;
}

void DynamicPlane::SetColor(const Ogre::ColourValue& clr)
{
	setCustomParameter(0, Ogre::Vector4(clr.r, clr.g, clr.b, clr.a));
}

void DynamicPlane::SetBound(const Ogre::AxisAlignedBox& box)
{
	AABB_ = box;
	Ogre::Aabb aabb(AABB_.getCenter(), AABB_.getHalfSize());
	mObjectData.mLocalAabb->setFromAabb(aabb, mObjectData.mIndex);
	mObjectData.mLocalRadius[mObjectData.mIndex] = aabb.getRadius();
}

void DynamicPlane::UpdatePln(const std::vector<Ogre::Vector3>& plnList)
{
	RO_.vertexData->vertexCount = plnList.size() + 2;

	auto decl = RO_.vertexData->vertexDeclaration;

	auto posEle = decl->findElementBySemantic(Ogre::VES_POSITION);

	Ogre::AxisAlignedBox box;

	auto vBuf = RO_.vertexData->vertexBufferBinding->getBuffer(0);
	auto lockSize = RO_.vertexData->vertexCount * vBuf->getVertexSize();
	auto pBuf = static_cast<uint8_t*>( vBuf->lock(0, lockSize, Ogre::HardwareBuffer::HBL_WRITE_ONLY) );
	{
		float* pVal;

		auto pFirst = pBuf;

		pBuf += vBuf->getVertexSize();

		for ( auto index = 0U; index < plnList.size(); ++index )
		{
			posEle->baseVertexPointerToElement(pBuf, &pVal);
			auto& curPnt1 = *reinterpret_cast<Ogre::Vector3*>( pVal );
			curPnt1 = plnList[index];
			box.merge(curPnt1);

			pBuf += vBuf->getVertexSize();
		}

		{//first
			posEle->baseVertexPointerToElement(pFirst, &pVal);
			*reinterpret_cast<Ogre::Vector3*>( pVal ) = box.getCenter();
		}

		{//last
			auto& firstPnt = plnList.front();
			posEle->baseVertexPointerToElement(pBuf, &pVal);
			*reinterpret_cast<Ogre::Vector3*>( pVal ) = firstPnt;
			pBuf += vBuf->getVertexSize();
		}

		vBuf->unlock();
	}

	SetBound(box);
}

void DynamicPlane::Destory()
{
	detachFromParent();

	Smgr_->destroyMovableObject(this);
}