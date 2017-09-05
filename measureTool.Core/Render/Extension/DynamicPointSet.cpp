#include "DynamicPointSet.h"

#include "Render/RenderGroupID.h"
#include "Render/QueryMask.h"
#include "Render/PrefabResourceMgr.h"

#include "Ogre.h"

static auto maxVerSize = 3000;

DynamicPointSet::DynamicPointSet(const Ogre::IdType& id, Ogre::ObjectMemoryManager *objectMemoryManager)
	:MovableObject(id, objectMemoryManager, ERG_Manipulator_RingFace)
{
	Material_ = PrefabResourceMgr::GetInstance().GetMateiral(PrefabResourceMgr::EPMAT_DebugPointSet);

	RO_.vertexData = new Ogre::VertexData;
	RO_.useIndexes = false;
	RO_.operationType = Ogre::RenderOperation::OT_POINT_LIST;

	auto decl = RO_.vertexData->vertexDeclaration;
	auto& posEle = decl->addElement(0, 0, Ogre::VET_FLOAT3, Ogre::VES_POSITION);

	auto vBuf = Ogre::HardwareBufferManager::getSingleton().createVertexBuffer(decl->getVertexSize(0), maxVerSize, Ogre::HardwareBuffer::HBU_DYNAMIC_WRITE_ONLY, true);
	RO_.vertexData->vertexBufferBinding->setBinding(0, vBuf);
	RO_.vertexData->vertexCount = maxVerSize;

	SetColor(Ogre::ColourValue::White);

	Ogre::AxisAlignedBox box(Ogre::Vector3::ZERO, Ogre::Vector3::UNIT_SCALE);
	SetBound(box);
}

DynamicPointSet::~DynamicPointSet()
{
	delete RO_.vertexData;
}

const Ogre::String& DynamicPointSet::getMovableType(void) const
{
	return DynamicPointSetFactory::GetFactoryName();
}

const Ogre::LightList& DynamicPointSet::getLights(void) const
{
	return queryLights();
}

void DynamicPointSet::getRenderOperation(Ogre::RenderOperation& op)
{
	op = RO_;
}

void DynamicPointSet::getWorldTransforms(Ogre::Matrix4* xform) const
{	
	*xform = getParentNode()->_getFullTransform();
}

void DynamicPointSet::_updateRenderQueue(Ogre::RenderQueue* queue, Ogre::Camera *camera, const Ogre::Camera *lodCamera)
{
	queue->addRenderable(this, mRenderQueueID, mRenderQueuePriority);
}

void DynamicPointSet::visitRenderables(Renderable::Visitor* visitor, bool debugRenderables /*= false*/)
{
	visitor->visit(this, 0, false);
}

const Ogre::MaterialPtr& DynamicPointSet::getMaterial(void) const
{
	return Material_;
}

Ogre::Real DynamicPointSet::getSquaredViewDepth(const Ogre::Camera* cam) const
{
	auto diff = (getParentNode()->_getFullTransform()).getTrans() - cam->getDerivedPosition();
	return diff.squaredLength();
}

const Ogre::UserObjectBindings& DynamicPointSet::GetUserObjectBindings() const
{
	return Ogre::MovableObject::getUserObjectBindings();
}

Ogre::UserObjectBindings& DynamicPointSet::GetUserObjectBindings()
{
	return Ogre::MovableObject::getUserObjectBindings();
}

void DynamicPointSet::SetSmgr(Ogre::SceneManager* smgr)
{
	Smgr_ = smgr;
}

Ogre::SceneManager* DynamicPointSet::GetSmgr() const
{
	return Smgr_;
}

void DynamicPointSet::SetColor(const Ogre::ColourValue& clr)
{
	setCustomParameter(0, Ogre::Vector4(clr.r, clr.g, clr.b, clr.a));
}

void DynamicPointSet::SetBound(const Ogre::AxisAlignedBox& box)
{
	AABB_ = box;
	Ogre::Aabb aabb(AABB_.getCenter(), AABB_.getHalfSize());
	mObjectData.mLocalAabb->setFromAabb(aabb, mObjectData.mIndex);
	mObjectData.mLocalRadius[mObjectData.mIndex] = aabb.getRadius();
}

const Ogre::AxisAlignedBox& DynamicPointSet::GetBound() const
{
	return AABB_;
}

void DynamicPointSet::Update(const std::vector<Ogre::Vector3>& plnList)
{
	auto curSize = static_cast<decltype( maxVerSize )>( plnList.size() );
	auto maxSize = std::min(curSize, maxVerSize);

	RO_.vertexData->vertexCount = maxSize;

	auto decl = RO_.vertexData->vertexDeclaration;

	auto posEle = decl->findElementBySemantic(Ogre::VES_POSITION);

	Ogre::AxisAlignedBox box;

	auto vBuf = RO_.vertexData->vertexBufferBinding->getBuffer(0);
	auto lockSize = maxSize * vBuf->getVertexSize();
	auto pBuf = static_cast<uint8_t*>( vBuf->lock(0, lockSize, Ogre::HardwareBuffer::HBL_WRITE_ONLY) );
	{
		float* pVal;

		for ( auto index = 0; index < maxSize; ++index )
		{
			posEle->baseVertexPointerToElement(pBuf, &pVal);
			auto& curPnt = *reinterpret_cast<Ogre::Vector3*>( pVal );
			curPnt = plnList[index];
			box.merge(curPnt);

			pBuf += vBuf->getVertexSize();
		}

		vBuf->unlock();
	}

	SetBound(box);
}

void DynamicPointSet::Destory()
{
	detachFromParent();

	Smgr_->destroyMovableObject(this);
}