#include "DynamicCurve.h"

#include "Render/RenderGroupID.h"
#include "Render/QueryMask.h"
#include "Render/PrefabResourceMgr.h"

#include "Util/MeshUtil.h"

#include "Ogre.h"

static auto maxSize = 3000;

DynamicCurve::DynamicCurve(const Ogre::IdType& id, Ogre::ObjectMemoryManager *objectMemoryManager)
	:MovableObject(id, objectMemoryManager, ERG_Curve)
{
	Material_ = PrefabResourceMgr::GetInstance().GetMateiral(PrefabResourceMgr::EPMAT_Aux_Curve);

	RO_.vertexData = new Ogre::VertexData;
	RO_.useIndexes = true;
	RO_.operationType = Ogre::RenderOperation::OT_TRIANGLE_LIST;

	RO_.indexData = new Ogre::IndexData;

	auto decl = RO_.vertexData->vertexDeclaration;
	auto& posEle = decl->addElement(0, 0, Ogre::VET_FLOAT3, Ogre::VES_POSITION);

	SetColor(Ogre::ColourValue::White);

	Ogre::AxisAlignedBox box(Ogre::Vector3::ZERO, Ogre::Vector3::UNIT_SCALE);
	SetBound(box);
}

DynamicCurve::~DynamicCurve()
{
	delete RO_.vertexData;
	delete RO_.indexData;
}

const Ogre::String& DynamicCurve::getMovableType(void) const
{
	return DynamicCurveFactory::GetFactoryName();
}

const Ogre::LightList& DynamicCurve::getLights(void) const
{
	return queryLights();
}

void DynamicCurve::getRenderOperation(Ogre::RenderOperation& op)
{
	op = RO_;
}

void DynamicCurve::getWorldTransforms(Ogre::Matrix4* xform) const
{	
	*xform = getParentNode()->_getFullTransform();
}

void DynamicCurve::_updateRenderQueue(Ogre::RenderQueue* queue, Ogre::Camera *camera, const Ogre::Camera *lodCamera)
{
	if ( Created_ )
	{
		queue->addRenderable(this, mRenderQueueID, mRenderQueuePriority);
	}
}

void DynamicCurve::visitRenderables(Renderable::Visitor* visitor, bool debugRenderables /*= false*/)
{
	visitor->visit(this, 0, false);
}

const Ogre::MaterialPtr& DynamicCurve::getMaterial(void) const
{
	return Material_;
}

Ogre::Real DynamicCurve::getSquaredViewDepth(const Ogre::Camera* cam) const
{
	auto diff = (getParentNode()->_getFullTransform()).getTrans() - cam->getDerivedPosition();
	return diff.squaredLength();
}

const Ogre::UserObjectBindings& DynamicCurve::GetUserObjectBindings() const
{
	return Ogre::MovableObject::getUserObjectBindings();
}

Ogre::UserObjectBindings& DynamicCurve::GetUserObjectBindings()
{
	return Ogre::MovableObject::getUserObjectBindings();
}

void DynamicCurve::SetSmgr(Ogre::SceneManager* smgr)
{
	Smgr_ = smgr;
}

Ogre::SceneManager* DynamicCurve::GetSmgr() const
{
	return Smgr_;
}

void DynamicCurve::SetColor(const Ogre::ColourValue& clr)
{
	setCustomParameter(0, Ogre::Vector4(clr.r, clr.g, clr.b, clr.a));
}

void DynamicCurve::SetBound(const Ogre::AxisAlignedBox& box)
{
	AABB_ = box;
	Ogre::Aabb aabb(AABB_.getCenter(), AABB_.getHalfSize());
	mObjectData.mLocalAabb->setFromAabb(aabb, mObjectData.mIndex);
	mObjectData.mLocalRadius[mObjectData.mIndex] = aabb.getRadius();
}

void DynamicCurve::UpdateCurve(const std::vector<Ogre::Vector3>& plnList, int segment, float radius, bool closed, bool shrunk)
{
	auto buildInfo = MeshUtil::BuildPipe(plnList, segment, radius, closed);
	auto& verList = std::get<0>(buildInfo);
	auto& indList = std::get<1>(buildInfo);

	auto decl = RO_.vertexData->vertexDeclaration;

	auto needCreateBuf = false;
	if ( !RO_.vertexData->vertexBufferBinding->isBufferBound(0) )
	{
		needCreateBuf = true;
	}
	else
	{
		auto vBuf = RO_.vertexData->vertexBufferBinding->getBuffer(0);
		if ( shrunk )
		{
			if ( vBuf->getNumVertices() != verList.size() )
			{
				needCreateBuf = true;
				RO_.vertexData->vertexBufferBinding->unsetBinding(0);
				RO_.indexData->indexBuffer.setNull();
			}
		}
	}

	if ( needCreateBuf )
	{
		Created_ = true;
		auto verSize = shrunk ? verList.size() : maxSize;
		auto indSize = shrunk ? indList.size() : maxSize * 6;

		auto vBuf = Ogre::HardwareBufferManager::getSingleton().createVertexBuffer(decl->getVertexSize(0), verSize, Ogre::HardwareBuffer::HBU_DYNAMIC_WRITE_ONLY, true);
		RO_.vertexData->vertexBufferBinding->setBinding(0, vBuf);
		RO_.vertexData->vertexCount = verSize;

		RO_.indexData->indexBuffer = Ogre::HardwareBufferManager::getSingleton().createIndexBuffer(Ogre::HardwareIndexBuffer::IT_16BIT, indSize, Ogre::HardwareBuffer::HBU_DYNAMIC_WRITE_ONLY, true);
	}

	RO_.vertexData->vertexCount = verList.size();

	auto posEle = decl->findElementBySemantic(Ogre::VES_POSITION);

	Ogre::AxisAlignedBox box;

	auto vBuf = RO_.vertexData->vertexBufferBinding->getBuffer(0);
	auto lockSize = RO_.vertexData->vertexCount * vBuf->getVertexSize();
	auto pBuf = static_cast<uint8_t*>( vBuf->lock(0, lockSize, Ogre::HardwareBuffer::HBL_WRITE_ONLY) );
	{
		float* pVal;

		for ( auto index = 0U; index < verList.size(); ++index )
		{
			posEle->baseVertexPointerToElement(pBuf, &pVal);
			auto& curPnt1 = *reinterpret_cast<Ogre::Vector3*>( pVal );
			auto& curVer = verList[index];
			curPnt1 = curVer;
			box.merge(curPnt1);

			pBuf += vBuf->getVertexSize();
		}

		vBuf->unlock();
	}

	auto indexSize = RO_.indexData->indexBuffer->getIndexSize() * indList.size();
	auto iBuf = static_cast<int16_t*>( RO_.indexData->indexBuffer->lock(0, indexSize, Ogre::HardwareBuffer::HBL_WRITE_ONLY) );
	{
		for ( auto& cur : indList )
		{
			*iBuf = static_cast<int16_t>( cur );
			++iBuf;
		}

		RO_.indexData->indexBuffer->unlock();
	}
	RO_.indexData->indexStart = 0;
	RO_.indexData->indexCount = indList.size();

	SetBound(box);
}

void DynamicCurve::Destory()
{
	detachFromParent();

	Smgr_->destroyMovableObject(this);
}

void DynamicCurve::SaveToFile(const std::wstring& fileName)
{
	std::vector<Ogre::Vector3> verList;
	{
		verList.reserve(RO_.vertexData->vertexCount);

		auto vBuf = RO_.vertexData->vertexBufferBinding->getBuffer(0);
		auto pBuf = static_cast<uint8_t*>( vBuf->lock(Ogre::HardwareBuffer::HBL_READ_ONLY) );
		auto posEle = RO_.vertexData->vertexDeclaration->findElementBySemantic(Ogre::VES_POSITION);

		float* pVal;

		for ( auto index = 0U; index < RO_.vertexData->vertexCount; ++index )
		{
			posEle->baseVertexPointerToElement(pBuf, &pVal);
			auto& curPnt1 = *reinterpret_cast<Ogre::Vector3*>( pVal );
			verList.push_back(curPnt1);

			pBuf += vBuf->getVertexSize();
		}

		vBuf->unlock();
	}
	

	std::vector<int> indList;
	{
		indList.reserve(RO_.indexData->indexCount);

		auto iBuf = static_cast<int16_t*>( RO_.indexData->indexBuffer->lock(Ogre::HardwareBuffer::HBL_READ_ONLY) );

		for ( auto index = 0U; index < RO_.indexData->indexCount; ++index )
		{
			indList.push_back(*iBuf);
			++iBuf;
		}

		RO_.indexData->indexBuffer->unlock();
	}
	
	MeshUtil::SaveToPly(fileName, verList, indList);

	MeshUtil::SaveToPly(L"pntOnly" + fileName, verList, {});
}
