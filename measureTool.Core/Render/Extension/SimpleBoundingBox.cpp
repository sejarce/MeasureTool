#pragma execution_character_set("utf-8")

#include "SimpleBoundingBox.h"

#include "ExtensionUtil.h"

#include "Ogre.h"

class	SimpleBoundingBoxImp
{
public:

	std::unique_ptr<Ogre::VertexData>	VertexData_;

	static SimpleBoundingBoxImp&	GetInstance()
	{
		static SimpleBoundingBoxImp sIns;
		return sIns;
	}
};


SimpleBoundingBox::SimpleBoundingBox(const Ogre::IdType& id, Ogre::ObjectMemoryManager *objectMemoryManager)
	:MovableObject(id, objectMemoryManager, Ogre::RENDER_QUEUE_MAIN)
{
	RO_.vertexData = SimpleBoundingBoxImp::GetInstance().VertexData_.get();
	RO_.indexData = 0;
	RO_.operationType = Ogre::RenderOperation::OT_LINE_LIST;
	RO_.useIndexes = false;
	RO_.useGlobalInstancingVertexBufferIsAvailable = false;

	Transform_ = Ogre::Matrix4::IDENTITY;
	Material_ = Ogre::MaterialManager::getSingleton().getByName("Mat/Base/LineColor");

	SetColor(Ogre::ColourValue::White);
}

SimpleBoundingBox::~SimpleBoundingBox()
{

}

const Ogre::String& SimpleBoundingBox::getMovableType(void) const
{
	return SimpleBoundingBoxFactory::GetFactoryName();
}

const Ogre::LightList& SimpleBoundingBox::getLights(void) const
{
	return queryLights();
}

void SimpleBoundingBox::getRenderOperation(Ogre::RenderOperation& op)
{
	op = RO_;
}

void SimpleBoundingBox::getWorldTransforms(Ogre::Matrix4* xform) const
{
	*xform = getParentNode()->_getFullTransform() * Transform_;
}

void SimpleBoundingBox::_updateRenderQueue(Ogre::RenderQueue* queue, Ogre::Camera *camera, const Ogre::Camera *lodCamera)
{
	queue->addRenderable(this, mRenderQueueID, mRenderQueuePriority);
}

void SimpleBoundingBox::visitRenderables(Renderable::Visitor* visitor, bool debugRenderables /*= false*/)
{
	visitor->visit(this, 0, false);
}

const Ogre::MaterialPtr& SimpleBoundingBox::getMaterial(void) const
{
	return Material_;
}

Ogre::Real SimpleBoundingBox::getSquaredViewDepth(const Ogre::Camera* cam) const
{
	auto diff = ( getParentNode()->_getFullTransform() * Transform_ ).getTrans() - cam->getDerivedPosition();
	return diff.squaredLength();
}

void SimpleBoundingBox::SetColor(const Ogre::ColourValue& clr)
{
	setCustomParameter(0, Ogre::Vector4(clr.r, clr.g, clr.b, clr.a));
}

void SimpleBoundingBox::SetupBoundingBox(const Ogre::AxisAlignedBox& aabb)
{
	AABB_ = aabb;

	auto center = AABB_.getCenter();
	auto scales = AABB_.getSize();

	auto vmax = AABB_.getMaximum();
	auto vmin = AABB_.getMinimum();

	auto sqLen = std::max(vmax.squaredLength(), vmin.squaredLength());
	BoundRadius_ = Ogre::Math::Sqrt(sqLen);

	Transform_.setTrans(center);
	Transform_.setScale(scales);
}

const Ogre::UserObjectBindings& SimpleBoundingBox::GetUserObjectBindings() const
{
	return Ogre::MovableObject::getUserObjectBindings();
}

Ogre::UserObjectBindings& SimpleBoundingBox::GetUserObjectBindings()
{
	return Ogre::MovableObject::getUserObjectBindings();
}

const Ogre::AxisAlignedBox& SimpleBoundingBox::GetAABox() const
{
	return AABB_;
}

void SimpleBoundingBox::SetSmgr(Ogre::SceneManager* smgr)
{
	Smgr_ = smgr;
}

Ogre::SceneManager* SimpleBoundingBox::GetSmgr() const
{
	return Smgr_;
}

void SimpleBoundingBox::Destory()
{
	detachFromParent();

	Smgr_->destroyMovableObject(this);
}

void SimpleBoundingBoxFactory::Init()
{
	auto& impIns = SimpleBoundingBoxImp::GetInstance();

	impIns.VertexData_ = std::make_unique<Ogre::VertexData>();

	auto vertexData = impIns.VertexData_.get();

	vertexData->vertexCount = 24;
	vertexData->vertexStart = 0;

	auto decl = vertexData->vertexDeclaration;

	decl->addElement(0, 0, Ogre::VET_FLOAT3, Ogre::VES_POSITION);

	auto vbuf = Ogre::HardwareBufferManager::getSingleton().createVertexBuffer(decl->getVertexSize(0), vertexData->vertexCount, Ogre::HardwareBuffer::HBU_STATIC_WRITE_ONLY);
	vertexData->vertexBufferBinding->setBinding(0, vbuf);

	Ogre::Vector3 vmax(.5f, .5f, .5f);
	Ogre::Vector3 vmin(-.5f, -.5f, -.5f);

	auto maxx = vmax.x;
	auto maxy = vmax.y;
	auto maxz = vmax.z;

	auto minx = vmin.x;
	auto miny = vmin.y;
	auto minz = vmin.z;

	float* pPos = static_cast<float*>( vbuf->lock(Ogre::HardwareBuffer::HBL_DISCARD) );

	// line 0
	*pPos++ = minx;
	*pPos++ = miny;
	*pPos++ = minz;
	*pPos++ = maxx;
	*pPos++ = miny;
	*pPos++ = minz;
	// line 1
	*pPos++ = minx;
	*pPos++ = miny;
	*pPos++ = minz;
	*pPos++ = minx;
	*pPos++ = miny;
	*pPos++ = maxz;
	// line 2
	*pPos++ = minx;
	*pPos++ = miny;
	*pPos++ = minz;
	*pPos++ = minx;
	*pPos++ = maxy;
	*pPos++ = minz;
	// line 3
	*pPos++ = minx;
	*pPos++ = maxy;
	*pPos++ = minz;
	*pPos++ = minx;
	*pPos++ = maxy;
	*pPos++ = maxz;
	// line 4
	*pPos++ = minx;
	*pPos++ = maxy;
	*pPos++ = minz;
	*pPos++ = maxx;
	*pPos++ = maxy;
	*pPos++ = minz;
	// line 5
	*pPos++ = maxx;
	*pPos++ = miny;
	*pPos++ = minz;
	*pPos++ = maxx;
	*pPos++ = miny;
	*pPos++ = maxz;
	// line 6
	*pPos++ = maxx;
	*pPos++ = miny;
	*pPos++ = minz;
	*pPos++ = maxx;
	*pPos++ = maxy;
	*pPos++ = minz;
	// line 7
	*pPos++ = minx;
	*pPos++ = maxy;
	*pPos++ = maxz;
	*pPos++ = maxx;
	*pPos++ = maxy;
	*pPos++ = maxz;
	// line 8
	*pPos++ = minx;
	*pPos++ = maxy;
	*pPos++ = maxz;
	*pPos++ = minx;
	*pPos++ = miny;
	*pPos++ = maxz;
	// line 9
	*pPos++ = maxx;
	*pPos++ = maxy;
	*pPos++ = minz;
	*pPos++ = maxx;
	*pPos++ = maxy;
	*pPos++ = maxz;
	// line 10
	*pPos++ = maxx;
	*pPos++ = miny;
	*pPos++ = maxz;
	*pPos++ = maxx;
	*pPos++ = maxy;
	*pPos++ = maxz;
	// line 11
	*pPos++ = minx;
	*pPos++ = miny;
	*pPos++ = maxz;
	*pPos++ = maxx;
	*pPos++ = miny;
	*pPos++ = maxz;
	vbuf->unlock();
}

void SimpleBoundingBoxFactory::UnInit()
{
	auto& impIns = SimpleBoundingBoxImp::GetInstance();

	impIns.VertexData_.reset();
}