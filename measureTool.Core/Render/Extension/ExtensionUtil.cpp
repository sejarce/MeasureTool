#include "ExtensionUtil.h"

#include "SimpleBoundingBox.h"
#include "Line3D.h"
#include "Point3D.h"
#include "Circle3D.h"
#include "Torus.h"
#include "DynamicPlane.h"
#include "DynamicCurve.h"
#include "DynamicPointSet.h"

#include "Ogre.h"

#include <vector>

class	ExtensionUtil::Imp
{
public:

	using	IExtMovableObjFactoryUPtr = std::unique_ptr<IExtMovableObjFactory>;

public:

	std::vector<IExtMovableObjFactoryUPtr>	FactoryList_;

public:

	static	Imp&	GetInstance()
	{
		static Imp sIns;
		return sIns;
	}
};

void ExtensionUtil::Init()
{
	auto& list = Imp::GetInstance().FactoryList_;

	list.emplace_back(std::make_unique<SimpleBoundingBoxFactory>());
	list.emplace_back(std::make_unique<Line3DFactory>());
	list.emplace_back(std::make_unique<Point3DFactory>());
	list.emplace_back(std::make_unique<Circle3DFactory>());
	list.emplace_back(std::make_unique<TorusFactory>());
	list.emplace_back(std::make_unique<DynamicPlaneFactory>());
	list.emplace_back(std::make_unique<DynamicCurveFactory>());
	list.emplace_back(std::make_unique<DynamicPointSetFactory>());

	for ( auto& cur : list)
	{
		Ogre::Root::getSingletonPtr()->addMovableObjectFactory(cur.get());
		cur->Init();
	}
}

void ExtensionUtil::UnInit()
{
	auto& list = Imp::GetInstance().FactoryList_;

	for ( auto& cur : list )
	{
		cur->UnInit();
	}
}

void ExtensionUtil::CalcBoundsFromVertexBuffer(Ogre::VertexData* vertexData, Ogre::AxisAlignedBox& outAABB, Ogre::Real& outRadius, bool extendOnly /*= false*/)
{
	if ( vertexData->vertexCount == 0 )
	{
		if ( !extendOnly )
		{
			outAABB = Ogre::AxisAlignedBox(Ogre::Vector3::ZERO, Ogre::Vector3::ZERO);
			outRadius = 0;
		}
		return;
	}
	auto elemPos = vertexData->vertexDeclaration->findElementBySemantic(Ogre::VES_POSITION);
	auto vbuf = vertexData->vertexBufferBinding->getBuffer(elemPos->getSource());

	auto vertex = static_cast<unsigned char*>(vbuf->lock(Ogre::HardwareBuffer::HBL_READ_ONLY));

	if ( !extendOnly )
	{
		// init values
		outRadius = 0;
		float* pFloat;
		elemPos->baseVertexPointerToElement(vertex, &pFloat);
		Ogre::Vector3 basePos(pFloat[0], pFloat[1], pFloat[2]);
		outAABB.setExtents(basePos, basePos);
	}

	auto vSize = vbuf->getVertexSize();
	auto vEnd = vertex + vertexData->vertexCount * vSize;
	auto radiusSqr = outRadius * outRadius;
	// Loop through all vertices.
	for ( ; vertex < vEnd; vertex += vSize )
	{
		float* pFloat;
		elemPos->baseVertexPointerToElement(vertex, &pFloat);
		Ogre::Vector3 pos(pFloat[0], pFloat[1], pFloat[2]);
		outAABB.getMinimum().makeFloor(pos);
		outAABB.getMaximum().makeCeil(pos);
		radiusSqr = std::max<Ogre::Real>(radiusSqr, pos.squaredLength());
	}
	outRadius = std::sqrt(radiusSqr);

	vbuf->unlock();
}