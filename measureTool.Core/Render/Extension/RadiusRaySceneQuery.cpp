#pragma execution_character_set("utf-8")

#include "RadiusRaySceneQuery.h"
#include "Render/VisibilityFlag.h"

#include "Math/Array/OgreMathlib.h"
#include "Math/Array/OgreArrayAabb.h"
#include "Math/Array/OgreArraySphere.h"
#include "Math/Array/OgreBooleanMask.h"

#include "Ogre.h"
#include "OgrePlatform.h"

using namespace Ogre;

RadiusRaySceneQuery::RadiusRaySceneQuery(Ogre::SceneManager* creator) : RaySceneQuery(creator)
{
	mSupportedWorldFragments.insert(SceneQuery::WFT_NONE);
}

RadiusRaySceneQuery::~RadiusRaySceneQuery()
{

}

void RadiusRaySceneQuery::execute(Ogre::RaySceneQueryListener* listener )
{
	assert(mFirstRq < mLastRq && "This query will never hit any result!");

	for ( size_t i = 0; i < Ogre::NUM_SCENE_MEMORY_MANAGER_TYPES; ++i )
	{
		auto& memoryManager = mParentSceneMgr->_getEntityMemoryManager(static_cast<Ogre::SceneMemoryMgrTypes>(i));

		const auto numRenderQueues = memoryManager.getNumRenderQueues();

		auto keepIterating = true;
		auto firstRq = std::min<size_t>(mFirstRq, numRenderQueues);
		auto lastRq = std::min<size_t>(mLastRq, numRenderQueues);

		for ( auto j = firstRq; j < lastRq && keepIterating; ++j )
		{
			Ogre::ObjectData objData;
			const size_t totalObjs = memoryManager.getFirstObjectData(objData, j);
			keepIterating = _Execute(objData, totalObjs, listener);
		}
	}
}

bool RadiusRaySceneQuery::_Execute(Ogre::ObjectData objData, size_t numNodes, Ogre::RaySceneQueryListener* listener)
{
	Ogre::ArrayVector3 rayOrigin;
	Ogre::ArrayVector3 rayDir;

	auto ourQueryMask = Ogre::Mathlib::SetAll(mQueryMask);

	rayOrigin.setAll(mRay.getOrigin());
	rayDir.setAll(mRay.getDirection());

	auto hitRaduis = Ogre::Mathlib::SetAll(Radius_);
	Ogre::ArrayVector3 halfHitSize(hitRaduis, hitRaduis, hitRaduis);

	for ( size_t i = 0; i < numNodes; i += ARRAY_PACKED_REALS )
	{
		// Check origin inside first
		auto hitMaskR = objData.mWorldAabb->contains(rayOrigin);

		auto distance = Ogre::Mathlib::CmovRobust(ARRAY_REAL_ZERO, Ogre::Mathlib::INFINITEA, hitMaskR);

		auto vMin = objData.mWorldAabb->getMinimum();
		auto vMax = objData.mWorldAabb->getMaximum();

		auto*RESTRICT_ALIAS visibilityFlags = reinterpret_cast<Ogre::ArrayInt*RESTRICT_ALIAS>(objData.mVisibilityFlags);
		auto*RESTRICT_ALIAS queryFlags = reinterpret_cast<Ogre::ArrayInt*RESTRICT_ALIAS>(objData.mQueryFlags);

		// Check each face in turn
		// Min x, y & z
		for ( size_t j = 0; j < 3; ++j )
		{
			auto t = (vMin.mChunkBase[j] - rayOrigin.mChunkBase[j]) / rayDir.mChunkBase[j];

			//mask = t >= 0; works even if t is nan (t = 0 / 0)
			auto mask = Ogre::Mathlib::CompareGreaterEqual(t, ARRAY_REAL_ZERO);
			auto hitPoint = rayOrigin + rayDir * t;

			//Fix accuracy issues for very thin aabbs
			hitPoint.mChunkBase[j] = objData.mWorldAabb->mCenter.mChunkBase[j];

			Ogre::ArrayAabb hitAabb(hitPoint, halfHitSize);

			//hitMaskR |= t >= 0 && mWorldAabb->contains( hitPoint );
			//distance = t >= 0 ? min( distance, t ) : t;
			hitMaskR = Ogre::Mathlib::Or(hitMaskR, Ogre::Mathlib::And(mask, hitAabb.intersects(*(objData.mWorldAabb))));
			distance = Ogre::Mathlib::CmovRobust(Ogre::Mathlib::Min(distance, t), distance, mask);
		}

		// Max x, y & z
		for ( size_t j = 0; j < 3; ++j )
		{
			auto t = (vMax.mChunkBase[j] - rayOrigin.mChunkBase[j]) / rayDir.mChunkBase[j];

			//mask = t >= 0; works even if t is nan (t = 0 / 0)
			auto mask = Ogre::Mathlib::CompareGreaterEqual(t, ARRAY_REAL_ZERO);
			auto hitPoint = rayOrigin + rayDir * t;

			//Fix accuracy issues for very thin aabbs
			hitPoint.mChunkBase[j] = objData.mWorldAabb->mCenter.mChunkBase[j];

			Ogre::ArrayAabb hitAabb(hitPoint, halfHitSize);

			//hitMaskR |= t >= 0 && mWorldAabb->contains( hitPoint );
			//distance = t >= 0 ? min( distance, t ) : t;
			hitMaskR = Ogre::Mathlib::Or(hitMaskR, Ogre::Mathlib::And(mask, hitAabb.intersects(*(objData.mWorldAabb))));
			distance = Ogre::Mathlib::CmovRobust(Ogre::Mathlib::Min(distance, t), distance, mask);
		}

		//hitMask = hitMask && ( (*queryFlags & ourQueryMask) != 0 ) && isVisble;
		auto hitMask = CastRealToInt(hitMaskR);
		hitMask = Ogre::Mathlib::And(hitMask, Ogre::Mathlib::TestFlags4(*queryFlags, ourQueryMask));
		hitMask = Ogre::Mathlib::And(hitMask,
									 Ogre::Mathlib::TestFlags4(*visibilityFlags,
									 Ogre::Mathlib::SetAll(uint32_t(EVF_QueryVisible))));

		const auto scalarMask = Ogre::BooleanMask4::getScalarMask(hitMask);
		OGRE_ALIGNED_DECL(Ogre::Real, scalarDistance[ARRAY_PACKED_REALS], OGRE_SIMD_ALIGNMENT);
		CastArrayToReal(scalarDistance, distance);

		for ( size_t j = 0; j < ARRAY_PACKED_REALS; ++j )
		{
			//Decompose the result for analyzing each MovableObject's
			//There's no need to check objData.mOwner[j] is null because
			//we set mVisibilityFlags to 0 on slot removals
			if ( IS_BIT_SET(j, scalarMask) )
			{
				if ( !listener->queryResult(objData.mOwner[j], scalarDistance[j]) )
					return false;
			}

#ifndef NDEBUG
			//Queries must be performed after all bounds have been updated
			//(i.e. SceneManager::updateSceneGraph does this for you), and don't
			//move the objects between that call and this query.
			//Ignore out of date Aabbs from objects that have been
			//explicitly disabled or fail the query mask.
			assert(!objData.mOwner[j]->isCachedAabbOutOfDate() &&
				   "Perform the queries after MovableObject::updateAllBounds has been called!");
#endif
		}

		objData.advancePack();
	}

	return true;
}

void RadiusRaySceneQuery::SetRadius(float val)
{
	Radius_ = val;
}

float RadiusRaySceneQuery::GetRadius() const
{
	return Radius_;
}

void RadiusRaySceneQuery::SetRqRange(int first, int last)
{
	mFirstRq = first;
	mLastRq = last + 1;
}
