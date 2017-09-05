#pragma once

#include "Ogre.h"

#include <memory>
#include <array>

namespace OIS
{
	class	MouseState;
}

class QueryUtil
{
public:

	static	Ogre::Ray	GetRayFromCamera(const Ogre::Camera* camera, const OIS::MouseState& mouseState, const Ogre::RenderTarget* rt);

	static	Ogre::Ray	GetRayFromCamera(const Ogre::Camera* camera, const Ogre::Vector2& screenRelPos);

public:

	Ogre::RaySceneQuery *mRaySceneQuery;
	Ogre::RaySceneQuery *mTSMRaySceneQuery;

	Ogre::SceneManager *mSceneMgr;

	QueryUtil(Ogre::SceneManager *sceneMgr);
	~QueryUtil();

	bool raycastFromCamera(Ogre::RenderWindow* rw, Ogre::Camera* camera, const Ogre::Vector2 &mousecoords, Ogre::Vector3 &result, Ogre::MovableObject* &target, float &closest_distance, const Ogre::uint32 queryMask = 0xFFFFFFFF);
	// convenience wrapper with Ogre::Entity to it:
	bool raycastFromCamera(Ogre::RenderWindow* rw, Ogre::Camera* camera, const Ogre::Vector2 &mousecoords, Ogre::Vector3 &result, Ogre::Entity* &target, float &closest_distance, const Ogre::uint32 queryMask = 0xFFFFFFFF);

	bool collidesWithEntity(const Ogre::Vector3& fromPoint, const Ogre::Vector3& toPoint, const float collisionRadius = 2.5f, const float rayHeightLevel = 0.0f, const Ogre::uint32 queryMask = 0xFFFFFFFF);

	void calculateY(Ogre::SceneNode *n, const bool doTerrainCheck = true, const bool doGridCheck = true, const float gridWidth = 1.0f, const Ogre::uint32 queryMask = 0xFFFFFFFF);

	float getTSMHeightAt(const float x, const float z);

	bool raycastFromPoint(const Ogre::Vector3 &point, const Ogre::Vector3 &normal, Ogre::Vector3 &result, Ogre::MovableObject* &target, float &closest_distance, const Ogre::uint32 queryMask = 0xFFFFFFFF);
	// convenience wrapper with Ogre::Entity to it:
	bool raycastFromPoint(const Ogre::Vector3 &point, const Ogre::Vector3 &normal, Ogre::Vector3 &result, Ogre::Entity* &target, float &closest_distance, const Ogre::uint32 queryMask = 0xFFFFFFFF);

	bool raycast(const Ogre::Ray &ray, Ogre::Vector3 &result, Ogre::MovableObject* &target, float &closest_distance, const Ogre::uint32 queryMask = 0xFFFFFFFF);
	// convenience wrapper with Ogre::Entity to it:
	bool raycast(const Ogre::Ray &ray, Ogre::Vector3 &result, Ogre::Entity* &target, float &closest_distance, const Ogre::uint32 queryMask = 0xFFFFFFFF);

	void setHeightAdjust(const float heightadjust);
	float getHeightAdjust(void);

	class	SQueryInfo
	{
	public:

		Ogre::MovableObject*			Movable = nullptr;
		std::vector<Ogre::Renderable*>	Renderables;
		uint32_t						SubIndex = 0;
		float							Distance = 0;
		Ogre::Vector3					InteractionPoint;
		std::array<Ogre::Vector3, 3>	Triangle;
	};
	typedef	std::shared_ptr<SQueryInfo>	SQueryInfoSPtr;
	typedef	std::vector<SQueryInfoSPtr>	SQueryInfoList;

	SQueryInfoList	RayCast(const Ogre::Ray &ray, uint32_t maxNr = 0, Ogre::uint32 queryMask = Ogre::SceneManager::QUERY_ENTITY_DEFAULT_MASK);

	static SQueryInfoList	RayCast(Ogre::RaySceneQuery * query, float radius, const Ogre::Ray &ray, uint32_t maxNr = 0);

private:

	class RenderableDataVisitor;

	float _heightAdjust;

	void GetMovableObjectInformation(Ogre::MovableObject* movable,
									 size_t &vertex_count,
									 std::vector<Ogre::Vector3> &vertices,
									 size_t &index_count,
									 std::vector<Ogre::uint32> &indices,
									 const Ogre::Vector3 &position,
									 const Ogre::Quaternion &orient,
									 const Ogre::Vector3 &scale);
};