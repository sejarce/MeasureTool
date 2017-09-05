#include "QueryUtil.h"

#include "SysEvent.h"
#include "SysEventRecorder.h"

class QueryUtil::RenderableDataVisitor :public Ogre::Renderable::Visitor
{
public:
	virtual void visit(Ogre::Renderable* rend, Ogre::ushort lodIndex, bool isDebug, Ogre::Any* pAny /* = 0 */) override
	{
		Ogre::RenderOperation ro;
		rend->getRenderOperation(ro);

		Ogre::VertexData* vertex_data = ro.vertexData;

		const Ogre::VertexElement* posElem =
			vertex_data->vertexDeclaration->findElementBySemantic(Ogre::VES_POSITION);

		Ogre::HardwareVertexBufferSharedPtr vbuf =
			vertex_data->vertexBufferBinding->getBuffer(posElem->getSource());

		unsigned char* vertex =
			static_cast<unsigned char*>(vbuf->lock(Ogre::HardwareBuffer::HBL_READ_ONLY));

		// There is _no_ baseVertexPointerToElement() which takes an Ogre::Ogre::Real or a double
		//  as second argument. So make it float, to avoid trouble when Ogre::Ogre::Real will
		//  be comiled/typedefed as double:
		//      Ogre::Ogre::Real* pOgre::Real;
		float* pReal;

		for (size_t j = 0; j < vertex_data->vertexCount; ++j, vertex += vbuf->getVertexSize())
		{
			posElem->baseVertexPointerToElement(vertex, &pReal);

			Ogre::Vector3 pt(pReal[0], pReal[1], pReal[2]);
			vertices.push_back(pt);
		}

		vbuf->unlock();


		Ogre::IndexData* index_data = ro.indexData;
		size_t numTris = index_data->indexCount / 3;
		Ogre::HardwareIndexBufferSharedPtr ibuf = index_data->indexBuffer;

		bool use32bitindexes = (ibuf->getType() == Ogre::HardwareIndexBuffer::IT_32BIT);

		Ogre::uint32*  pLong = static_cast<Ogre::uint32*>(ibuf->lock(Ogre::HardwareBuffer::HBL_READ_ONLY));
		unsigned short* pShort = reinterpret_cast<unsigned short*>(pLong);

		if (use32bitindexes)
		{
			for (size_t k = 0; k < numTris * 3; ++k)
			{
				indices.push_back(pLong[k] + offset);
			}
		}
		else
		{
			for (size_t k = 0; k < numTris * 3; ++k)
			{
				indices.push_back(static_cast<Ogre::uint32>(pShort[k]) + offset);
			}
		}

		ibuf->unlock();
		offset = static_cast<Ogre::uint32>( vertices.size() );
	}

public:
	Ogre::uint32 offset = 0;
	std::vector<Ogre::Vector3> vertices;
	std::vector<Ogre::uint32>  indices;
};

class	SimpleRenderableVisitor : public Ogre::Renderable::Visitor
{
public:

	std::vector<Ogre::Renderable*>	RenderableList_;

public:

	virtual void visit(Ogre::Renderable* rend, Ogre::ushort lodIndex, bool isDebug, Ogre::Any* pAny = 0) override
	{
		RenderableList_.push_back(rend);
	}
};

QueryUtil::QueryUtil(Ogre::SceneManager *sceneMgr)
{
	mSceneMgr = sceneMgr;

	mRaySceneQuery = mSceneMgr->createRayQuery(Ogre::Ray());
	if ( NULL == mRaySceneQuery )
	{
		// LOG_ERROR << "Failed to create Ogre::RaySceneQuery instance" << ENDLOG;
		return;
	}
	mRaySceneQuery->setSortByDistance(true);

	mTSMRaySceneQuery = mSceneMgr->createRayQuery(Ogre::Ray());

	_heightAdjust = 0.0f;
}

QueryUtil::~QueryUtil()
{
	if ( mRaySceneQuery != NULL )
		delete mRaySceneQuery;

	if ( mTSMRaySceneQuery != NULL )
		delete mTSMRaySceneQuery;
}

bool QueryUtil::raycastFromCamera(Ogre::RenderWindow* rw, Ogre::Camera* camera, const Ogre::Vector2 &mousecoords, Ogre::Vector3 &result, Ogre::Entity* &target, float &closest_distance, const Ogre::uint32 queryMask)
{
	return raycastFromCamera(rw, camera, mousecoords, result, (Ogre::MovableObject*&) target, closest_distance, queryMask);
}

bool QueryUtil::raycastFromCamera(Ogre::RenderWindow* rw, Ogre::Camera* camera, const Ogre::Vector2 &mousecoords, Ogre::Vector3 &result, Ogre::MovableObject* &target, float &closest_distance, const Ogre::uint32 queryMask)
{
	// Create the ray to test
	Ogre::Real tx = mousecoords.x / (Ogre::Real) rw->getWidth();
	Ogre::Real ty = mousecoords.y / (Ogre::Real) rw->getHeight();
	Ogre::Ray ray = camera->getCameraToViewportRay(tx, ty);

	return raycast(ray, result, target, closest_distance, queryMask);
}

bool QueryUtil::collidesWithEntity(const Ogre::Vector3& fromPoint, const Ogre::Vector3& toPoint, const float collisionRadius, const float rayHeightLevel, const Ogre::uint32 queryMask)
{
	Ogre::Vector3 fromPointAdj(fromPoint.x, fromPoint.y + rayHeightLevel, fromPoint.z);
	Ogre::Vector3 toPointAdj(toPoint.x, toPoint.y + rayHeightLevel, toPoint.z);
	Ogre::Vector3 normal = toPointAdj - fromPointAdj;
	float distToDest = normal.normalise();

	Ogre::Vector3 myResult(0, 0, 0);
	Ogre::MovableObject* myObject = NULL;
	float distToColl = 0.0f;

	if ( raycastFromPoint(fromPointAdj, normal, myResult, myObject, distToColl, queryMask) )
	{
		distToColl -= collisionRadius;
		return (distToColl <= distToDest);
	}
	else
	{
		return false;
	}
}

float QueryUtil::getTSMHeightAt(const float x, const float z)
{
	float y = 0.0f;

	static Ogre::Ray updateRay;

	updateRay.setOrigin(Ogre::Vector3(x, 9999, z));
	updateRay.setDirection(Ogre::Vector3::NEGATIVE_UNIT_Y);

	mTSMRaySceneQuery->setRay(updateRay);
	Ogre::RaySceneQueryResult& qryResult = mTSMRaySceneQuery->execute();

	Ogre::RaySceneQueryResult::iterator i = qryResult.begin();
	if ( i != qryResult.end() && i->worldFragment )
	{
		y = i->worldFragment->singleIntersection.y;
	}
	return y;
}

void QueryUtil::calculateY(Ogre::SceneNode *n, const bool doTerrainCheck, const bool doGridCheck, const float gridWidth, const Ogre::uint32 queryMask)
{
	Ogre::Vector3 pos = n->getPosition();

	float x = pos.x;
	float z = pos.z;
	float y = pos.y;

	Ogre::Vector3 myResult(0, 0, 0);
	Ogre::MovableObject *myObject = NULL;
	float distToColl = 0.0f;

	float terrY = 0, colY = 0, colY2 = 0;

	if ( raycastFromPoint(Ogre::Vector3(x, y, z), Ogre::Vector3::NEGATIVE_UNIT_Y, myResult, myObject, distToColl, queryMask) )
	{
		if ( myObject != NULL )
		{
			colY = myResult.y;
		}
		else
		{
			colY = -99999;
		}
	}

	//if doGridCheck is on, repeat not to fall through small holes for example when crossing a hangbridge
	if ( doGridCheck )
	{
		if ( raycastFromPoint(Ogre::Vector3(x, y, z) + (n->getOrientation()*Ogre::Vector3(0, 0, gridWidth)), Ogre::Vector3::NEGATIVE_UNIT_Y, myResult, myObject, distToColl, queryMask) )
		{
			if ( myObject != NULL )
			{
				colY = myResult.y;
			}
			else
			{
				colY = -99999;
			}
		}
		if ( colY < colY2 ) colY = colY2;
	}

	// set the parameter to false if you are not using ETM or TSM
	if ( doTerrainCheck )
	{
		// TSM height value
		terrY = getTSMHeightAt(x, z);

		if ( terrY < colY )
		{
			n->setPosition(x, colY + _heightAdjust, z);
		}
		else
		{
			n->setPosition(x, terrY + _heightAdjust, z);
		}
	}
	else
	{
		if ( !doTerrainCheck && colY == -99999 ) colY = y;
		n->setPosition(x, colY + _heightAdjust, z);
	}
}

// raycast from a point in to the scene.
// returns success or failure.
// on success the point is returned in the result.
bool QueryUtil::raycastFromPoint(const Ogre::Vector3 &point,
										const Ogre::Vector3 &normal,
										Ogre::Vector3 &result, Ogre::Entity* &target,
										float &closest_distance,
										const Ogre::uint32 queryMask)
{
	return raycastFromPoint(point, normal, result, (Ogre::MovableObject*&) target, closest_distance, queryMask);
}

bool QueryUtil::raycastFromPoint(const Ogre::Vector3 &point,
										const Ogre::Vector3 &normal,
										Ogre::Vector3 &result, Ogre::MovableObject* &target,
										float &closest_distance,
										const Ogre::uint32 queryMask)
{
	// create the ray to test
	static Ogre::Ray ray;
	ray.setOrigin(point);
	ray.setDirection(normal);

	return raycast(ray, result, target, closest_distance, queryMask);
}

bool QueryUtil::raycast(const Ogre::Ray &ray, Ogre::Vector3 &result, Ogre::Entity* &target, float &closest_distance, const Ogre::uint32 queryMask)
{
	return raycast(ray, result, (Ogre::MovableObject*&)target, closest_distance, queryMask);
}

bool QueryUtil::raycast(const Ogre::Ray &ray, Ogre::Vector3 &result, Ogre::MovableObject* &target, float &closest_distance, const Ogre::uint32 queryMask)
{
	target = NULL;

	// check we are initialised
	if ( mRaySceneQuery != NULL )
	{
		// create a query object
		mRaySceneQuery->setRay(ray);
		mRaySceneQuery->setSortByDistance(true);
		mRaySceneQuery->setQueryMask(queryMask);
		// execute the query, returns a vector of hits
		if ( mRaySceneQuery->execute().size() <= 0 )
		{
			// raycast did not hit an objects bounding box
			return (false);
		}
	}
	else
	{
		//LOG_ERROR << "Cannot raycast without RaySceneQuery instance" << ENDLOG;
		return (false);
	}

	// at this point we have raycast to a series of different objects bounding boxes.
	// we need to test these different objects to see which is the first polygon hit.
	// there are some minor optimizations (distance based) that mean we wont have to
	// check all of the objects most of the time, but the worst case scenario is that
	// we need to test every triangle of every object.
	//Ogre::Ogre::Real closest_distance = -1.0f;
	closest_distance = -1.0f;
	Ogre::Vector3 closest_result;
	Ogre::RaySceneQueryResult &query_result = mRaySceneQuery->getLastResults();
	for ( size_t qr_idx = 0; qr_idx < query_result.size(); qr_idx++ )
	{
		// stop checking if we have found a raycast hit that is closer
		// than all remaining entities
		if ( (closest_distance >= 0.0f) &&
			(closest_distance < query_result[qr_idx].distance) )
		{
			break;
		}

		// only check this result if its a hit against an entity
		if ( (query_result[qr_idx].movable != NULL) 
			/*&&(query_result[qr_idx].movable->getMovableType().compare("Entity") == 0)*/ )
		{
			// get the entity to check
			Ogre::MovableObject *pentity = static_cast<Ogre::MovableObject*>(query_result[qr_idx].movable);

			// mesh data to retrieve
			size_t vertex_count;
			size_t index_count;
			std::vector<Ogre::Vector3> vertices;
			std::vector<Ogre::uint32> indices;

			// get the mesh information
			GetMovableObjectInformation(pentity, vertex_count, vertices, index_count, indices,
								pentity->getParentNode()->_getDerivedPosition(),
								pentity->getParentNode()->_getDerivedOrientation(),
								pentity->getParentNode()->_getDerivedScale());

			// test for hitting individual triangles on the mesh
			bool new_closest_found = false;
			for ( size_t i = 0; i < index_count; i += 3 )
			{
				// check for a hit against this triangle
				std::pair<bool, Ogre::Real> hit = Ogre::Math::intersects(ray, vertices[indices[i]],
																			vertices[indices[i + 1]], vertices[indices[i + 2]], true, false);

				// if it was a hit check if its the closest
				if ( hit.first )
				{
					if ( (closest_distance < 0.0f) ||
						(hit.second < closest_distance) )
					{
						// this is the closest so far, save it off
						closest_distance = hit.second;
						new_closest_found = true;
					}
				}
			}

			// if we found a new closest raycast for this object, update the
			// closest_result before moving on to the next object.
			if ( new_closest_found )
			{
				target = pentity;
				closest_result = ray.getPoint(closest_distance);
			}
		}
	}

	// return the result
	if ( closest_distance >= 0.0f )
	{
		// raycast success
		result = closest_result;
		return (true);
	}
	else
	{
		// raycast failed
		return (false);
	}
}


// Get the mesh information for the given mesh.
// Code found on this forum link: http://www.ogre3d.org/wiki/index.php/RetrieveVertexData
void QueryUtil::GetMovableObjectInformation(Ogre::MovableObject* movable,
										size_t &vertex_count,
										std::vector<Ogre::Vector3> &vertices,
										size_t &index_count,
										std::vector<Ogre::uint32> &indices,
										const Ogre::Vector3 &position,
										const Ogre::Quaternion &orient,
										const Ogre::Vector3 &scale)
{
	vertex_count = index_count = 0;

	RenderableDataVisitor* visitor = new RenderableDataVisitor;
	movable->visitRenderables(visitor);

	vertices = std::move(visitor->vertices);
	indices = std::move(visitor->indices);

	for (auto& vertex:vertices)
	{
		vertex = orient*(vertex*scale) + position;
	}

	vertex_count = vertices.size();
	index_count = indices.size();
	
	delete visitor;
}

void QueryUtil::setHeightAdjust(const float heightadjust)
{
	_heightAdjust = heightadjust;
}

float QueryUtil::getHeightAdjust(void)
{
	return _heightAdjust;
}

QueryUtil::SQueryInfoList QueryUtil::RayCast(const Ogre::Ray &ray, uint32_t maxNr /*= 0*/, Ogre::uint32 queryMask /*= Ogre::SceneManager::QUERY_ENTITY_DEFAULT_MASK*/)
{
	//mRaySceneQuery->setQueryMask(queryMask);
	//
	//return RayCast(mRaySceneQuery, ray, maxNr);

	return{};
}

static Ogre::Vector3 ProjectPointOnLine(const Ogre::Vector3& point, const Ogre::Vector3& param1, const Ogre::Vector3& param2)
{
	auto vecA = point - param1;
	auto vecB = param2 - param1;

	auto vecD = vecA - vecA.dotProduct(vecB) * vecB / vecB.squaredLength();

	auto pointD = point - vecD;

	return pointD;
}

QueryUtil::SQueryInfoList QueryUtil::RayCast(Ogre::RaySceneQuery * query, float radius, const Ogre::Ray &ray, uint32_t maxNr /*= 0*/)
{
	SQueryInfoList ret;

	if ( query == nullptr )
	{
		return ret;
	}

	query->setRay(ray);
	query->setSortByDistance(true, maxNr);
	if ( query->execute().empty() )
	{
		return ret;
	}

	auto& queryList = query->getLastResults();
	for ( auto& curQuery : queryList )
	{
		assert(curQuery.movable);

		auto curMeshQueryInfo = std::make_shared<SQueryInfo>();
		curMeshQueryInfo->Movable = curQuery.movable;
		curMeshQueryInfo->Distance = std::numeric_limits<float>::max();

		Ogre::Matrix4 transformation, invTransformation, normalTransformation;
		{
			Ogre::Matrix4 s, r, t;
			s = Ogre::Matrix4::IDENTITY;
			r = Ogre::Matrix4::IDENTITY;
			t = Ogre::Matrix4::IDENTITY;
			Ogre::Matrix3 r3;
			auto translation = curQuery.movable->getParentSceneNode()->_getDerivedPosition();
			auto rotation = curQuery.movable->getParentSceneNode()->_getDerivedOrientation();
			auto scale = curQuery.movable->getParentSceneNode()->_getDerivedScale();
			t.setTrans(translation);
			rotation.ToRotationMatrix(r3);
			s.setScale(scale);
			r = r3;

			transformation = t * r * s;
			normalTransformation = r;
			invTransformation = transformation;
			invTransformation.inverse();
		}

		SimpleRenderableVisitor srv;
		curQuery.movable->visitRenderables(&srv);
		curMeshQueryInfo->Renderables = srv.RenderableList_;

		auto subIndex = 0U;
		for ( auto curRend : srv.RenderableList_ )
		{
			auto foundHit = false;

			Ogre::RenderOperation op;
			curRend->getRenderOperation(op);

			auto vertexData = op.vertexData;
			auto posElem = vertexData->vertexDeclaration->findElementBySemantic(Ogre::VES_POSITION);
			auto normalElem = vertexData->vertexDeclaration->findElementBySemantic(Ogre::VES_NORMAL);
			auto vertexBuf = vertexData->vertexBufferBinding->getBuffer(posElem->getSource());
			auto pVertex = static_cast<uint8_t*>(vertexBuf->lock(Ogre::HardwareBuffer::HBL_READ_ONLY));
			float *pReal = nullptr;

			if ( op.operationType == Ogre::RenderOperation::OT_POINT_LIST )
			{
				Ogre::Vector3 v, n;
				auto squareRadius = radius * radius;

				for ( auto vIndex = 0; vIndex < vertexData->vertexCount; ++vIndex )
				{
					auto pVertex1 = pVertex + ( vertexBuf->getVertexSize() ) * vIndex;
					posElem->baseVertexPointerToElement(pVertex1, &pReal);
					v = *reinterpret_cast<Ogre::Vector3*>( pReal );
					v = transformation * v;

					auto pntOnRay = ProjectPointOnLine(v, ray.getOrigin(), ray.getPoint(1));

					if ( pntOnRay.squaredDistance(v) < squareRadius )
					{
						foundHit = true;
						curMeshQueryInfo->Distance = std::min(pntOnRay.distance(ray.getOrigin()), curMeshQueryInfo->Distance);
						curMeshQueryInfo->InteractionPoint = pntOnRay;
						curMeshQueryInfo->SubIndex = subIndex;
					}
				}
			}
			else
			{
				auto indexData = op.indexData;
				auto indexBuf = indexData->indexBuffer;
				bool use32bitindexes = indexBuf->getType() == Ogre::HardwareIndexBuffer::IT_32BIT;
				auto pLongIndex = static_cast<Ogre::uint32*>( indexBuf->lock(Ogre::HardwareBuffer::HBL_READ_ONLY) );
				auto pShort = reinterpret_cast<Ogre::uint16*>( pLongIndex );

				switch ( op.operationType )
				{
				case Ogre::RenderOperation::OT_TRIANGLE_LIST:
				{
					assert(( indexData->indexCount % 3 ) == 0);

					auto trianglesCount = indexData->indexCount / 3;
					for ( unsigned triIndex = 0; triIndex < trianglesCount; ++triIndex )
					{
						Ogre::Vector3 v1, v2, v3, n1, n2, n3;
						{
							auto index1 = static_cast<Ogre::uint32>( use32bitindexes ? pLongIndex[triIndex * 3 + 0] : pShort[triIndex * 3 + 0] );
							auto pVertex1 = pVertex + ( vertexBuf->getVertexSize() ) * index1;
							posElem->baseVertexPointerToElement(pVertex1, &pReal);
							v1.x = pReal[0];
							v1.y = pReal[1];
							v1.z = pReal[2];
							v1 = transformation * v1;
							if ( normalElem )
							{
								normalElem->baseVertexPointerToElement(pVertex1, &pReal);
								n1.x = pReal[0];
								n1.y = pReal[1];
								n1.z = pReal[2];
								n1 = normalTransformation * n1;
							}

							auto index2 = static_cast<Ogre::uint32>( use32bitindexes ? pLongIndex[triIndex * 3 + 1] : pShort[triIndex * 3 + 1] );
							auto pVertex2 = pVertex + ( vertexBuf->getVertexSize() ) * index2;
							posElem->baseVertexPointerToElement(pVertex2, &pReal);
							v2.x = pReal[0];
							v2.y = pReal[1];
							v2.z = pReal[2];
							v2 = transformation * v2;
							if ( normalElem )
							{
								normalElem->baseVertexPointerToElement(pVertex2, &pReal);
								n2.x = pReal[0];
								n2.y = pReal[1];
								n2.z = pReal[2];
								n2 = normalTransformation * n2;
							}

							auto index3 = static_cast<Ogre::uint32>( use32bitindexes ? pLongIndex[triIndex * 3 + 2] : pShort[triIndex * 3 + 2] );
							auto pVertex3 = pVertex + ( vertexBuf->getVertexSize() ) * index3;
							posElem->baseVertexPointerToElement(pVertex3, &pReal);
							v3.x = pReal[0];
							v3.y = pReal[1];
							v3.z = pReal[2];
							v3 = transformation * v3;
							if ( normalElem )
							{
								normalElem->baseVertexPointerToElement(pVertex3, &pReal);
								n3.x = pReal[0];
								n3.y = pReal[1];
								n3.z = pReal[2];
								n3 = normalTransformation * n3;
							}
						}

						std::pair<bool, Ogre::Real> hit;
						if ( normalElem )
						{
							hit = Ogre::Math::intersects(ray, v1, v2, v3, ( n1 + n2 + n3 ) / 3, true, false);
						}
						else
						{
							hit = Ogre::Math::intersects(ray, v1, v2, v3, true, false);
						}

						if ( hit.first )
						{
							if ( !foundHit || curMeshQueryInfo->Distance > hit.second )
							{
								foundHit = true;

								curMeshQueryInfo->InteractionPoint = ray * hit.second;
								curMeshQueryInfo->Distance = hit.second;
								curMeshQueryInfo->SubIndex = subIndex;
								curMeshQueryInfo->Triangle[0] = v1;
								curMeshQueryInfo->Triangle[1] = v2;
								curMeshQueryInfo->Triangle[2] = v3;
							}
						}
					}
				}
				break;
				case Ogre::RenderOperation::OT_TRIANGLE_FAN:
				{
					auto trianglesCount = indexData->indexCount - 2;
					for ( unsigned triIndex = 0; triIndex < trianglesCount; ++triIndex )
					{
						Ogre::Vector3 v1, v2, v3;
						{
							auto index1 = static_cast<Ogre::uint32>( use32bitindexes ? pLongIndex[0] : pShort[0] );
							auto pVertex1 = pVertex + ( vertexBuf->getVertexSize() ) * index1;
							posElem->baseVertexPointerToElement(pVertex1, &pReal);
							v1.x = pReal[0];
							v1.y = pReal[1];
							v1.z = pReal[2];
							v1 = transformation * v1;

							auto index2 = static_cast<Ogre::uint32>( use32bitindexes ? pLongIndex[triIndex + 1] : pShort[triIndex + 1] );
							auto pVertex2 = pVertex + ( vertexBuf->getVertexSize() ) * index2;
							posElem->baseVertexPointerToElement(pVertex2, &pReal);
							v2.x = pReal[0];
							v2.y = pReal[1];
							v2.z = pReal[2];
							v2 = transformation * v2;

							auto index3 = static_cast<Ogre::uint32>( use32bitindexes ? pLongIndex[triIndex + 2] : pShort[triIndex + 2] );
							auto pVertex3 = pVertex + ( vertexBuf->getVertexSize() ) * index3;
							posElem->baseVertexPointerToElement(pVertex3, &pReal);
							v3.x = pReal[0];
							v3.y = pReal[1];
							v3.z = pReal[2];
							v3 = transformation * v3;
						}

						auto hit = Ogre::Math::intersects(ray, v1, v2, v3, true, false);
						if ( hit.first )
						{
							if ( !foundHit || curMeshQueryInfo->Distance > hit.second )
							{
								foundHit = true;

								curMeshQueryInfo->InteractionPoint = ray * hit.second;
								curMeshQueryInfo->Distance = hit.second;
								curMeshQueryInfo->SubIndex = subIndex;
								curMeshQueryInfo->Triangle[0] = v1;
								curMeshQueryInfo->Triangle[1] = v2;
								curMeshQueryInfo->Triangle[2] = v3;
							}
						}
					}
				}
				break;
				case Ogre::RenderOperation::OT_TRIANGLE_STRIP:
				{
					auto trianglesCount = indexData->indexCount - 2;
					for ( unsigned triIndex = 0; triIndex < trianglesCount; ++triIndex )
					{
						Ogre::Vector3 v1, v2, v3;
						{
							auto index1 = static_cast<Ogre::uint32>( use32bitindexes ? pLongIndex[triIndex + 0] : pShort[triIndex + 0] );
							auto pVertex1 = pVertex + ( vertexBuf->getVertexSize() ) * index1;
							posElem->baseVertexPointerToElement(pVertex1, &pReal);
							v1.x = pReal[0];
							v1.y = pReal[1];
							v1.z = pReal[2];
							v1 = transformation * v1;

							auto index2 = static_cast<Ogre::uint32>( use32bitindexes ? pLongIndex[triIndex + 1] : pShort[triIndex + 1] );
							auto pVertex2 = pVertex + ( vertexBuf->getVertexSize() ) * index2;
							posElem->baseVertexPointerToElement(pVertex2, &pReal);
							v2.x = pReal[0];
							v2.y = pReal[1];
							v2.z = pReal[2];
							v2 = transformation * v2;

							auto index3 = static_cast<Ogre::uint32>( use32bitindexes ? pLongIndex[triIndex + 2] : pShort[triIndex + 2] );
							auto pVertex3 = pVertex + ( vertexBuf->getVertexSize() ) * index3;
							posElem->baseVertexPointerToElement(pVertex3, &pReal);
							v3.x = pReal[0];
							v3.y = pReal[1];
							v3.z = pReal[2];
							v3 = transformation * v3;
						}

						auto hit = Ogre::Math::intersects(ray, v1, v2, v3, true, false);
						if ( hit.first )
						{
							if ( !foundHit || curMeshQueryInfo->Distance > hit.second )
							{
								foundHit = true;

								curMeshQueryInfo->InteractionPoint = ray * hit.second;
								curMeshQueryInfo->Distance = hit.second;
								curMeshQueryInfo->SubIndex = subIndex;
								curMeshQueryInfo->Triangle[0] = v1;
								curMeshQueryInfo->Triangle[1] = v2;
								curMeshQueryInfo->Triangle[2] = v3;
							}
						}
					}
				}
				break;
				default:continue;
				}

				indexBuf->unlock();
			}
			
			vertexBuf->unlock();

			if ( foundHit )
			{
				ret.push_back(curMeshQueryInfo);
			}

			++subIndex;
		}
	}

	return ret;
}

Ogre::Ray QueryUtil::GetRayFromCamera(const Ogre::Camera* camera, const OIS::MouseState& mouseState, const Ogre::RenderTarget* rt)
{
	auto sx = static_cast<float>(mouseState.X.rel) / rt->getWidth();
	auto sy = static_cast<float>(mouseState.Y.rel) / rt->getHeight();
	return GetRayFromCamera(camera, {sx, sy});
}

Ogre::Ray QueryUtil::GetRayFromCamera(const Ogre::Camera* camera, const Ogre::Vector2& screenRelPos)
{
	auto ray = camera->getCameraToViewportRay(screenRelPos.x, screenRelPos.y);

	return ray;
}