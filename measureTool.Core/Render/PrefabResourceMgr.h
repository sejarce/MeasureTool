#pragma once

#include "OgrePrerequisites.h"

#include <memory>

class	PrefabResourceMgr
{
	class	Imp;
	std::unique_ptr<Imp>	ImpUPtr_;

public:

	PrefabResourceMgr();

	~PrefabResourceMgr();

public:

	static	PrefabResourceMgr&	GetInstance();

public:

	enum EPrefabMesh
	{
		EPM_Grid,
		EPM_UnitLine,
		EPM_UnitSphere,
		EPM_UnitCircle,
		EPM_UnitTorus,
		EPM_AuxX,
		EPM_AuxY,
		EPM_AuxZ,
		EPM_AuxXYZ
	};

	Ogre::MeshPtr	GetMesh(EPrefabMesh prefabName);

public:

	enum EPrefabMaterial
	{
		EPMAT_Grid,
		EPMAT_Curve,
		EPMAT_Point,
		EPMAT_Manipulator_CircleFace,
		EPMAT_Manipulator_Torus,
		EPMAT_Manipulator_Arrow,
		EPMAT_Aux_DynamicFace,
		EPMAT_Aux_Curve,
		EPMAT_DebugPointSet,
		EPMAT_Aux_X,
		EPMAT_Aux_Y,
		EPMAT_Aux_Z
	};

	Ogre::MaterialPtr	GetMateiral(EPrefabMaterial prefabName);

public:

	enum EPrefabColor
	{
		EPC_Clr
	};
};