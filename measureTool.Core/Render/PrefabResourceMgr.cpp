#include "PrefabResourceMgr.h"

#include "Ogre.h"

#include <map>

class	PrefabResourceMgr::Imp
{
public:

	std::map<EPrefabMaterial, std::string>	PrefabMaterialList_;
	std::map<EPrefabMesh, std::string>	PrefabMeshList_;

public:

	Imp()
	{
		PrefabMaterialList_.emplace(EPMAT_Curve, "measureTool/Curve");
		PrefabMaterialList_.emplace(EPMAT_Point, "measureTool/Point");
		PrefabMaterialList_.emplace(EPMAT_Manipulator_CircleFace, "measureTool/Manipulator/CircleFace");
		PrefabMaterialList_.emplace(EPMAT_Manipulator_Torus, "measureTool/Manipulator/Torus");
		PrefabMaterialList_.emplace(EPMAT_Manipulator_Arrow, "measureTool/Manipulator/Arrow");
		PrefabMaterialList_.emplace(EPMAT_Aux_DynamicFace, "measureTool/AuxFace");
		PrefabMaterialList_.emplace(EPMAT_Aux_Curve, "measureTool/AuxCurve");
		PrefabMaterialList_.emplace(EPMAT_DebugPointSet, "measureTool/SelectedColor");
		PrefabMaterialList_.emplace(EPMAT_Aux_X, "jt_lambert2SG");
		PrefabMaterialList_.emplace(EPMAT_Aux_Y, "jt_lambert3SG");
		PrefabMaterialList_.emplace(EPMAT_Aux_Z, "jt_lambert4SG");


		PrefabMeshList_.emplace(EPM_UnitLine, "line.mesh");
		PrefabMeshList_.emplace(EPM_UnitSphere, "sphere.mesh");
		PrefabMeshList_.emplace(EPM_UnitCircle, "circle.mesh");
		PrefabMeshList_.emplace(EPM_UnitTorus, "torus.mesh");
		PrefabMeshList_.emplace(EPM_AuxX, "aux_x.mesh");
		PrefabMeshList_.emplace(EPM_AuxY, "aux_y.mesh");
		PrefabMeshList_.emplace(EPM_AuxZ, "aux_z.mesh");
		//PrefabMeshList_.emplace(EPM_AuxXYZ, "xyz.mesh");
	}
};

PrefabResourceMgr::PrefabResourceMgr() :ImpUPtr_(std::make_unique<Imp>())
{

}

PrefabResourceMgr::~PrefabResourceMgr()
{

}

PrefabResourceMgr& PrefabResourceMgr::GetInstance()
{
	static PrefabResourceMgr sIns;

	return sIns;
}

Ogre::MeshPtr PrefabResourceMgr::GetMesh(EPrefabMesh prefabName)
{
	auto& imp_ = *ImpUPtr_;

	auto meshName = imp_.PrefabMeshList_.at(prefabName);
	auto mesh = Ogre::MeshManager::getSingleton().prepare(meshName, "General").staticCast<Ogre::Mesh>();

	return mesh;
}

Ogre::MaterialPtr PrefabResourceMgr::GetMateiral(EPrefabMaterial prefabName)
{
	auto& imp_ = *ImpUPtr_;

	auto matName = imp_.PrefabMaterialList_.at(prefabName);
	auto mat = Ogre::MaterialManager::getSingleton().getByName(matName, "General");

	return mat;
}
