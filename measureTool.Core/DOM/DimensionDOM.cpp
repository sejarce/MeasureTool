#include "DimensionDOM.h"

#include "Document.h"

#include "Util/MathUtil.h"

#include "Ogre.h"

#include "pcl/surface/convex_hull.h"

#include "TopoDS_Edge.hxx"
#include "GProp_GProps.hxx"
#include "GeomAPI_PointsToBSpline.hxx"
#include "GeomAPI_Interpolate.hxx"
#include "Geom_BSplineCurve.hxx"
#include "BrepTools.hxx"
#include "BRep_Builder.hxx"
#include "BRepGProp.hxx"
#include "BRepBuilderAPI_MakeEdge.hxx"

#include "BRepBuilderAPI_MakeVertex.hxx"
#include "BRepTools.hxx"

#include <sstream>

class	DimensionDOM::Imp
{
public:

	EDOMType		Type_;
	
	Ogre::Vector3	PlnNormal_;
	Ogre::Vector3	PlnPos_;
	Handle(Geom_BSplineCurve)	FittingCurve_;
	TopoDS_Edge		Edge_;

	PntList			PntList_;
	//DocumentSPtr	Doc_;
	std::weak_ptr<Document> Doc_;
};

DimensionDOM::DimensionDOM(const DocumentSPtr& doc, uint32_t index, EDOMType domType) :IDOM(doc, index)
, ImpUPtr_(std::make_unique<Imp>())
{
	auto& imp_ = *ImpUPtr_;

	imp_.Type_ = domType;
	imp_.Doc_ = doc;
}

DimensionDOM::~DimensionDOM()
{
	auto& imp_ = *ImpUPtr_;

	imp_.PntList_.clear();
	PntList().swap(imp_.PntList_);
}

IDOM::EDOMType DimensionDOM::GetType() const
{
	auto& imp_ = *ImpUPtr_;

	return imp_.Type_;
}

const Ogre::Vector3& DimensionDOM::GetPlnNormal() const
{
	auto& imp_ = *ImpUPtr_;

	return imp_.PlnNormal_;
}

const Ogre::Vector3& DimensionDOM::GetPlnPos() const
{
	auto& imp_ = *ImpUPtr_;

	return imp_.PlnPos_;
}

TopoDS_Edge DimensionDOM::GetEdge() const
{
	auto& imp_ = *ImpUPtr_;

	return imp_.Edge_;
}

void DimensionDOM::UpdateCurve(const Ogre::Vector3& plnNormal, const PntList& verList)
{
	auto& imp_ = *ImpUPtr_;

	if (plnNormal.distance(Ogre::Vector3::ZERO) < 1e-3)
	{
		auto siz = verList.size();
		if (siz > 2)
		{			
			auto v1 = verList[siz / 2] - verList[0];
			auto v2 = verList[siz - 1] - verList[0];
			imp_.PlnNormal_ = v1.crossProduct(v2).normalisedCopy();
		}
	}
	else
	{
		imp_.PlnNormal_ = plnNormal;
	}

	//根据特征点分块
	std::vector<Ogre::Vector3> verListCut;
	auto dist = (imp_.Doc_.lock()->getLeftPoint() - imp_.Doc_.lock()->getRightPoint()).length();
	if (dist > 1e-3)
	{
		Ogre::Vector3 centerPos(0.0, 0.0, 0.0);
		for (auto& cur : verList)
		{
			centerPos += cur;
		}
		centerPos /= verList.size();
		auto areaFlag = MathUtil::RegionalismFromFeaturePoints(imp_.Doc_.lock()->getCrothPoint(), imp_.Doc_.lock()->getLeftPoint(), imp_.Doc_.lock()->getRightPoint(), centerPos);
		auto ys = (imp_.Doc_.lock()->getLeftPoint().y + imp_.Doc_.lock()->getRightPoint().y) / 2;
		auto ye = imp_.Doc_.lock()->getCrothPoint().y;

		bool bcull = false;
		if (abs(centerPos.y - ys) < 0.15 || (centerPos.y - ye < 0 && centerPos.y - ye > -0.2))
			bcull = true;
		
		for (auto& cur : verList)
		{
			if (bcull)
			{
				auto aflag = MathUtil::RegionalismFromFeaturePoints(imp_.Doc_.lock()->getCrothPoint(), imp_.Doc_.lock()->getLeftPoint(), imp_.Doc_.lock()->getRightPoint(), cur);
				if (aflag == areaFlag)
				{
					verListCut.emplace_back(cur);
				}
			}
			else
			{
				verListCut.emplace_back(cur);
			}
		}
		
	}
	else
	{
		for (auto& cur : verList)
		{
			verListCut.emplace_back(cur);
		}
	}

	if ( GetType() == IDOM::EDT_Dimension )
	{
		TColgp_Array1OfPnt pntList(1, verListCut.size() + 1);
		{
			//TopoDS_Builder builder;
			//TopoDS_Compound comp;
			//builder.MakeCompound(comp);

			auto index = 0;
			for (auto& cur : verListCut)
			{
				pntList.SetValue(index + 1, { cur.x, cur.y, cur.z });
				//builder.Add(comp, BRepBuilderAPI_MakeVertex({ cur.x, cur.y, cur.z }).Vertex());
				++index;
			}
			pntList.SetValue(index + 1, { verListCut.front().x, verListCut.front().y, verListCut.front().z });
			//builder.Add(comp, BRepBuilderAPI_MakeVertex({ verListCut.front().x, verListCut.front().y, verListCut.front().z }).Vertex());

			//BRepTools::Write(comp, "unconvex.brep");
		}

		GeomAPI_PointsToBSpline approx(pntList, 3, 3, GeomAbs_G1, 2e-3);

		imp_.FittingCurve_ = approx.Curve();
		imp_.Edge_ = BRepBuilderAPI_MakeEdge(imp_.FittingCurve_).Edge();
		//BRepTools::Write(imp_.Edge_, "unconvex2.brep");
	}
	else if (GetType() == IDOM::EDT_ConvexDimension)
	{
		//TopoDS_Builder builder;
		//TopoDS_Compound comp;
		//builder.MakeCompound(comp);

		imp_.PntList_.clear();
		PntList().swap(imp_.PntList_);
		imp_.PntList_.reserve(verListCut.size());
		for (auto cur : verListCut)
		{
			//builder.Add(comp, BRepBuilderAPI_MakeVertex({ cur.x, cur.y, cur.z }).Vertex());
			imp_.PntList_.emplace_back(cur);
		}
		//BRepTools::Write(comp, "unconvex.brep");

		if (!MathUtil::ConvexPolygon(plnNormal, imp_.PntList_))
			return;

		{
		//TopoDS_Builder builder;
		//TopoDS_Compound comp;
		//builder.MakeCompound(comp);
		////Handle(TColgp_HArray1OfPnt) pntList = new TColgp_HArray1OfPnt(1, convexList.size());
		TColgp_Array1OfPnt pntList(1, imp_.PntList_.size()+1);
		{
			auto index = 0;
			for (auto& cur : imp_.PntList_)
			{
				pntList.SetValue(index + 1, gp_Pnt{ cur.x, cur.y, cur.z });
				//builder.Add(comp, BRepBuilderAPI_MakeVertex({ cur.x, cur.y, cur.z }).Vertex());
				++index;
			}
			pntList.SetValue(index + 1, gp_Pnt{ imp_.PntList_.front().x, imp_.PntList_.front().y, imp_.PntList_.front().z });
			//builder.Add(comp, BRepBuilderAPI_MakeVertex({ imp_.PntList_.front().x, imp_.PntList_.front().y, imp_.PntList_.front().z }).Vertex());
		}
		//BRepTools::Write(comp, "convex.brep");

		GeomAPI_PointsToBSpline approx(pntList, 3, 3, GeomAbs_G1, 2e-3);
		////GeomAPI_Interpolate approx(pntList, Standard_True, 2e-4);

		imp_.FittingCurve_ = approx.Curve();
		imp_.Edge_ = BRepBuilderAPI_MakeEdge(imp_.FittingCurve_).Edge();
		}
	}

	UpdateDOM();
}

void DimensionDOM::OnUpdate(float& val, PntList& pntList)
{
	auto& imp_ = *ImpUPtr_;

	GProp_GProps sys;
	BRepGProp::LinearProperties(imp_.Edge_, sys);
	imp_.PlnPos_ = MathUtil::ToOgre(sys.CentreOfMass());
	val = static_cast<float>(sys.Mass());

	if (GetType() == IDOM::EDT_Dimension){
		auto buildCount = 300;
		pntList.clear();
		pntList.reserve(buildCount);

		//TopoDS_Builder builder;
		//TopoDS_Compound comp;
		//builder.MakeCompound(comp);

		auto step = 1.f / (buildCount - 1);
		gp_Pnt pnt;
		for (auto index = 0; index < buildCount; ++index)
		{
			imp_.FittingCurve_->D0(step*index, pnt);
			pntList.emplace_back(static_cast<float>(pnt.X()), static_cast<float>(pnt.Y()), static_cast<float>(pnt.Z()));
			//builder.Add(comp, BRepBuilderAPI_MakeVertex({ pnt.X(), pnt.Y(), pnt.Z() }).Vertex());
		}
		//BRepTools::Write(comp, "FittingCurve.brep");
	}
	else if (GetType() == IDOM::EDT_ConvexDimension)
	{
		if (imp_.PntList_.empty())
		{
			auto buildCount = 300;
			pntList.clear();
			pntList.reserve(buildCount);

			auto step = 1.f / (buildCount - 1);
			gp_Pnt pnt;
			for (auto index = 0; index < buildCount; ++index)
			{
				imp_.FittingCurve_->D0(step*index, pnt);
				pntList.emplace_back(static_cast<float>(pnt.X()), static_cast<float>(pnt.Y()), static_cast<float>(pnt.Z()));
			}

			auto plnNormal = (pntList[0] - pntList[buildCount / 3]).crossProduct(pntList[buildCount / 3] - pntList[buildCount * 2 / 3]);

			if (!MathUtil::ConvexPolygon(plnNormal, pntList))
				return;
		}
		else
		{
			pntList.clear();
			pntList.reserve(imp_.PntList_.size());

			for (auto curr : imp_.PntList_)
			{
				pntList.emplace_back(curr);
			}
		}
	}

	auto dis = 0.f;
	std::adjacent_find(pntList.begin(), pntList.end(), [&](const Ogre::Vector3& v1, const Ogre::Vector3& v2)
	{
		dis += v1.distance(v2);

		return false;
	});

	dis += pntList.front().distance(pntList.back());

	val = dis;
}

void DimensionDOM::OnDeserialize(const std::string& buf)
{
	auto& imp_ = *ImpUPtr_;

	auto bufsize = buf.size();
	std::stringstream ss;
	ss << buf;
	
	BRep_Builder builder;
	BRepTools::Read(imp_.Edge_, ss, builder);

	Standard_Real f, l;
	auto curve = BRep_Tool::Curve(imp_.Edge_, f, l);
	imp_.FittingCurve_ = Handle(Geom_BSplineCurve)::DownCast(curve);
}

void DimensionDOM::OnDeserialize(const PntList& pList)
{
	auto& imp_ = *ImpUPtr_;

	auto verList = pList;

	TColgp_Array1OfPnt pntList(1, verList.size());
	{
		auto index = 0;
		for (auto& cur : verList)
		{
			pntList.SetValue(index + 1, { cur.x, cur.y, cur.z });
			++index;
		}
	}

	GeomAPI_PointsToBSpline approx(pntList, 3, 3, GeomAbs_G1, 2e-3);

	imp_.FittingCurve_ = approx.Curve();
	imp_.Edge_ = BRepBuilderAPI_MakeEdge(imp_.FittingCurve_).Edge();

}

void DimensionDOM::OnSerialize(std::string& buf) const
{
	auto& imp_ = *ImpUPtr_;

	std::stringstream ss;

	BRepTools::Write(imp_.Edge_, ss);

	buf = ss.str();
}
