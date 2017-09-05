#include "CurveLengthDOM.h"

#include "Util/MathUtil.h"

#include "Ogre.h"

#include "TopoDS_Edge.hxx"
#include "GProp_GProps.hxx"
#include "GeomAPI_PointsToBSpline.hxx"
#include "Geom_BSplineCurve.hxx"
#include "BrepTools.hxx"
#include "BRep_Builder.hxx"
#include "BRepGProp.hxx"
#include "BRepBuilderAPI_MakeEdge.hxx"
#include "BRepBuilderAPI_MakeVertex.hxx"
#include <sstream>

class	CurveLengthDOM::Imp
{
public:
	Pnt3D	StartPos_;
	Pnt3D	EndPos_;
	Handle(Geom_BSplineCurve)	FittingCurve_;
	TopoDS_Edge		Edge_;
};

CurveLengthDOM::CurveLengthDOM(const DocumentSPtr& doc, uint32_t index) :IDOM(doc, index)
, ImpUPtr_(std::make_unique<Imp>())
{
	auto& imp_ = *ImpUPtr_;
}

CurveLengthDOM::~CurveLengthDOM()
{

}

IDOM::EDOMType CurveLengthDOM::GetType() const
{
	return IDOM::EDT_CurveLength;
}

const CurveLengthDOM::Pnt3D& CurveLengthDOM::GetStartPos() const
{
	auto& imp_ = *ImpUPtr_;

	return imp_.StartPos_;
}

const CurveLengthDOM::Pnt3D& CurveLengthDOM::GetEndPos() const
{
	auto& imp_ = *ImpUPtr_;

	return imp_.EndPos_;
}

TopoDS_Edge CurveLengthDOM::GetEdge() const
{
	auto& imp_ = *ImpUPtr_;

	return imp_.Edge_;
}

void CurveLengthDOM::UpdateCurve(const CurveLengthDOM::Pnt3D& startpos, const CurveLengthDOM::Pnt3D& endpos, const PntList& pnlList)
{
	auto& imp_ = *ImpUPtr_;

	imp_.StartPos_ = startpos;
	imp_.EndPos_ = endpos;

	//TopoDS_Builder builder;
	//TopoDS_Compound comp;
	//builder.MakeCompound(comp);

	TColgp_Array1OfPnt pntList(1, pnlList.size());
	{
		auto index = 0;
		for ( auto& cur : pnlList )
		{
			pntList.SetValue(index + 1, { cur.x, cur.y, cur.z });

			//builder.Add(comp, BRepBuilderAPI_MakeVertex({ cur.x, cur.y, cur.z }).Vertex());

			++index;
		}
	}
	//BRepTools::Write(comp, "old.brep");


	GeomAPI_PointsToBSpline approx(pntList, 3, 3, GeomAbs_G1, 5e-3);

	

	imp_.FittingCurve_ = approx.Curve();
	imp_.Edge_ = BRepBuilderAPI_MakeEdge(imp_.FittingCurve_).Edge();
	//////////////////////////////////////////////////////////////////////////
	//BRepTools::Write(imp_.Edge_, "new.brep");
	//////////////////////////////////////////////////////////////////////////
	UpdateDOM();
}

void CurveLengthDOM::OnUpdate(float& val, PntList& pntList)
{
	auto& imp_ = *ImpUPtr_;

	GProp_GProps sys;
	BRepGProp::LinearProperties(imp_.Edge_, sys);
	val = static_cast<float>( sys.Mass() );

	auto buildCount = 100;
	pntList.clear();
	pntList.reserve(buildCount);

	auto step = 1.f / (buildCount-1);
	gp_Pnt pnt;
	for ( auto index = 0; index < buildCount; ++index )
	{
		imp_.FittingCurve_->D0(step*index, pnt);
		pntList.emplace_back(static_cast<float>( pnt.X() ), static_cast<float>( pnt.Y() ), static_cast<float>( pnt.Z() ));
	}
}

void CurveLengthDOM::OnDeserialize(const std::string& buf)
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

	gp_Pnt pnt;
	imp_.FittingCurve_->D0(0.0, pnt);
	imp_.StartPos_.second = { static_cast<float>(pnt.X()), static_cast<float>(pnt.Y()), static_cast<float>(pnt.Z()) };
	imp_.FittingCurve_->D0(1.0, pnt);
	imp_.EndPos_.second = { static_cast<float>(pnt.X()), static_cast<float>(pnt.Y()), static_cast<float>(pnt.Z()) };

	imp_.StartPos_.first = -1;
	imp_.EndPos_.first = -1;
}

void CurveLengthDOM::OnDeserialize(const PntList& pList)
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

	imp_.StartPos_.second = pList.front();
	imp_.EndPos_.second = pList.back();

	imp_.StartPos_.first = -1;
	imp_.EndPos_.first = -1;
}

void CurveLengthDOM::OnSerialize(std::string& buf) const
{
	auto& imp_ = *ImpUPtr_;

	std::stringstream ss;

	BRepTools::Write(imp_.Edge_, ss);

	buf = ss.str();
}