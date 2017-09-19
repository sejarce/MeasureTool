#include "Document.h"

#include "HeightDOM.h"
#include "LineLengthDOM.h"
#include "DimensionDOM.h"
#include "CurveLengthDOM.h"

#include "ROM/DocumentROM.h"

#include "Command/CommandMgr.h"
#include "Command/CreateCMD.h"
#include "Command/DeleteCMD.h"

#include "Util/StringUtil.h"
#include "Util/MeshUtil.h"

#include "proto/YZMFile.pb.h"

#include "openpose/autoOpenPose.h"

#include "Ogre.h"

#include "Util/StringUtil.h"
#include "Util/PCLOctree.h"
#include "Util/MathUtil.h"

#include <array>

#include <boost/format.hpp>
#include <boost/filesystem.hpp>
#include <boost/filesystem/path.hpp>

class	Document::Imp
{
public:

	//using	DocumentROMWPtr = std::weak_ptr<DocumentROM>;

public:

	boost::filesystem::path		FilePath_;
	std::string					ImportBuf;
	SListener					Listener_;
	std::vector<IDOMSPtr>		DOMList_;
	DocumentROMSPtr				DocumentROM_;
	uint32_t					DocIndex_{};
	yzm::PointCloud				PointCloud_;

	CLOUDREAM::CloudColorNormal::Ptr Cloud_{};
	std::map<int, Ogre::Vector3> FusionJoints_;	

	Ogre::Vector3				CrothPoint_{};
	Ogre::Vector3				LeftPoint_{};
	Ogre::Vector3				RightPoint_{};

public:

	IDOMSPtr	CreateItem(DocumentSPtr& doc, IDOM::EDOMType itemType, uint32_t index)
	{
		switch ( itemType )
		{
		case IDOM::EDT_Height:
		{
			return std::make_shared<HeightDOM>(doc, index);
		}
		break;
		case IDOM::EDT_LineLength:
		{
			return std::make_shared<LineLengthDOM>(doc, index);
		}
		break;
		case IDOM::EDT_CurveLength:
		{
			return std::make_shared<CurveLengthDOM>(doc, index);
		}
		break;
		case IDOM::EDT_Dimension:
		{
			return std::make_shared<DimensionDOM>(doc, index, itemType);
		}
		break;
		case IDOM::EDT_ConvexDimension:
		{
			return std::make_shared<DimensionDOM>(doc, index, itemType);
		}
		break;
		case IDOM::EDT_COUNT:
		assert(0);
		break;
		default:
		break;
		}

		return{};
	}

	bool CreateItem(IDOMSPtr item, PCLOctreeSPtr Octree, const std::vector<Ogre::Vector3>& SampleDirList, const std::wstring& name, const Ogre::Vector3& pos, const Ogre::Vector3& normal)
	{
		if (item->GetType() == IDOM::EDT_ConvexDimension || item->GetType() == IDOM::EDT_Dimension)
		{
			//求dom的pointList
			Ogre::Vector3 pnt = pos;
			Ogre::Plane pln(normal, pnt);
			const float resolution = 9e-3;
			static const auto sampleEps = 5e-3;
			auto sqSampleEps = sampleEps * sampleEps;

			auto plnList = Octree->PlaneQuery(pln, pnt, resolution);
			if (plnList.empty())
				return false;

			Ogre::Vector3 centroid = Ogre::Vector3::ZERO;
			for (auto& cur : plnList)
			{
				centroid += cur;
			}
			centroid /= plnList.size();

			std::vector<Ogre::Vector3> frontList, backList;

			std::vector<Ogre::Vector3> tmpFrontList, tmpBackList;
			Ogre::Vector3 tmpFrontCentroid = Ogre::Vector3::ZERO;
			Ogre::Vector3 tmpBackCentroid = Ogre::Vector3::ZERO;

			for (auto& curSampleDir : SampleDirList)
			{
				tmpFrontList.clear();
				tmpBackList.clear();
				tmpFrontCentroid = Ogre::Vector3::ZERO;
				tmpBackCentroid = Ogre::Vector3::ZERO;

				auto curAdjSampleDir = Ogre::Vector3::UNIT_Y.getRotationTo(pln.normal) * curSampleDir;

				for (auto& curPlnPnt : plnList)
				{
					auto projPnt = MathUtil::ProjectPointOnLine(curPlnPnt, centroid, curAdjSampleDir);
					auto sd = curPlnPnt.squaredDistance(projPnt);
					if (sd > sqSampleEps)
					{
						continue;
					}

					auto curDir = projPnt - centroid;
					if (curDir.dotProduct(curAdjSampleDir) > 0)
					{
						tmpFrontList.push_back(projPnt);
						tmpFrontCentroid += projPnt;
					}
					else
					{
						tmpBackList.push_back(projPnt);
						tmpBackCentroid += projPnt;
					}
				}

				if (!tmpFrontList.empty())
				{
					//取中间点
					//tmpFrontCentroid /= tmpFrontList.size();
					//取外层点（最远点）
					float maxSd = 0.0;
					for (auto curr : tmpFrontList)
					{
						auto sd = curr.squaredDistance(centroid);
						if (sd > maxSd)
						{
							maxSd = sd;
							tmpFrontCentroid = curr;
						}
					}
					frontList.push_back(tmpFrontCentroid);
				}

				if (!tmpBackList.empty())
				{
					//取中间点
					//tmpBackCentroid /= tmpBackList.size();
					//取外层点（最远点）
					float maxSd = 0.0;
					for (auto curr : tmpBackList)
					{
						auto sd = curr.squaredDistance(centroid);
						if (sd > maxSd)
						{
							maxSd = sd;
							tmpBackCentroid = curr;
						}
					}
					backList.push_back(tmpBackCentroid);
				}
			}

			if (frontList.empty() && backList.empty())
			{
				return false;
			}

			std::vector<Ogre::Vector3> CurQueryList;
			CurQueryList.swap(frontList);
			std::copy(backList.begin(), backList.end(), std::back_inserter(CurQueryList));

			//set item
			item->SetOsgPoint(false);
			item->SetName(name, false);
			item->DeSerialize(CurQueryList);
			item->SetSaved();
			
			DOMList_.push_back(item);
			Listener_.OnCreateDOM(item);
		}

		return true;
	}
};

Document::Document() :ImpUPtr_(std::make_unique<Imp>())
{
	ImpUPtr_->CrothPoint_ = { 0.0, 0.0, 0.0 };
	ImpUPtr_->LeftPoint_ = { 0.0, 0.0, 0.0 };
	ImpUPtr_->RightPoint_ = { 0.0, 0.0, 0.0 };
}

Document::~Document()
{
	auto& imp_ = *ImpUPtr_;

	imp_.PointCloud_.Clear();

	for ( auto& cur : imp_.DOMList_ )
	{
		imp_.Listener_.OnRemoveDOM(cur);
	}
}

bool Document::ImportFile(const std::wstring& filePath)
{
	auto& imp_ = *ImpUPtr_;

	imp_.FusionJoints_.clear();
	std::map<int, Ogre::Vector3>().swap(imp_.FusionJoints_);

	if ( !boost::filesystem::exists(filePath) )
	{
		return false;
	}

	auto fileSize = boost::filesystem::file_size(filePath);
	imp_.ImportBuf.resize(fileSize);
	boost::filesystem::ifstream ifs(filePath, std::ios::binary);
	ifs.read(&imp_.ImportBuf[0], imp_.ImportBuf.size());

	return MeshUtil::ImportMesh(imp_.PointCloud_, imp_.ImportBuf);
}

bool Document::ImportPlyFile(const std::wstring& filePath)
{
	auto& imp_ = *ImpUPtr_;
	//imp_.Cloud_ = boost::make_shared<CLOUDREAM::CloudColorNormal>();

	if (!AutoOpenPose::GetInstance().loadFile(StringUtil::UTF16ToUTF8(filePath)))
		return false;

	if (!AutoOpenPose::GetInstance().researchPoseFeature())
		return false;

	imp_.FusionJoints_.clear();
	std::map<int, Ogre::Vector3>().swap(imp_.FusionJoints_);
	auto fusionJoints = AutoOpenPose::GetInstance().getFusionJoints();
	for (auto cur : fusionJoints)
	{
		imp_.FusionJoints_[cur.first] = { cur.second.x,cur.second.y,cur.second.z };
	}

	imp_.FilePath_ = filePath;
	imp_.Cloud_ = AutoOpenPose::GetInstance().getCloudColorNormal();

	//填充imp_.PointCloud_
	imp_.PointCloud_.Clear();
	imp_.PointCloud_.set_point_type(yzm::PointCloud_EPntType_Ply);

	auto pointList = imp_.PointCloud_.mutable_point_list();

	for (auto cur : *imp_.Cloud_)
	{
		auto newVer = imp_.PointCloud_.add_point_list();

		newVer->set_x(cur.x);
		newVer->set_y(cur.y);
		newVer->set_z(cur.z);

		newVer->set_r(cur.b/255.f);
		newVer->set_g(cur.g/255.f);
		newVer->set_b(cur.r/255.f);
	}

	return true;
}

bool Document::OpenFile(const std::wstring& filePath)
{
	auto& imp_ = *ImpUPtr_;

	imp_.FusionJoints_.clear();
	std::map<int, Ogre::Vector3>().swap(imp_.FusionJoints_);

	boost::filesystem::ifstream ifs(filePath, std::ios::in | std::ios::binary);

	yzm::YZMFile fileInfo;
	auto ret = fileInfo.ParseFromIstream(&ifs);
	if ( !ret )
	{
		//return false;
	}

	imp_.FilePath_ = filePath;

	imp_.PointCloud_.Swap(fileInfo.mutable_point_cloud());
	auto bOsgPoint = imp_.PointCloud_.point_type() == yzm::PointCloud_EPntType_OSG;

	auto& docInfo = *( fileInfo.mutable_document() );
	imp_.DocIndex_ = docInfo.doc_index();

	for ( auto& curItem : *docInfo.mutable_item_list() )
	{
		if ( curItem.auto_measure_type() != yzm::MeasureItem_EAutoMeasureType_None )
		{//自动测量数据
			IDOM::EDOMType itemType;
			std::wstring name;

			switch (curItem.auto_measure_type())
			{
			case yzm::MeasureItem_EAutoMeasureType_BodyHeight:
			{
				itemType = IDOM::EDT_Height;
				name = L"身高";
			}
			break;
			case yzm::MeasureItem_EAutoMeasureType_ChestWidth:
			{
				itemType = IDOM::EDT_CurveLength;
				name = L"胸宽";
			}
			break;
			case yzm::MeasureItem_EAutoMeasureType_BackWidth:
			{
				itemType = IDOM::EDT_CurveLength;
				name = L"背宽";
			}
			break;
			case yzm::MeasureItem_EAutoMeasureType_ShoulderWidth:
			{
				itemType = IDOM::EDT_CurveLength;
				name = L"肩宽";
			}
			break;
			case yzm::MeasureItem_EAutoMeasureType_ChestDimension:
			{
				itemType = IDOM::EDT_ConvexDimension;
				name = L"胸围";
			}
			break;
			case yzm::MeasureItem_EAutoMeasureType_WaistDimension:
			{
				itemType = IDOM::EDT_ConvexDimension;
				name = L"腰围";
			}
			break;
			case yzm::MeasureItem_EAutoMeasureType_WristDimension:
			{
				itemType = IDOM::EDT_ConvexDimension;
				name = L"腕围";
			}
			break;
			case yzm::MeasureItem_EAutoMeasureType_KneeDimension:
			{
				itemType = IDOM::EDT_ConvexDimension;
				name = L"膝围";
			}
			break;
			case yzm::MeasureItem_EAutoMeasureType_HipDimension:
			{
				itemType = IDOM::EDT_ConvexDimension;
				name = L"臀围";
			}
			break;
			case yzm::MeasureItem_EAutoMeasureType_ArmDimension:
			{
				itemType = IDOM::EDT_ConvexDimension;
				name = L"臂围";
			}
			break;
			case yzm::MeasureItem_EAutoMeasureType_ArmLength:
			{
				itemType = IDOM::EDT_LineLength;
				name = L"臂长";
			}
			break;
			case yzm::MeasureItem_EAutoMeasureType_BPDistance:
			{
				itemType = IDOM::EDT_LineLength;
				name = L"BP距";
			}
			break;
			case yzm::MeasureItem_EAutoMeasureType_LegDimension:
			{
				itemType = IDOM::EDT_ConvexDimension;
				name = L"腿围";
			}
			break;
			case yzm::MeasureItem_EAutoMeasureType_LegLength:
			{
				itemType = IDOM::EDT_Height;
				name = L"腿长";
			}
			break;
			default:
			assert(0);
			break;
			}

			curItem.set_dom_type(itemType);
			curItem.set_dom_index(imp_.DocIndex_++);
			curItem.set_name(StringUtil::UTF16ToUTF8(name));
		}
		
		auto item = imp_.CreateItem(shared_from_this(), static_cast<IDOM::EDOMType>( curItem.dom_type() ), curItem.dom_index());

		if (curItem.value() < 1e-3)
			continue;
		item->SetOsgPoint(bOsgPoint);
		item->DeSerialize(curItem);
		item->SetSaved();
		imp_.DOMList_.push_back(item);
		imp_.Listener_.OnCreateDOM(item);
	}

	//特征点
	auto crothPoint = *docInfo.mutable_croth_point();
	auto leftPoint = *docInfo.mutable_left_armpit_point();
	auto rightPoint = *docInfo.mutable_right_armpit_point();
	if (bOsgPoint){
		imp_.CrothPoint_ = { crothPoint.x(), crothPoint.z(), -crothPoint.y() };
		imp_.LeftPoint_ = { leftPoint.x(), leftPoint.z(), -leftPoint.y() };
		imp_.RightPoint_ = { rightPoint.x(), rightPoint.z(), -rightPoint.y() };
	}
	else
	{
		imp_.CrothPoint_ = { crothPoint.x(), crothPoint.y(), crothPoint.z() };
		imp_.LeftPoint_ = { leftPoint.x(), leftPoint.y(), leftPoint.z() };
		imp_.RightPoint_ = { rightPoint.x(), rightPoint.y(), rightPoint.z() };
	}
	return true;
}

const std::wstring& Document::GetFilePath() const
{
	auto& imp_ = *ImpUPtr_;

	return imp_.FilePath_.wstring();
}

void Document::SaveToFile(const std::wstring& filePath)
{
	auto& imp_ = *ImpUPtr_;

	Save();

	yzm::YZMFile msg;
	auto& doc = *( msg.mutable_document() );
	*(msg.mutable_point_cloud()) = (imp_.PointCloud_);
	doc.set_doc_index(imp_.DocIndex_);

	for ( auto& curDOM : imp_.DOMList_ )
	{
		auto curMsg = doc.add_item_list();
		curMsg->set_auto_measure_type(yzm::MeasureItem_EAutoMeasureType_None);
		curDOM->Serialize(*curMsg);
	}
	auto crothpoint = msg.mutable_document()->mutable_croth_point();
	auto leftpoint = msg.mutable_document()->mutable_left_armpit_point();
	auto rightpoint = msg.mutable_document()->mutable_right_armpit_point();
	if (imp_.PointCloud_.point_type())
	{
		crothpoint->set_x(getCrothPoint().x);
		crothpoint->set_y(getCrothPoint().y);
		crothpoint->set_z(getCrothPoint().z);

		leftpoint->set_x(getLeftPoint().x);
		leftpoint->set_y(getLeftPoint().y);
		leftpoint->set_z(getLeftPoint().z);

		rightpoint->set_x(getRightPoint().x);
		rightpoint->set_y(getRightPoint().y);
		rightpoint->set_z(getRightPoint().z);
	}
	else
	{
		crothpoint->set_x(getCrothPoint().x);
		crothpoint->set_y(-getCrothPoint().z);
		crothpoint->set_z(getCrothPoint().y);

		leftpoint->set_x(getLeftPoint().x);
		leftpoint->set_y(-getLeftPoint().z);
		leftpoint->set_z(getLeftPoint().y);

		rightpoint->set_x(getRightPoint().x);
		rightpoint->set_y(-getRightPoint().z);
		rightpoint->set_z(getRightPoint().y);
	}

	boost::filesystem::ofstream ofs(filePath, std::ios::trunc | std::ios::binary);

	msg.SerializePartialToOstream(&ofs);

	imp_.FilePath_ = filePath;
}

void Document::ExportCSVFile(const std::wstring& filePath)
{
	auto& imp_ = *ImpUPtr_;

	std::wstringstream ss;

	ss << L"数据项,值" << std::endl;

	boost::wformat fmt(L"%s,%.1fcm\n");

	for (auto& curDom : imp_.DOMList_ )
	{
		if ( curDom->TestState(IDOM::EDS_Delete) )
		{
			continue;
		}

		fmt.clear();

		fmt % curDom->GetName() % ( curDom->GetValue() * 100 );

		ss << fmt.str();
	}

	boost::filesystem::ofstream ofs(filePath, std::ios::trunc | std::ios::binary);
	std::array<uint8_t, 3> bom;
	bom[0] = static_cast<uint8_t>( 0xef );
	bom[1] = static_cast<uint8_t>( 0xbb );
	bom[2] = static_cast<uint8_t>( 0xbf );
	ofs.write(reinterpret_cast<char*>( bom.data() ), bom.size());
	auto str = StringUtil::UTF16ToUTF8(ss.str());
	ofs << str;
}

const yzm::PointCloud& Document::GetPointCloud() const
{
	auto& imp_ = *ImpUPtr_;

	return imp_.PointCloud_;
}

Document::SListener& Document::GetListener()
{
	auto& imp_ = *ImpUPtr_;

	return imp_.Listener_;
}

IDOMSPtr Document::CreateDOM(IDOM::EDOMType itemType)
{
	auto& imp_ = *ImpUPtr_;

	auto item = imp_.CreateItem(shared_from_this(), itemType, imp_.DocIndex_++);
	item->SetState(IDOM::EDS_New, true);
	
	static std::map<IDOM::EDOMType, std::wstring> nameMap;
	if ( nameMap.empty() )
	{
		nameMap[IDOM::EDT_Height] = L"高度";
		nameMap[IDOM::EDT_LineLength] = L"长度";
		nameMap[IDOM::EDT_CurveLength] = L"表面长度";
		nameMap[IDOM::EDT_Dimension] = L"表面围度";
		nameMap[IDOM::EDT_ConvexDimension] = L"围度";
	}

	item->SetName(nameMap[itemType] + std::to_wstring(item->GetIndex()), false);
	//item->SetSaved(false);

	imp_.DOMList_.push_back(item);
	imp_.Listener_.OnCreateDOM(item);

	auto cmd = std::make_shared<CreateCMD>(item);
	CommandMgr::GetInstance().PushCommand(cmd);

	return item;
}

void Document::RemoveDOM(const IDOMSPtr& item)
{
	auto& imp_ = *ImpUPtr_;

	auto itor = std::find(imp_.DOMList_.begin(), imp_.DOMList_.end(), item);
	assert(itor != imp_.DOMList_.end());

	//此处会crash，暂时注释掉，启用undo，redo时需从新考虑
	//if ( item->TestState(IDOM::EDS_New) && !item->TestState(IDOM::EDS_Editable) )
	//{
	//	imp_.DOMList_.erase(itor, imp_.DOMList_.end());
	//	CommandMgr::GetInstance().RemoveRelativeCommand(item);
	//}
	//else
	{
		item->SetState(IDOM::EDS_Delete, true);

		auto cmd = std::make_shared<DeleteCMD>(item);
		CommandMgr::GetInstance().PushCommand(cmd);
	}
}

const IDOMList& Document::GetDOMList() const
{
	auto& imp_ = *ImpUPtr_;

	return imp_.DOMList_;
}

void Document::SetDocumentROM(const DocumentROMSPtr& docROM)
{
	auto& imp_ = *ImpUPtr_;

	imp_.DocumentROM_ = docROM;
}

DocumentROMSPtr Document::GetDocumentROM() const
{
	auto& imp_ = *ImpUPtr_;

	return imp_.DocumentROM_/*.lock()*/;
}

Ogre::Vector3 Document::getCrothPoint() const
{
	auto& imp_ = *ImpUPtr_;
	return imp_.CrothPoint_;
}

Ogre::Vector3 Document::getLeftPoint() const
{
	auto& imp_ = *ImpUPtr_;
	return imp_.LeftPoint_;
}

Ogre::Vector3 Document::getRightPoint() const
{
	auto& imp_ = *ImpUPtr_;
	return imp_.RightPoint_;
}

void Document::setCrothPoint(const Ogre::Vector3& vec)
{
	auto& imp_ = *ImpUPtr_;
	imp_.CrothPoint_ = vec;
}

void Document::setLeftPoint(const Ogre::Vector3& vec)
{
	auto& imp_ = *ImpUPtr_;
	imp_.LeftPoint_ = vec;
}

void Document::setRightPoint(const Ogre::Vector3& vec)
{
	auto& imp_ = *ImpUPtr_;
	imp_.RightPoint_ = vec;
}

std::map<int, Ogre::Vector3> Document::getFeaturePoints() const
{
	auto &imp_ = *ImpUPtr_;
	return imp_.FusionJoints_;
}

bool Document::estimateData()
{
	auto& imp_ = *ImpUPtr_;

	if (imp_.FusionJoints_.empty() || !imp_.DocumentROM_.get())
		return false;

	auto Octree = imp_.DocumentROM_->GetOctree();
	if (!Octree)
		return false;

	//绕y轴360°的向量
	std::vector<Ogre::Vector3> SampleDirList;
	auto step = 3;
	auto haflCount = 180 / step;
	for (auto deg = 0; deg < haflCount; ++deg)
	{
		auto curDir = deg * step;
		Ogre::Quaternion qua;
		qua.FromAngleAxis(Ogre::Radian(Ogre::Degree(deg * 3)), Ogre::Vector3::UNIT_Y);
		auto dir = qua * Ogre::Vector3::UNIT_X;

		SampleDirList.push_back(dir);
	}

	imp_.DocIndex_ = 0;

	//根据特征点，添加自动测量部分数据：
	//1.	大臂围
	{
		//创建dom
		auto item = imp_.CreateItem(shared_from_this(), IDOM::EDT_Dimension, imp_.DocIndex_++);

		auto pos = imp_.FusionJoints_[2]*0.5 + imp_.FusionJoints_[3]*0.5;
		auto normal = (imp_.FusionJoints_[2] - imp_.FusionJoints_[3]).normalisedCopy();
		if (!imp_.CreateItem(item, Octree, SampleDirList, L"大臂围", pos, normal))
			return false;
	}
	//2.	腕围
	{
		//创建dom
		auto item = imp_.CreateItem(shared_from_this(), IDOM::EDT_Dimension, imp_.DocIndex_++);

		auto pos = imp_.FusionJoints_[4];
		auto normal = (imp_.FusionJoints_[3] - imp_.FusionJoints_[4]).normalisedCopy();
		if (!imp_.CreateItem(item, Octree, SampleDirList, L"腕围", pos, normal))
			return false;
	}
	//3.	肩宽
	//4.	背宽
	//5.	胸围
	//6.	中腰围
	//7.	腰围
	//8.	穿裆长
	//9.	臀围
	{
		//创建dom
		auto item = imp_.CreateItem(shared_from_this(), IDOM::EDT_Dimension, imp_.DocIndex_++);

		auto pos = imp_.FusionJoints_[11];
		auto normal = Ogre::Vector3::UNIT_Y;
		if (!imp_.CreateItem(item, Octree, SampleDirList, L"臀围", pos, normal))
			return false;
	}
	//10.	大腿围
	{
		//创建dom
		auto item = imp_.CreateItem(shared_from_this(), IDOM::EDT_Dimension, imp_.DocIndex_++);

		auto pos = imp_.FusionJoints_[11] * 0.5 + imp_.FusionJoints_[12] * 0.5;
		auto normal = (imp_.FusionJoints_[11] - imp_.FusionJoints_[12]).normalisedCopy();
		if (!imp_.CreateItem(item, Octree, SampleDirList, L"大腿围", pos, normal))
			return false;
	}
	//11.	膝围
	{
		//创建dom
		auto item = imp_.CreateItem(shared_from_this(), IDOM::EDT_Dimension, imp_.DocIndex_++);

		auto pos = imp_.FusionJoints_[12];
		auto normal = (imp_.FusionJoints_[11] - imp_.FusionJoints_[13]).normalisedCopy();
		if (!imp_.CreateItem(item, Octree, SampleDirList, L"膝围", pos, normal))
			return false;
	}
	//12.	腿长
	//13.	臂长
	//14.	身高	

	return true;
}

void Document::Save()
{
	auto& imp_ = *ImpUPtr_;

	auto itor = std::remove_if(imp_.DOMList_.begin(), imp_.DOMList_.end(), [](const IDOMSPtr& dom)
	{
		if ( dom->TestState(IDOM::EDS_Delete) )
		{
			return true;
		}

		dom->SetState(IDOM::EDS_New, false);
		dom->SetState(IDOM::EDS_Editable, false);

		return false;
	});

	imp_.DOMList_.erase(itor, imp_.DOMList_.end());

	CommandMgr::GetInstance().Clear();
}

void Document::Redo()
{
	auto& imp_ = *ImpUPtr_;

	CommandMgr::GetInstance().Redo();
}

void Document::Undo()
{
	auto& imp_ = *ImpUPtr_;

	CommandMgr::GetInstance().Undo();
}
