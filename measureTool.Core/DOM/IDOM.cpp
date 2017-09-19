#include "IDOM.h"

#include "Command/CommandMgr.h"
#include "Command/EditNameCMD.h"

#include "Render/IFrameListener.h"
#include "FrameEvent/ToolBarEvent.h"
#include "FrameEvent/ControlEvent.h"

#include "Util/StringUtil.h"

#include "proto/YZMMeasureItem.pb.h"

#include "OgreVector3.h"

#include <bitset>
#include <chrono>

#include <iostream>

using	DocumentWPtr = std::weak_ptr<Document>;

class	IDOM::Imp
{
public:

	using	IROMWPtr = std::weak_ptr<IROM>;

public:

	DocumentWPtr			Document_;
	IROMWPtr				ROM_;
	SListener				Listener_;
	std::bitset<EDS_COUNT>	State_;
	uint32_t				Index_{};
	uint32_t				LastEditTime_{};
	float					Value_{};
	std::wstring			Name_;
	PntList					PntList_;
	bool					OsgPoint_;
	bool					Dirty_;
	bool					IsSaved_;
};


IDOM::IDOM(const DocumentSPtr& doc, uint32_t index) :ImpUPtr_(std::make_unique<Imp>())
{
	auto& imp_ = *ImpUPtr_;

	imp_.Document_ = doc;
	imp_.Index_ = index;
	imp_.OsgPoint_ = false;
	imp_.Dirty_ = false;
	imp_.IsSaved_ = false;
}

IDOM::~IDOM()
{
	auto& imp_ = *ImpUPtr_;
}

IDOM::SListener& IDOM::GetListener()
{
	auto& imp_ = *ImpUPtr_;

	return imp_.Listener_;
}

DocumentSPtr IDOM::GetDocument() const
{
	auto& imp_ = *ImpUPtr_;

	return imp_.Document_.lock();
}

void IDOM::SetROM(const IROMSPtr& rom)
{
	auto& imp_ = *ImpUPtr_;

	imp_.ROM_ = rom;
}

IROMSPtr IDOM::GetROM() const
{
	auto& imp_ = *ImpUPtr_;

	return imp_.ROM_.lock();
}

void IDOM::SetName(const std::wstring& name, bool needUndo)
{
	auto& imp_ = *ImpUPtr_;

	imp_.Listener_.OnChangeUIValue(name, imp_.Value_, imp_.LastEditTime_);

	if ( needUndo )
	{
		auto cmd = std::make_shared<EditNameCMD>(shared_from_this(), imp_.Name_, name);
		CommandMgr::GetInstance().PushCommand(cmd);
	}

	imp_.Name_ = name;
}

const std::wstring& IDOM::GetName() const
{
	auto& imp_ = *ImpUPtr_;

	return imp_.Name_;
}

float IDOM::GetValue() const
{
	auto& imp_ = *ImpUPtr_;

	return imp_.Value_;
}

void IDOM::SetValue(float val)
{
	auto& imp_ = *ImpUPtr_;

	imp_.Value_ = val;
}

void IDOM::UpdateEditTime()
{
	auto& imp_ = *ImpUPtr_;

	if (imp_.Dirty_)
	{
		auto now = std::chrono::system_clock::now();
		imp_.LastEditTime_ = static_cast<uint32_t>(std::chrono::duration_cast<std::chrono::seconds>(now.time_since_epoch()).count());
		//imp_.Dirty_ = false;
	}
}

uint32_t IDOM::GetLastEditTime() const
{
	auto& imp_ = *ImpUPtr_;

	return imp_.LastEditTime_;
}

bool IDOM::TestState(EDOMState state)
{
	auto& imp_ = *ImpUPtr_;

	return imp_.State_.test(state);
}

void IDOM::SetState(EDOMState state, bool val)
{
	auto& imp_ = *ImpUPtr_;

	if ( imp_.State_.test(state) == val )
	{
		return;
	}

	imp_.Listener_.OnChangeState(state, val);

	imp_.State_.set(state, val);
}

uint32_t IDOM::GetIndex() const
{
	auto& imp_ = *ImpUPtr_;

	return imp_.Index_;
}

const IDOM::PntList& IDOM::GetPntList() const
{
	auto& imp_ = *ImpUPtr_;

	return imp_.PntList_;
}

void IDOM::UpdateDOM()
{
	auto& imp_ = *ImpUPtr_;
	OnUpdate(imp_.Value_, imp_.PntList_);
	std::cout << "OnUpdate" << std::endl;
	//if (imp_.Dirty_)
	//{
	//	auto now = std::chrono::system_clock::now();
	//	imp_.LastEditTime_ = static_cast<uint32_t>(std::chrono::duration_cast<std::chrono::seconds>(now.time_since_epoch()).count());
	//	imp_.Dirty_ = false;
	//}
	if ( TestState(EDS_New) && !TestState(EDS_Editable) )
	{
		SetState(EDS_Editable, true);
	}

	imp_.Listener_.OnChangeUIValue(imp_.Name_, imp_.Value_, imp_.LastEditTime_);

	std::cout << "UpdateDOM" << std::endl;
}

void IDOM::DeSerialize(const yzm::MeasureItem& dominfo)
{
	auto& imp_ = *ImpUPtr_;

	imp_.Index_ = dominfo.dom_index();
	imp_.Name_ = StringUtil::UTF8ToUTF16(dominfo.name());
	imp_.Value_ = dominfo.value();
	imp_.LastEditTime_ = dominfo.edit_time();
	imp_.PntList_.clear();
	imp_.PntList_.reserve(dominfo.point_list_size());
	auto type = dominfo.auto_measure_type();

	for ( auto& curPnt : dominfo.point_list() )
	{
		if (type)
			imp_.PntList_.emplace_back(curPnt.x(), curPnt.z(), -curPnt.y());
		else
			imp_.PntList_.emplace_back(curPnt.x(), curPnt.y(), curPnt.z());
	}

	if (!type)
	{
		//if (dominfo.dom_type() == IDOM::EDT_CurveLength || dominfo.dom_type() == IDOM::EDT_ConvexDimension)
		//	OnDeserialize(imp_.PntList_);
		//else
			OnDeserialize(dominfo.dom_extension());
	}
	else
	{
		if (imp_.PntList_.size() == 0)
			return;
		OnDeserialize(imp_.PntList_);
	}

	UpdateDOM();
}

void IDOM::DeSerialize(const PntList& pList)
{
	auto& imp_ = *ImpUPtr_;

	imp_.PntList_.clear();
	for (auto cur : pList)
	{
		imp_.PntList_.emplace_back(cur);
	}

	OnDeserialize(imp_.PntList_);

	UpdateDOM();
}

void IDOM::Serialize(yzm::MeasureItem& buf) const
{
	auto& imp_ = *ImpUPtr_;
	
	buf.set_dom_type(GetType());
	buf.set_dom_index(imp_.Index_);
	buf.set_name(StringUtil::UTF16ToUTF8(imp_.Name_));
	buf.set_value(imp_.Value_);
	buf.set_edit_time(imp_.LastEditTime_);
	buf.mutable_point_list()->Clear();
	buf.mutable_point_list()->Reserve(static_cast<int>(imp_.PntList_.size()));
	for ( auto& curPnt : imp_.PntList_ )
	{
		auto curAdd = buf.add_point_list();
		curAdd->set_x(curPnt.x);
		curAdd->set_y(curPnt.y);
		curAdd->set_z(curPnt.z);
	}

	OnSerialize(*(buf.mutable_dom_extension()));
}

void IDOM::SetOsgPoint(bool flag)
{
	auto& imp_ = *ImpUPtr_;
	imp_.OsgPoint_ = flag;
}

void IDOM::Dirty(const bool flag)
{
	auto& imp_ = *ImpUPtr_;
	imp_.Dirty_ = flag;
}

bool IDOM::isDirty()
{
	auto& imp_ = *ImpUPtr_;
	return imp_.Dirty_;
}

void IDOM::SetSaved(const bool flag)
{
	auto& imp_ = *ImpUPtr_;
	imp_.IsSaved_ = flag;
}

bool IDOM::isSaved()
{
	auto& imp_ = *ImpUPtr_;
	return imp_.IsSaved_;
}