#pragma once

#include "IDOMFwd.h"
#include "DocumentFwd.h"

#include "ROM/IROMFwd.h"

#include "OgreVector3.h"

#include <memory>
#include <vector>

#include <boost/signals2.hpp>

namespace yzm
{
class	MeasureItem;
}

class	IDOM : public std::enable_shared_from_this<IDOM>
{
	class	Imp;
	std::unique_ptr<Imp>	ImpUPtr_;

public:

	enum EDOMState
	{
		//新建状态
		EDS_New,
		//删除状态
		EDS_Delete,
		//可编辑状态
		EDS_Editable,
		EDS_COUNT
	};

	enum EDOMType
	{
		//高度
		EDT_Height,
		//长度
		EDT_LineLength,
		//表面长度
		EDT_CurveLength,
		//表面围度
		EDT_Dimension,
		//围度
		EDT_ConvexDimension,
		EDT_COUNT
	};

	class	SListener
	{
	public:

		using	OnChangeUIignal			= boost::signals2::signal<void(const std::wstring&, float, uint32_t)>;
		using	OnChangeStateSignal		= boost::signals2::signal<void(EDOMState, bool)>;
		using	OnApplyContentSignal	= boost::signals2::signal<void(IDOMSPtr&)>;

	public:

		OnChangeUIignal			OnChangeUIValue;
		OnChangeStateSignal		OnChangeState;
	};

	using	PntList = std::vector<Ogre::Vector3>;

public:

	IDOM(const DocumentSPtr& doc, uint32_t index);

	~IDOM();

public:

	SListener&		GetListener();

	DocumentSPtr	GetDocument() const;

	void			SetROM(const IROMSPtr& rom);

	IROMSPtr		GetROM() const;

public:

	virtual EDOMType	GetType() const
	{
		return EDT_COUNT;
	}

public:

	void				SetName(const std::wstring& name, bool needUndo);

	const std::wstring&	GetName() const;

	float				GetValue() const;

	void				SetValue(float val);

	void				UpdateEditTime();

	uint32_t			GetLastEditTime() const;

	uint32_t			GetIndex() const;

	const PntList&		GetPntList() const;

	bool				TestState(EDOMState state);

	void				SetState(EDOMState state, bool val);

	//update from rom
	void				UpdateDOM();

	void				DeSerialize(const yzm::MeasureItem& buf);

	void				DeSerialize(const PntList& pList);

	void				Serialize(yzm::MeasureItem& buf) const;

	void				SetOsgPoint(bool flag);

	void				Dirty(const bool flag = true);

	bool				isDirty();

	void				SetSaved(const bool flag = true);

	bool				isSaved();
protected:

	virtual void	OnUpdate(float& val, PntList& pntList)	{}

	virtual	void	OnDeserialize(const std::string& buf) {}

	virtual	void	OnDeserialize(const PntList& pList) {}

	virtual	void	OnSerialize(std::string& buf) const {}
};