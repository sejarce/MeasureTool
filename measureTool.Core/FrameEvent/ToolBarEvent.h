#pragma once

#include "FrameEvent.h"

#include "DOM/IDOMFwd.h"
#include "ROM/IROMFwd.h"
#include "ROM/DocumentROMFwd.h"

class	SFE_OpenFile : public TFrameEvent<SFE_OpenFile>
{
public://TOUI

	enum ELoadingState
	{
		ELS_SucessLoadMesh,
		ELS_FailLoadMesh,
		ELS_FinishBuildInfo
	};

	ELoadingState	LoadingState;

public://TO3D

	std::wstring	FilePath;
};

class	SFE_OpenDocROM : public TFrameEvent<SFE_OpenDocROM>
{
public:

	DocumentROMSPtr	DocROM;
};

class	SFE_SDC : public TFrameEvent<SFE_SDC>
{
public://TOUI

	enum EToUIState
	{
		EUS_ConfiromSave,
		EUS_ConfiromDelete,
		EUS_ConfiromCancel,
		EUS_ConfiromDirty
	};

	EToUIState	ConfirmUIState;

public://TO3D
	enum ETo3DState
	{
		ES_Save,
		ES_Delete,
		ES_Cancel
	};

	ETo3DState	To3DState;
};

class	SFE_EditItem : public TFrameEvent<SFE_EditItem>
{
public:

	enum EItemType
	{
		EIT_Height,
		EIT_LineLength,
		EIT_CurveLength,
		EIT_Dimension,
		EIT_ConvexDimension
	};

	EItemType	Type;

public://TO3D

	enum EStatus
	{
		ES_CreateMode,
		ES_EditMode,
		ES_COUNT
	};

	IDOMWPtr	DOM;

	EStatus Status;
};

class SFE_ToolBarState : public TFrameEvent<SFE_ToolBarState>
{
public://TOUI
	enum EToToolBarState
	{
		ETBS_None = 0x0,
		ETBS_ConSee = 0x1,
		ETBS_ConHeight = 0x2,
		ETBS_ConLineLength = 0x4,
		ETBS_ConCurveLength = 0x8,
		ETBS_ConDimension = 0x10,
		ETBS_ConvexDimension = 0x20 
	};

	EToToolBarState	Contate;
};
