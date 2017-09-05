#pragma once

#include "FrameEvent.h"
#include "DOM/Document.h"

class SFE_ResetCamera : public TFrameEvent<SFE_ResetCamera>
{

};

class SFE_ShowAllData : public TFrameEvent<SFE_ShowAllData>
{

};

class SFE_EditFeaturePoints : public TFrameEvent<SFE_EditFeaturePoints>
{
public:
	enum EState
	{
		E_Hide = 0,
		E_Show		
	};

	EState conState;
public:
	//DocumentSPtr DOM;
	std::weak_ptr<Document> DOM;
};

class SFE_FloatBarState : public TFrameEvent<SFE_FloatBarState>
{
public://TOUI
	enum EToToolBarState
	{
		ETBS_None = 0x0,
		ETBS_ConAllSee = 0x1,
		ETBS_ConResetCamera = 0x2,
		ETBS_ConFeaturePoint = 0x4
	};

	EToToolBarState	Contate;
};