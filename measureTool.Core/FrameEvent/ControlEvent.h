#pragma once

#include "FrameEvent.h"

#include "DOM/IDOMFwd.h"
#include "ROM/IROMFwd.h"
#include "ROM/DocumentROMFwd.h"

class	SFE_Finish : public TFrameEvent<SFE_Finish>
{
public:

	bool	Success{ false };
};

class	SFE_Selection : public TFrameEvent<SFE_Selection>
{
public:

	IDOMSPtr	SelectedDOM;
};

class	SFE_DataDirtyChange : public TFrameEvent<SFE_DataDirtyChange>
{
public:

	bool dirty;
};