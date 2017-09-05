#pragma once

#include "OgreRenderQueue.h"

enum ERenderGroupID
{
	ERG_Grid = Ogre::RENDER_QUEUE_WORLD_GEOMETRY_1 + 1,
	ERG_PointCloud = Ogre::RENDER_QUEUE_MAIN,
	ERG_Curve,
	ERG_Curve_Sweep,
	ERG_Curve_Select,
	ERG_Point,
	ERG_Point_Sweep,
	ERG_Point_Select,
	ERG_Line_Begin = 71,
	ERG_Line,
	ERG_Line_Sweep,
	ERG_Line_Select,
	ERG_Aux_Begin = 81,
	ERG_Aux_FittingFace,
	ERG_Manipulator_Begin = 91,
	ERG_Manipulator_Ring,
	ERG_Manipulator_RingFace,
	ERG_Manipulator_Arrow,
	ERG_END
};