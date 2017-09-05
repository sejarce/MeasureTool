#pragma once

enum _EQueryMask
{
	_EQM_HeightLine,
	_EQM_Line,
	_EQM_SurfaceCurve,
	_EQM_ConvexCurve,
	_EQM_SplineCurve,
	_EQM_Point,
	_EQM_COUNT
};

//��ײ�������
enum EQueryFlag
{
	EQF_HeightLine		= 1 << _EQM_HeightLine,
	EQF_Line			= 1 << _EQM_Line,
	EQF_SurfaceCurve	= 1 << _EQM_SurfaceCurve,
	EQF_ConvexCurve		= 1 << _EQM_ConvexCurve,
	EQF_SplineCurve		= 1 << _EQM_SplineCurve,
	EQF_Point			= 1 << _EQM_Point,

	EQF_ALL_Curve = EQF_HeightLine | EQF_Line | EQF_SurfaceCurve | EQF_ConvexCurve | EQF_SplineCurve,
	EQF_ALL = EQF_ALL_Curve | EQF_Point
};