#pragma once
#include <memory>

#include "util_xieke.h"
#include "JointExtractor.h"

class AutoOpenPose
{
	class Imp;
	std::unique_ptr<Imp>	ImpUPtr_;

	AutoOpenPose();

public:
	~AutoOpenPose();

public:

	static AutoOpenPose&	GetInstance();

public:
	void Init();

	bool loadFile(const std::string& filePath);

	bool researchPoseFeature();

	CLOUDREAM::CloudColorNormal::Ptr	getCloudColorNormal() const;

	std::map<int, cv::Point3f>			getFusionJoints() const;
};