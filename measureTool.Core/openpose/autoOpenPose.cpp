#include "autoOpenPose.h"

std::string g_model_dir = "./openpose/models/";

#define SOURCE 10F

class	AutoOpenPose::Imp
{
public:
	CLOUDREAM::JointExtractorRender*	extractor_ { nullptr };
	CLOUDREAM::CloudColorNormal::Ptr	Cloud_{ nullptr };
	std::map<int, cv::Point3f>			FusionJoints_;
};

AutoOpenPose::AutoOpenPose() : ImpUPtr_(new Imp)
{
}

AutoOpenPose::~AutoOpenPose()
{
	auto& imp_ = *ImpUPtr_;

	if (imp_.extractor_)
		delete imp_.extractor_;
}

AutoOpenPose& AutoOpenPose::GetInstance()
{
	static AutoOpenPose sins;
	return  sins;
}

void AutoOpenPose::Init()
{
	auto& imp_ = *ImpUPtr_;

	imp_.extractor_ = new CLOUDREAM::JointExtractorRender(g_model_dir);
	std::cout << "AutoOpenPose init()" << std::endl;
}

bool AutoOpenPose::loadFile(const std::string& filePath)
{
	auto& imp_ = *ImpUPtr_;

	imp_.Cloud_ = boost::make_shared<CLOUDREAM::CloudColorNormal>();
	pcl::io::loadPLYFile(filePath, *imp_.Cloud_);

#ifdef SOURCE == 10F
	// if input data is from 10F scan system
	std::vector<Eigen::Vector3f> basis = {
		Eigen::Vector3f(1,0,0),
		Eigen::Vector3f(0,0,1),
		Eigen::Vector3f(0,-1,0) };
#elif SOURCE == 12F
	// if input data is from 12F scan system
	std::vector<Eigen::Vector3f> basis = {
		Eigen::Vector3f(1,0,0),
		Eigen::Vector3f(0,-1,0),
		Eigen::Vector3f(0,0,-1) };
#endif
	CLOUDREAM::CloudXYZ::Ptr cloud_xyz(new CLOUDREAM::CloudXYZ());
	pcl::copyPointCloud(*imp_.Cloud_, *cloud_xyz);
	Eigen::Matrix4f T;
	CLOUDREAM::downSampling<pcl::PointXYZ>(*cloud_xyz, 0.01);
	if (CLOUDREAM::getTranformToInertialSystem(cloud_xyz, basis, T) >= 0)
	{		
		pcl::transformPointCloudWithNormals(*imp_.Cloud_, *imp_.Cloud_, T);
		CLOUDREAM::computeNormal(imp_.Cloud_);
		CLOUDREAM::reorientCloudNormal(imp_.Cloud_);
		return true;
	}
	else
	{
		std::cerr << "Failed to transform coordinate system!\n";
		return false;
	}
}

bool AutoOpenPose::researchPoseFeature()
{
	auto& imp_ = *ImpUPtr_;
	
	if (!imp_.Cloud_ || !imp_.extractor_)
		return false;

	imp_.extractor_->setInterestedJointSet(CLOUDREAM::stable_body_joints);
	imp_.extractor_->setStandardBodyCloud(imp_.Cloud_);

	if (imp_.extractor_->extractJoints() == CLOUDREAM::stable_body_joints.size())
	{
		imp_.FusionJoints_.clear();
		std::map<int, cv::Point3f>().swap(imp_.FusionJoints_);
		imp_.FusionJoints_ = imp_.extractor_->getFusionJoints();

		for (auto joint_id : CLOUDREAM::stable_body_joints)
			std::cout << "\tjoint id: " << joint_id << "\t" << imp_.FusionJoints_.at(joint_id) << std::endl;
		
		return true;
	}
	else
	{
		std::cout << "\tFailed to extract all stable body joints!\n";
		return false;
	}
}

CLOUDREAM::CloudColorNormal::Ptr AutoOpenPose::getCloudColorNormal() const
{
	auto& imp_ = *ImpUPtr_;
	return imp_.Cloud_;
}

std::map<int, cv::Point3f> AutoOpenPose::getFusionJoints() const
{
	auto& imp_ = *ImpUPtr_;
	return imp_.FusionJoints_;
}
