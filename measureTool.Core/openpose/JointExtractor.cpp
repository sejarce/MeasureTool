#include "JointExtractor.h"
#include <pcl/recognition/linemod/line_rgbd.h>

namespace CLOUDREAM
{
	inline cv::Point3f transformCvPoint(const Eigen::Matrix4f& T, cv::Point3f pt)
	{
		Eigen::Vector4f ptHomo = T * Eigen::Vector4f(pt.x, pt.y, pt.z, 1);
		return cv::Point3f(ptHomo.x(), ptHomo.y(), ptHomo.z());
	};

	cv::Mat renderImageFromCloud(pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr colorCloud,
		Eigen::Matrix4f cameraPose, cv::Size imgSize, float f,
		cv::Mat& depthImg = cv::Mat(), cv::Mat& mask = cv::Mat());


	/************************************************************************/
	/* JointExtractorRender
	/************************************************************************/
	JointExtractorRender::JointExtractorRender(std::string model_folder,
		cv::Size img_size, float f) : m_imgSize(img_size), m_focal(f)
	{
		op::PoseModel model_pose = op::PoseModel::COCO_18;

		// use default parameters
		int gpu_id = 0;
		double scale_gap = 0.3;
		int scale_number = 1;

		const auto imgSize = op::Point<int>(img_size.width, img_size.height);
		const auto netInputSize = op::Point<int>(656, 368);
		const auto netOutputSize = netInputSize;

		// construct method for formating input image
		m_cvMatToOpInput = new op::CvMatToOpInput(netInputSize, scale_number, scale_gap);

		// construct network and init
		m_caffePoseExtractor = new op::PoseExtractorCaffe(netInputSize, netOutputSize, imgSize, scale_number, model_pose,
			model_folder, gpu_id);
		m_caffePoseExtractor->initializationOnThread();

		// Allocation
		m_bodyCloud = pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr(new pcl::PointCloud<pcl::PointXYZRGBNormal>());

	}

	bool JointExtractorRender::loadStandardBodyCloud(std::string plyFile)
	{
		// Check exsitence
		if (pcl::io::loadPLYFile(plyFile, *m_bodyCloud) < 0)
		{
			std::cerr << "JointExtractorRender:loadStandardBodyCloud, ply file path not exist, " << plyFile << std::endl;
			return false;
		}

		return setStandardBodyCloud(m_bodyCloud);
	}

	bool JointExtractorRender::setStandardBodyCloud(CloudColorNormal::Ptr bodyCloud)
	{
		m_bodyCloud = bodyCloud;

		// We hope to process under camera coordinate system(local)
		m_T_src_to_local = Eigen::Matrix4f::Identity();
		m_T_src_to_local(1, 1) = -1;
		m_T_src_to_local(2, 2) = -1;

		pcl::transformPointCloudWithNormals(*m_bodyCloud, *m_bodyCloud, m_T_src_to_local);

		renderFrontAndBackImage();


		if (m_frontColorImg.empty() || m_backColorImg.empty())
		{
			std::cout << "Failed to render front and back image!\n";
			return false;
		}
		else
			return true;
	}

	void JointExtractorRender::renderFrontAndBackImage()
	{
		// 1. compute camera translate along z axis
		float max_float = std::numeric_limits<float>::max();
		float min_float = std::numeric_limits<float>::min();

		pcl::PointXYZ minPt(max_float, max_float, max_float);
		pcl::PointXYZ maxPt(min_float, min_float, min_float);
		for (auto pt : m_bodyCloud->points)
		{
			if (pt.x > maxPt.x)
				maxPt.x = pt.x;
			if (pt.y > maxPt.y)
				maxPt.y = pt.y;
			if (pt.z > maxPt.z)
				maxPt.z = pt.z;

			if (pt.x < minPt.x)
				minPt.x = pt.x;
			if (pt.y < minPt.y)
				minPt.y = pt.y;
			if (pt.z < minPt.z)
				minPt.z = pt.z;
		}
		
		std::vector<float> minTranslation;
		minTranslation.push_back(2.0f * m_focal * fabs(minPt.x) / m_imgSize.width);
		minTranslation.push_back(2.0f * m_focal * fabs(minPt.y) / m_imgSize.height);
		minTranslation.push_back(2.0f * m_focal * fabs(maxPt.x) / m_imgSize.height);
		minTranslation.push_back(2.0f * m_focal * fabs(maxPt.y) / m_imgSize.height);


		float translation = *std::max_element(minTranslation.begin(), minTranslation.end()) * 1.1;

		// 2. construct front and back camera poses
		Eigen::Matrix4f frontCameraPose = Eigen::Matrix4f::Identity();
		frontCameraPose(2, 3) = translation; // meter

		Eigen::Matrix4f backCameraPose = Eigen::Matrix4f::Identity();
		backCameraPose(0, 0) = -1;
		backCameraPose(2, 2) = -1;
		backCameraPose(2, 3) = translation;

		m_frontColorImg = renderImageFromCloud(m_bodyCloud, frontCameraPose, m_imgSize, m_focal, m_frontDepthImg, m_frontRenderMask);
		m_backColorImg = renderImageFromCloud(m_bodyCloud, backCameraPose, m_imgSize, m_focal, m_backDepthImg, m_backRenderMask);
	}


	int JointExtractorRender::extractJoints()
	{

		// Convert image to Array format
		op::Array<float> netInputArrayFront, netInputArrayBack;
		std::vector<float> scaleRatiosFront, scaleRatiosBack;
		std::tie(netInputArrayFront, scaleRatiosFront) = m_cvMatToOpInput->format(m_frontColorImg);
		std::tie(netInputArrayBack, scaleRatiosBack) = m_cvMatToOpInput->format(m_backColorImg);

		// Run forward network computation
		m_caffePoseExtractor->forwardPass(netInputArrayFront, { m_imgSize.width, m_imgSize.height }, scaleRatiosFront);
		op::Array<float> poseKeypointsFront = m_caffePoseExtractor->getPoseKeypoints();
		if (poseKeypointsFront.getSize(0) != 1)
			return -1;

		m_caffePoseExtractor->forwardPass(netInputArrayBack, { m_imgSize.width, m_imgSize.height }, scaleRatiosBack);
		op::Array<float> poseKeypointsBack = m_caffePoseExtractor->getPoseKeypoints();
		if (poseKeypointsBack.getSize(0) != 1)
			return -1;

		// check result and format to 2D points
		float x, y, score;

		m_fusionJoint3DPoints.clear();
		// only focus on interested joints
		for (int joint_id : m_interestedJointSet)
		{
			int i = joint_id;
			const auto baseIndex = 3 * i;

			x = poseKeypointsFront[baseIndex];
			y = poseKeypointsFront[baseIndex + 1];
			score = poseKeypointsFront[baseIndex + 2];	// confidence, 0-1

			std::vector<float> frontPt;
			if (score != 0)
			{
				m_frontJoint2DPoints.insert(std::pair<int, cv::Point3f>(i, cv::Point3f(x, y, score)));
				
				cv::Vec3f val(0,0,0);
				if (m_frontRenderMask.at<uchar>(y, x) == 0)
				{
					// if location pixel has no depth value, search on its neighbors
					std::vector<cv::Vec3f> neighborVals;
					if (m_frontRenderMask.at<uchar>(y - 1, x) == 255)
						neighborVals.push_back(m_frontDepthImg.at<cv::Vec3f>(y - 1, x));
					if (m_frontRenderMask.at<uchar>(y + 1, x) == 255)
						neighborVals.push_back(m_frontDepthImg.at<cv::Vec3f>(y + 1, x));
					if (m_frontRenderMask.at<uchar>(y, x-1) == 255)
						neighborVals.push_back(m_frontDepthImg.at<cv::Vec3f>(y, x-1));
					if (m_frontRenderMask.at<uchar>(y, x+1) == 255)
						neighborVals.push_back(m_frontDepthImg.at<cv::Vec3f>(y, x+1));

					if (neighborVals.empty())
					{
						std::cerr << "OpenPose Location Error, there is no depth at JOINT-" << i << "!\n";
						continue;
					}	
					else
					{
						for (auto v : neighborVals)
							val += v;
						val = val / (int)neighborVals.size();
					}
				}
				else
					val = m_frontDepthImg.at<cv::Vec3f>(y, x);
				
				frontPt = { val(0), val(1), val(2) };
				m_frontJoint3DPoints.insert(std::pair<int, cv::Point3f>(i, cv::Point3f(val(0), val(1), val(2))));
			}



			x = poseKeypointsBack[baseIndex];
			y = poseKeypointsBack[baseIndex + 1];
			score = poseKeypointsBack[baseIndex + 2];
			std::vector<float> backPt;
			if (score != 0)
			{
				m_backJoint2DPoints.insert(std::pair<int, cv::Point3f>(i, cv::Point3f(x, y, score)));
				cv::Vec3f val(0, 0, 0);
				if (m_backRenderMask.at<uchar>(y, x) == 0)
				{
					std::vector<cv::Vec3f> neighborVals;
					if (m_backRenderMask.at<uchar>(y - 1, x) == 255)
						neighborVals.push_back(m_backDepthImg.at<cv::Vec3f>(y - 1, x));
					if (m_backRenderMask.at<uchar>(y + 1, x) == 255)
						neighborVals.push_back(m_backDepthImg.at<cv::Vec3f>(y + 1, x));
					if (m_backRenderMask.at<uchar>(y, x - 1) == 255)
						neighborVals.push_back(m_backDepthImg.at<cv::Vec3f>(y, x - 1));
					if (m_backRenderMask.at<uchar>(y, x + 1) == 255)
						neighborVals.push_back(m_backDepthImg.at<cv::Vec3f>(y, x + 1));

					if (neighborVals.empty())
					{
						std::cerr << "OpenPose Location Error, there is no depth at Joint-" << i<<  "!\n";
						return -3;
					}
					else
					{
						for (auto v : neighborVals)
							val += v;
						val = val / (int)neighborVals.size();
					}
				}
				else
					val = m_backDepthImg.at<cv::Vec3f>(y, x);


				backPt = { val(0), val(1), val(2) };
				m_backJoint3DPoints.insert(std::pair<int, cv::Point3f>(i, cv::Point3f(val(0), val(1), val(2))));
			}

			// Fusion 
			if (frontPt.empty() || backPt.empty())
				continue;
			else
			{
				cv::Point3f fusionPt;
				fusionPt.x = (frontPt[0] + backPt[0]) / 2.0f;
				fusionPt.y = (frontPt[1] + backPt[1]) / 2.0f;
				fusionPt.z = (frontPt[2] + backPt[2]) / 2.0f;

				m_fusionJoint3DPoints.insert(std::pair<int, cv::Point3f>(i, fusionPt));
			}
		}

		return m_fusionJoint3DPoints.size();
	}
 

	std::map<int, cv::Point3f> JointExtractorRender::getFusionJoints() const
	{
		std::map<int, cv::Point3f> joints;
		std::map<int, cv::Point3f>::const_iterator it = m_fusionJoint3DPoints.begin();
		for (; it != m_fusionJoint3DPoints.end(); ++it)
		{
			int jointId = it->first;
			cv::Point3f ptLocal = it->second;
			cv::Point3f ptSrc = transformCvPoint(m_T_src_to_local.inverse(), ptLocal);
			joints.insert(std::pair<int, cv::Point3f>(jointId, ptSrc));
		}

		return joints;
	};

	std::map<int, cv::Point3f> JointExtractorRender::getFrontJoints() const
	{
		std::map<int, cv::Point3f> joints;
		std::map<int, cv::Point3f>::const_iterator it = m_frontJoint3DPoints.begin();
		for (; it != m_frontJoint3DPoints.end(); ++it)
		{
			int jointId = it->first;
			cv::Point3f ptLocal = it->second;
			cv::Point3f ptSrc = transformCvPoint(m_T_src_to_local.inverse(), ptLocal);
			joints.insert(std::pair<int, cv::Point3f>(jointId, ptSrc));
		}

		return joints;
	};

	std::map<int, cv::Point3f> JointExtractorRender::getBackJoints() const
	{
		std::map<int, cv::Point3f> joints;
		std::map<int, cv::Point3f>::const_iterator it = m_backJoint3DPoints.begin();
		for (; it != m_backJoint3DPoints.end(); ++it)
		{
			int jointId = it->first;
			cv::Point3f ptLocal = it->second;
			cv::Point3f ptSrc = transformCvPoint(m_T_src_to_local.inverse(), ptLocal);
			joints.insert(std::pair<int, cv::Point3f>(jointId, ptSrc));
		}

		return joints;
	};


	cv::Mat renderImageFromCloud(CloudColorNormal::Ptr colorCloud, Eigen::Matrix4f cameraPose, 
		cv::Size imgSize, float f, cv::Mat& depthImg, cv::Mat& mask)
	{
		typedef Eigen::Matrix<float, 7, 1> Vector7f;

		mask = cv::Mat(imgSize, CV_8UC1, cv::Scalar(0, 0, 0));
		depthImg = cv::Mat(imgSize, CV_32FC3);

		cv::Mat image(imgSize, CV_8UC3, cv::Scalar(0, 0, 0));

		std::map<int, Vector7f> surface_points_rgbd; //map: pixel index ===> r,g,b,x,y,z,d

		int w = imgSize.width;
		int h = imgSize.height;

		float cx = (imgSize.width - 1) / 2.0f;
		float cy = (imgSize.height - 1) / 2.0f;

		Eigen::Matrix3f K;
		K << f, 0, cx, 0, f, cy, 0, 0, 1;

		Eigen::Matrix3Xf P = K * cameraPose.topRows(3);

		Eigen::Vector3f position = cameraPose.inverse().col(3).topRows(3);

		for (auto pt : colorCloud->points)
		{

			Eigen::Vector4f pt3D_world(pt.x, pt.y, pt.z, 1);
			Eigen::Vector3f pt2D_homo = P * pt3D_world;
			float d = sqrt(pow(pt.x - position(0), 2) + pow(pt.y - position(1), 2) + pow(pt.z - position(2), 2));

			cv::Point2f pt2D_float(pt2D_homo.x() / pt2D_homo.z(), pt2D_homo.y() / pt2D_homo.z());
			cv::Point pt2D_pixel = cv::Point((int)std::round(pt2D_float.x), (int)std::round(pt2D_float.y));

			if (pt2D_pixel.x >= w || pt2D_pixel.x < 0 || pt2D_pixel.y >= h || pt2D_pixel.y < 0)
				continue;

			// Require point normal direction are visible under this camera pose
			Eigen::Vector3f surface_normal(pt.normal_x, pt.normal_y, pt.normal_z);
			Eigen::Vector3f point_view = pt3D_world.topRows(3) - position;
			if (point_view.dot(surface_normal) > 0)
				continue;

			Vector7f rgbd;
			rgbd << pt.r, pt.g, pt.b, pt.x, pt.y, pt.z, d;

			int index = pt2D_pixel.x + w * pt2D_pixel.y;

			// If there is no point on pixel, or depth of stored point is larger than current depth
			// fill in with current point
			if (surface_points_rgbd.find(index) == surface_points_rgbd.end())
				surface_points_rgbd.insert(std::pair<int, Vector7f>(index, rgbd));
			else if (surface_points_rgbd.at(index)(6) > rgbd(6))
				surface_points_rgbd[index] = rgbd;

		}

		std::map<int, Vector7f>::iterator it = surface_points_rgbd.begin();
		for (; it != surface_points_rgbd.end(); ++it)
		{
			int index = it->first;
			int row = index / w;
			int col = index % w;


			cv::Vec3b color(it->second(2), it->second(1), it->second(0));
			cv::Vec3f point(it->second(3), it->second(4), it->second(5));
			image.at<cv::Vec3b>(row, col) = color;
			mask.at<uchar>(row, col) = 255;
			depthImg.at<cv::Vec3f>(row, col) = point;
		}

		return image;
	}














}

