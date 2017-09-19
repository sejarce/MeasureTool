#pragma once
// OpenPose dependencies First
#include <openpose/core/headers.hpp>
#include <openpose/filestream/headers.hpp>
#include <openpose/gui/headers.hpp>
#include <openpose/pose/headers.hpp>
#include <openpose/utilities/headers.hpp>

#include <set>

#include "util_xieke.h"

#define JOINT_NUM 18	//  Num of joints

namespace CLOUDREAM
{
	// enum type of joints
	enum JOINT {
		NOSE = 0,
		NECK = 1,
		RSHOULDER = 2,
		RELBOW = 3,
		RWRIST = 4,
		LSHOULDER = 5,
		LELBOW = 6,
		LWRIST = 7,
		RHIP = 8,
		RKNEE = 9,
		RANKLE = 10,
		LHIP = 11,
		LKNEE = 12,
		LANKLE = 13,
		REYE = 14,
		LEYE = 15,
		REAR = 16,
		LEAR = 17,
		BACKGROUND = 18
	};

	const std::set<int> stable_body_joints = {NECK, 
		RSHOULDER, RELBOW, RWRIST, LSHOULDER, LELBOW, LWRIST, 
		RHIP, RKNEE, RANKLE, LHIP, LKNEE, LANKLE};

	// Joints extract algorithm based on rendering
	class JointExtractorRender
	{

	public:
		// Constructor
		JointExtractorRender(std::string model_folder/* = "models/"*/,			// pre-trained model folder, require COCO type
			cv::Size img_size = cv::Size(480, 640),								// size of image to render
			float f = 785.0f);													// focal of camera to render

		~JointExtractorRender()
		{
			delete m_cvMatToOpInput;
			delete m_caffePoseExtractor;
		};

		// 0. Set interested joints set, we will only try to extract these joints
		void setInterestedJointSet(const std::set<int>& jointSet) {
			m_interestedJointSet = jointSet;
		};

		// 1. Load/Set body cloud in standard cloudream body coordinate system(x as left, y as up, z as front)
		// Note: we will try to render front and back images followed, return false if failed to rendering.
		bool loadStandardBodyCloud(std::string plyFile);

		bool setStandardBodyCloud(CloudColorNormal::Ptr bodyCloud);


		// 2. Extract joints from render images and fusion to 3D points
		// return num of extracted joints
		int extractJoints();

		// 3. Get joints' 3D points, map key is joint id
		std::map<int, cv::Point3f> getFusionJoints() const;

		std::map<int, cv::Point3f> getFrontJoints() const;

		std::map<int, cv::Point3f> getBackJoints() const;

		// 4. Clear before extracting next body's joints
		void clear() {
			m_bodyCloud->clear();
			m_frontColorImg.release();
			m_backColorImg.release();
			m_frontRenderMask.release();
			m_backRenderMask.release();
			m_frontDepthImg.release();
			m_backDepthImg.release();
			m_frontJoint2DPoints.clear();
			m_backJoint2DPoints.clear();
			m_frontJoint3DPoints.clear();
			m_backJoint3DPoints.clear();
			m_fusionJoint3DPoints.clear();
		}

		/* Just for Visualization */
		// Show rendered front and back image
		void showRenderImage() const
		{
// 			cv::namedWindow("front view", 0);
// 			cv::namedWindow("back view", 0);
			cv::imshow("front view", m_frontColorImg);
			cv::imshow("back view", m_backColorImg);
			cv::waitKey();
		};

		// Draw joints on render images
		void showJointsOnImage(int r = 3, int thickness = 2) const
		{
			cv::Mat frontImg = m_frontColorImg.clone();
			cv::Mat backImg = m_backColorImg.clone();

			std::map<int, cv::Point3f>::const_iterator it;
			for (it = m_frontJoint2DPoints.begin(); it != m_frontJoint2DPoints.end(); ++it)
			{
				cv::Point2f pt(it->second.x, it->second.y);
				int jointId = it->first;
				cv::circle(frontImg, pt, r, cv::Scalar(255, 0, 0), thickness);
			}
			for (it = m_backJoint2DPoints.begin(); it != m_backJoint2DPoints.end(); ++it)
			{
				cv::Point2f pt(it->second.x, it->second.y);
				int jointId = it->first;
				cv::circle(backImg, pt, r, cv::Scalar(255,0,0), thickness);
			}

			cv::imshow("front view", frontImg);
			cv::imshow("back view", backImg);
			cv::waitKey();
		}

	protected:
		void renderFrontAndBackImage();

	private:
		/* Algorithms */
		op::CvMatToOpInput *m_cvMatToOpInput;
		op::PoseExtractorCaffe *m_caffePoseExtractor;

		// variable
		pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr m_bodyCloud;

		std::set<int> m_interestedJointSet;

		// camera parameters
		cv::Size m_imgSize;
		float m_focal;

		cv::Mat m_frontColorImg;
		cv::Mat m_backColorImg;

		cv::Mat m_frontRenderMask;
		cv::Mat m_backRenderMask;

		cv::Mat m_frontDepthImg;
		cv::Mat m_backDepthImg;

		std::map<int, cv::Point3f> m_frontJoint2DPoints;
		std::map<int, cv::Point3f> m_backJoint2DPoints;

		std::map<int, cv::Point3f> m_frontJoint3DPoints;
		std::map<int, cv::Point3f> m_backJoint3DPoints;

		std::map<int, cv::Point3f> m_fusionJoint3DPoints;

		Eigen::Matrix4f m_T_src_to_local;
	};
}
