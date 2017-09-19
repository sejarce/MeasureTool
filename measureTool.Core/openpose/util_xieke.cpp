#include "util_xieke.h"

namespace CLOUDREAM
{
	int getTranformToInertialSystem(CloudXYZ::Ptr cloud,
		std::vector<Eigen::Vector3f>& basisVector, Eigen::Matrix4f& T)
	{
		if (cloud->points.size() < 3)
		{
			std::cerr << "util_xieke.h==>getTranformToInertialSystem:\t too less points(" << cloud->points.size() << ")\n";
			return -1;
		}

		pcl::MomentOfInertiaEstimation <pcl::PointXYZ> feature_extractor;
		feature_extractor.setInputCloud(cloud);
		feature_extractor.compute();

		pcl::PointXYZ minPtOBB, maxPtOBB;
		pcl::PointXYZ centerOBB;
		Eigen::Matrix3f R_from_inertial;
		Eigen::Vector3f majorVector, middleVector, minorVector;
		Eigen::Vector3f centroid;

		feature_extractor.getOBB(minPtOBB, maxPtOBB, centerOBB, R_from_inertial);
		feature_extractor.getEigenVectors(majorVector, middleVector, minorVector);
		feature_extractor.getMassCenter(centroid);

		std::vector<Eigen::Vector3f> srcBasis = { majorVector, middleVector, minorVector };

		// now T is transform from inertial system to world system 
		Eigen::Matrix4f T_from_inertial_to_world, T_from_world_to_inertial;
		composeRt(T_from_inertial_to_world, R_from_inertial, centroid);
		T_from_world_to_inertial = T_from_inertial_to_world.inverse();

		// check initial basis vector is empty or not
		if (basisVector.empty())
		{
			T = T_from_world_to_inertial;
			return 0;
		}


		// if not empty, change the inertial system to follow it
		Eigen::Vector3f initAxisX = basisVector[0];
		Eigen::Vector3f initAxisY = basisVector[1];
		Eigen::Vector3f initAxisZ = basisVector[2];

		std::vector<Eigen::Vector3f> expectBasis = { initAxisX, initAxisY, initAxisZ };

		Eigen::Matrix3f R_from_expect_to_inertial = Eigen::Matrix3f::Zero();
		for (int j = 0; j < 3; j++)
		{
			float max = 0;
			int id = -1;
			bool sameDirection;
			for (int i = 0; i < 3; i++)
			{
				float cosine = srcBasis[i].dot(expectBasis[j]);
				if (fabs(cosine) > max)
				{
					max = fabs(cosine);
					id = i;
					sameDirection = (cosine >= 0) ? true : false;
				}
			}

			for (int i = 0; i < 3; i++)
			{
				if (i == id)
					R_from_expect_to_inertial(i, j) = (sameDirection) ? 1 : -1;
			}
		}

		Eigen::Matrix3f R_from_inertial_to_expect = R_from_expect_to_inertial.transpose();
		Eigen::Vector3f t_from_inertial_to_expect(0, 0, 0);

		Eigen::Matrix4f T_from_inertial_to_expect;
		composeRt(T_from_inertial_to_expect, R_from_inertial_to_expect, t_from_inertial_to_expect);
		T = T_from_inertial_to_expect * T_from_world_to_inertial;
		return 1;
	}


	void computeNormal(CloudColorNormal::Ptr cloud, int k)
	{
		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_xyz(new pcl::PointCloud<pcl::PointXYZ>());
		pcl::copyPointCloud(*cloud, *cloud_xyz);

		pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
		ne.setInputCloud(cloud_xyz);
		pcl::search::KdTree<pcl::PointXYZ>::Ptr kdtree(new pcl::search::KdTree<pcl::PointXYZ>());
		ne.setSearchMethod(kdtree);
		ne.setKSearch(k);

		pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>());
		ne.compute(*normals);

		pcl::concatenateFields(*cloud_xyz, *normals, *cloud);
	}

	void reorientCloudNormal(CloudColorNormal::Ptr cloud, int k)
	{
		// Construct Vertices and Edges
		const int num_nodes = cloud->points.size();

		// Build Kd-tree
		pcl::KdTreeFLANN<pcl::PointXYZRGBNormal> kdtree;
		kdtree.setInputCloud(cloud);

		std::vector<Edge> edges;
		std::vector<float> weights;
		for (int i = 0; i < cloud->points.size(); i++)
		{
			auto query_point = cloud->points[i];
			std::vector<int> indices;
			std::vector<float> distances;
			kdtree.nearestKSearch(query_point, k, indices, distances);
			for (auto index : indices)
			{
				if (index > i)
				{
					Edge edge(i, index);

					Eigen::Vector3f queryNormal(
						cloud->points[i].normal_x,
						cloud->points[i].normal_y,
						cloud->points[i].normal_z);

					Eigen::Vector3f neighborNormal(
						cloud->points[index].normal_x,
						cloud->points[index].normal_y,
						cloud->points[index].normal_z);

					double weight = 1.0 - std::abs(queryNormal.normalized().dot(neighborNormal.normalized()));
					if (weight < 0)
						weight = 0; // safety check

					edges.push_back(edge);
					weights.push_back(weight);
				}
			}
		}

		RiemannianGraph g(edges.begin(), edges.end(), weights.begin(), num_nodes);


		// call prim's MST algorithms
		std::vector <boost::graph_traits<RiemannianGraph>::vertex_descriptor> p(num_vertices(g));
		boost::prim_minimum_spanning_tree(g, &p[0]);

		if (p.size() != num_nodes)
			std::cerr << "mst nodes num != graph nodes num!\n";

		// Construct MST Graph
		std::vector<Edge> mstEdges;
		for (std::size_t i = 0; i != p.size(); ++i)
		{
			if (p[i] != i)
			{
				Edge edge(p[i], i);
				mstEdges.push_back(edge);
			}
		}
		MST_Graph mstGraph(mstEdges.begin(), mstEdges.end(), num_nodes);

		// Construct flip instance
		FlipAdjustor<CloudColorNormal::Ptr> adjustor = FlipAdjustor<CloudColorNormal::Ptr>(cloud);

		// Check and flip when breadth first search
		boost::breadth_first_search(mstGraph, vertex(0, mstGraph), visitor(adjustor));

		// return re-oriented cloud 
		cloud = adjustor.getReorientedCloud();

		// compute cloud centroid
		Eigen::Vector4f centroid;
		pcl::compute3DCentroid(*cloud, centroid);
		cv::Point3f center(centroid(0), centroid(1), centroid(2));

		// check orient inside or outside, inverse all normals if inside toward centroid
		int count = 0;
		for (auto pt : cloud->points)
		{
			Eigen::Vector3f normal(pt.normal_x, pt.normal_y, pt.normal_z);
			Eigen::Vector3f outside(pt.x - center.x, pt.y - center.y, pt.z - center.z);

			if (normal.dot(outside) >= 0)
				count++;
		}

		// inverse all normals
		if (count < 0.5 * cloud->points.size())
		{
			
			for (int i = 0; i < cloud->points.size(); i++)
			{
				cloud->points[i].normal_x = -cloud->points[i].normal_x;
				cloud->points[i].normal_y = -cloud->points[i].normal_y;
				cloud->points[i].normal_z = -cloud->points[i].normal_z;
			}
		}
	}

}