#pragma once

#include <vector>

#include <Eigen/Dense>

#include <opencv2/core/core.hpp>

#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/prim_minimum_spanning_tree.hpp>


#include <pcl/io/ply_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/features/moment_of_inertia_estimation.h>
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/common/centroid.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>


namespace CLOUDREAM
{
	typedef pcl::PointCloud<pcl::PointXYZRGBNormal> CloudColorNormal;
	typedef pcl::PointCloud<pcl::PointXYZRGB> CloudColor;
	typedef pcl::PointCloud<pcl::PointXYZ> CloudXYZ;


	inline void composeRt(Eigen::Matrix4f& T, Eigen::Matrix3f R, Eigen::Vector3f t)
	{
		T = Eigen::Matrix4f::Identity();
		T.topLeftCorner(3, 3) = R;
		T.topRightCorner(3, 1) = t;
	};

	inline void decomposeRt(Eigen::Matrix4f T, Eigen::Matrix3f& R, Eigen::Vector3f& t)
	{
		R = T.topLeftCorner(3, 3);
		t = T.topRightCorner(3, 1);
	};
	
	template <typename PointT>
	int downSampling(pcl::PointCloud<PointT>& cloud, float grid_size = 0.001/*1mm*/)
	{
		pcl::VoxelGrid<PointT> filter;
		filter.setInputCloud(cloud.makeShared());
		filter.setLeafSize(grid_size, grid_size, grid_size);
		filter.filter(cloud);

		return cloud.points.size();
	}


	int getTranformToInertialSystem(CloudXYZ::Ptr cloud,
		std::vector<Eigen::Vector3f>& basisVector, Eigen::Matrix4f& T);

	// Compute cloud's normal with local algorithm
	// Please use reorientCloudNormal() later if you want obtain global consistency outside normals
	void computeNormal(CloudColorNormal::Ptr cloud, int k = 20);

	// Re-orient cloud normal by MST algorithm
	void reorientCloudNormal(CloudColorNormal::Ptr cloud, int k = 12);


	/************************************************************************/
	/*	For Re-orient Normal Algorithm
	/************************************************************************/
	typedef boost::adjacency_list <boost::vecS, boost::vecS,	// 前两个模板参数分别表示边和顶点的存储所用的容器类型，这里表示使用vector
		boost::undirectedS,										// 第三个模板参数表示图的类型，这里表示无向图
		boost::property <boost::vertex_distance_t, int>,		// 顶点的属性
		boost::property <boost::edge_weight_t, float>> RiemannianGraph;	// 边的属性

	typedef boost::adjacency_list< boost::vecS, boost::vecS,
		boost::directedS,
		boost::property <boost::vertex_distance_t, int>> MST_Graph;

	typedef std::pair<int, int> Edge;

	template <typename PointCloudT>
	class FlipAdjustor : public boost::default_bfs_visitor
	{
	public:
		// Constructor
		FlipAdjustor(PointCloudT cloud) {
			m_cloud = pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr(new pcl::PointCloud<pcl::PointXYZRGBNormal>());
			m_cloud = cloud;
		};

		template <typename Edge, typename Graph>
		void tree_edge(Edge e, const Graph& g) const
		{
			typename boost::graph_traits<Graph>::vertex_descriptor
				u = source(e, g), v = target(e, g);

			Eigen::Vector3f u_normal(
				m_cloud->points[u].normal_x,
				m_cloud->points[u].normal_y,
				m_cloud->points[u].normal_z);

			Eigen::Vector3f v_normal(
				m_cloud->points[v].normal_x,
				m_cloud->points[v].normal_y,
				m_cloud->points[v].normal_z);

			if (u_normal.dot(v_normal) < 0)
			{
				v_normal.normalize();

				m_cloud->points[v].normal_x = -v_normal(0);
				m_cloud->points[v].normal_y = -v_normal(1);
				m_cloud->points[v].normal_z = -v_normal(2);

			}
		}

		PointCloudT getReorientedCloud()
		{
			return m_cloud;
		};

	private:
		PointCloudT m_cloud;
	};
}

