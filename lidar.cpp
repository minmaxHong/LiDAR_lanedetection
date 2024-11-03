#include <iostream>
#include <cstdio>
#include <cstdlib>
#include <ros/ros.h>
// PCL specific includes
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/PointCloud.h>
#include <std_msgs/Float32.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_plane.h>
#include <pcl/sample_consensus/sac_model_sphere.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/features/normal_3d.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/console/print.h>
#include <pcl/common/common.h>
#include <pcl/surface/convex_hull.h>

#include "macaron_06/lidar_info.h"

#include <chrono>
using namespace std::chrono;
using namespace std;

union Vector3d
{
  struct { float x; float y; float z; };
  struct { float i; float j; float k; };
  struct { float a; float b; float c; };
};

union Vector4d
{
  struct { float x; float y; float z; float intensity; };
  struct { float i; float j; float k; float w; };
  struct { float a; float b; float c; float d; };
};

struct SizeLimit
{
  Vector3d maxSize;
  Vector3d minSize;
};

enum class EMode
{
  CONE,
  PERSON,
  DELIVERY,
  PROCESSED,
  MINJAE,
  CENTROID,
  LANE,
  EModeLength
};

enum class EPub
{
	CLUSTER,
	PROCESSED,
	PROCESSED_MINJAE,
	CENTROID,
	RANSAC,
	MAXZ,
	EPubLength
};

SizeLimit objectSizeLimit = { 7.0f, 7.0f, 7.0f, 0.0f, 0.0f, 0.05f };
SizeLimit coneSizeLimit = { 0.7f, 0.7f, 1.0f, 0.0f, 0.0f, 0.0f };
SizeLimit personSizeLimit = { 0.7f, 0.7f, 2.5f, 0.07f, 0.07f, 0.01f };
SizeLimit deliverySizeLimit = { 1.0f, 1.0f, 2.0f, 0.0f, 0.5f, 1.0f };

constexpr int laneIntensityThreshold = 70;



// utils ////////////////////////////////////

pcl::PointCloud<pcl::PointXYZI>::Ptr cloudmsg2cloud(sensor_msgs::PointCloud2 cloudmsg)
{
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);
  pcl::fromROSMsg(cloudmsg, *cloud);
  return cloud;
}

sensor_msgs::PointCloud2 cloud2cloudmsg(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud, ros::Time stamp)
{
  sensor_msgs::PointCloud2 cloudmsg;
  pcl::toROSMsg(*cloud, cloudmsg);
  cloudmsg.header.frame_id = "velodyne";
  cloudmsg.header.stamp = stamp;
  return cloudmsg;
}

macaron_06::lidar_info cloud2msg(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud,
std::vector<pcl::PointIndices> clusterIndices, ros::Time stamp)
{
  macaron_06::lidar_info msg;

  msg.data = cloud2cloudmsg(cloud, stamp);
  msg.header.stamp = stamp;

  std::vector<int32_t> clusters;
  std::vector<uint32_t> clusterSize;
  clusterSize.reserve(clusters.size());

  for (const auto& clusterIndice : clusterIndices)
  {
    std::vector<int> cluster = clusterIndice.indices;
    clusterSize.push_back(cluster.size());
    clusters.insert(clusters.end(), cluster.begin(), cluster.end());
  }

  msg.clusters = clusters;
  msg.clusterSize = clusterSize;

  return msg;
}

pcl::PointXYZI GetCenterPoint(pcl::PointXYZI a, pcl::PointXYZI b)
{
	pcl::PointXYZI ret;
	ret.x = (a.x + b.x) / 2;
	ret.y = (a.y + b.y) / 2;
	ret.z = (a.z + b.z) / 2;

	return ret;
}

////////////////////////////////////////////////

class PubArray
{
private:
	int epubToPubs[static_cast<int>(EPub::EPubLength)];

public:
	std::vector<ros::Publisher> pubs;

	PubArray()
	{
		for (int i = 0; i < static_cast<int>(EPub::EPubLength); ++i)
		{
			epubToPubs[i] = -1;
		}
	}

	PubArray(const std::vector<EPub>& list)
	{
		for (int i = 0; i < static_cast<int>(EPub::EPubLength); ++i)
		{
			epubToPubs[i] = -1;
		}

		int count = 0;
		for (auto& element : list)
		{
			epubToPubs[static_cast<int>(element)] = count;
			++count;
		}
	}

	~PubArray()
	{
		
	}

	int GetPubIndex(EPub ePub)
	{
		return epubToPubs[static_cast<int>(ePub)];
	}

	ros::Publisher& GetPub(EPub ePub)
	{
		int index = GetPubIndex(ePub);
		if (index < 0)
		{
			cerr << "GetPub Index Error" << endl;
			exit(EXIT_FAILURE);
		}
		return pubs[GetPubIndex(ePub)];
	}

	void AddPub(EPub ePub, ros::Publisher pub)
	{
		int count = pubs.size();
		epubToPubs[static_cast<int>(ePub)] = count;

		pubs.push_back(pub);
	}
};

class LidarManager
{
private:
	static void SetROI(const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloudIn,
			pcl::PointCloud<pcl::PointXYZI>::Ptr* cloudOut,
			const std::string& fieldName,
			const float min,
			const float max)
	{
		pcl::PassThrough<pcl::PointXYZI> pass;
		pass.setInputCloud(cloudIn);
		pass.setFilterFieldName(fieldName);
		pass.setFilterLimits(min, max);
		pass.filter(**cloudOut);
	}

	static void DeleteCenter(const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloudIn,
			pcl::PointCloud<pcl::PointXYZI>::Ptr* cloudOut,
			const float frontX = 0.3f, const float backX = 2.0f,
			const float frontY = 0.7f, const float backY = 0.7f,
			const float frontZ = 1.0f, const float backZ = 0.5f)
	{
		pcl::PointCloud<pcl::PointXYZI>::Ptr ret(new pcl::PointCloud<pcl::PointXYZI>());

		for (pcl::PointXYZI point : cloudIn->points)
		{
			if (point.x < -backX || point.x > frontX || point.y < -backY || point.y > frontY || point.z < -backZ || point.z > frontZ)
			{
			ret->points.push_back(point);
			}
		}

		*cloudOut = ret;
	}
	
	static void Voxelize(const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloudIn,
			pcl::PointCloud<pcl::PointXYZI>::Ptr* cloudOut,
			const float voxelSize)
	{
		pcl::VoxelGrid<pcl::PointXYZI> vg;
		vg.setInputCloud(cloudIn);
		vg.setLeafSize(voxelSize, voxelSize, voxelSize);
		vg.filter(**cloudOut);
	}

	static bool RansacPerpendicularZ(const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloudIn,
			pcl::PointCloud<pcl::PointXYZI>::Ptr* cloudOut,
			const int maxIteration,
			const float distanceThreshold,
			pcl::PointCloud<pcl::PointXYZI>::Ptr* cloudLane=nullptr)
	{
		constexpr float epsAngle = 0.1;

		pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients());
		pcl::SACSegmentation<pcl::PointXYZI> seg;
		seg.setOptimizeCoefficients(true);
		seg.setModelType(pcl::SACMODEL_PERPENDICULAR_PLANE);
		seg.setMethodType(pcl::SAC_RANSAC);
		seg.setMaxIterations(maxIteration);
		seg.setDistanceThreshold(distanceThreshold);
		seg.setEpsAngle(epsAngle);
		seg.setAxis(Eigen::Vector3f::UnitZ());

		pcl::ExtractIndices<pcl::PointXYZI> extract;
		pcl::PointIndices::Ptr inliers(new pcl::PointIndices());

		seg.setInputCloud(cloudIn);
		seg.segment(*inliers, *coefficients);
		if (inliers->indices.size() == 0)
		{
			return false;
		}

		extract.setInputCloud(cloudIn);
		extract.setIndices(inliers);

		if (cloudLane != nullptr)
		{
			extract.setNegative(false);
			extract.filter(**cloudLane);
		}

		extract.setNegative(true);
		extract.filter(**cloudOut);

		return true;
	}

	static pcl::PointCloud<pcl::PointXYZI>::Ptr Ransac(const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud,
			pcl::PointCloud<pcl::PointXYZI>::Ptr* cloudLane=nullptr)
	{
		pcl::PointCloud<pcl::PointXYZI>::Ptr cloudCopy(new pcl::PointCloud<pcl::PointXYZI>);
		pcl::copyPointCloud(*cloud, *cloudCopy);

		DeleteCenter(cloudCopy, &cloudCopy);
		SetROI(cloudCopy, &cloudCopy, "y", -8.0f, 8.0f);
		SetROI(cloudCopy, &cloudCopy, "x", -5.0f, 15.0f);

		RansacPerpendicularZ(cloudCopy, &cloudCopy, 1000, 0.07f, cloudLane);

		return cloudCopy;
	}

	static pcl::PointCloud<pcl::PointXYZI>::Ptr RansacMinjae(const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud)
	{
		pcl::PointCloud<pcl::PointXYZI>::Ptr cloudCopy(new pcl::PointCloud<pcl::PointXYZI>);
		pcl::copyPointCloud(*cloud, *cloudCopy);

		DeleteCenter(cloudCopy, &cloudCopy);
		SetROI(cloudCopy, &cloudCopy, "x", -0.5f, 5.0f);
		SetROI(cloudCopy, &cloudCopy, "y", -4.0f, 4.0f);
		SetROI(cloudCopy, &cloudCopy, "z", -0.8f, 2.5f);

		RansacPerpendicularZ(cloudCopy, &cloudCopy, 2000, 0.07f);

		return cloudCopy;
	}

	static std::vector<pcl::PointIndices> Cluster(const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud)
	{
		std::vector<pcl::PointIndices> cluster_indices;

		pcl::search::KdTree<pcl::PointXYZI>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZI>);
		tree->setInputCloud(cloud);

		pcl::EuclideanClusterExtraction<pcl::PointXYZI> ec;
		ec.setInputCloud(cloud); 
		ec.setClusterTolerance(0.5f);
		ec.setMinClusterSize(3);
		ec.setMaxClusterSize(1000);
		ec.setSearchMethod(tree);
		ec.extract(cluster_indices);

		return cluster_indices;
	}

	static void Get3dBoundingBox(const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloudIn,
			pcl::PointCloud<pcl::PointXYZI>::Ptr* cloudOut,
			std::vector<pcl::PointIndices>& objectIndices,
			const bool bOnlyCentroid=false)
	{
		pcl::PointCloud<pcl::PointXYZI>::Ptr centroidPointCloud(new pcl::PointCloud<pcl::PointXYZI>);
		pcl::PointCloud<pcl::PointXYZI>::Ptr clusterCloud(new pcl::PointCloud<pcl::PointXYZI>);
		pcl::ExtractIndices<pcl::PointXYZI> extract;
		for (int i = 0; i < objectIndices.size(); ++i)
		{
			extract.setInputCloud(cloudIn);
			extract.setIndices(boost::make_shared<pcl::PointIndices>(objectIndices[i]));
			extract.setNegative(false);
			extract.filter(*clusterCloud);

			pcl::PointXYZI minPoint, maxPoint;
			pcl::getMinMax3D(*clusterCloud, minPoint, maxPoint);
			Vector3d size = {maxPoint.x - minPoint.x, maxPoint.y - minPoint.y, maxPoint.z - minPoint.z};

			pcl::PointXYZI centerPoint = GetCenterPoint(maxPoint, minPoint);
			
			centerPoint.intensity = 0.0f;
			if (maxPoint.z >= 0.25)
			{
				centerPoint.intensity = 1.0f;
			}

			
			if (!bOnlyCentroid && size.x > 0.1f && size.y > 0.1f)
			{
				pcl::PointXYZI xMaxPoint = centerPoint;
				pcl::PointXYZI xMinPoint = centerPoint;
				pcl::PointXYZI yMaxPoint = centerPoint;
				pcl::PointXYZI yMinPoint = centerPoint;
				xMaxPoint.x = maxPoint.x;
				xMinPoint.x = minPoint.x;
				yMaxPoint.y = maxPoint.y;
				yMinPoint.y = minPoint.y;

				pcl::PointXYZI leftFrontPoint = xMaxPoint;
				pcl::PointXYZI rightFrontPoint = xMaxPoint;
				pcl::PointXYZI leftBackPoint = xMinPoint;
				pcl::PointXYZI rightBackPoint = xMinPoint;
				leftFrontPoint.y = yMaxPoint.y;
				rightFrontPoint.y = yMinPoint.y;
				leftBackPoint.y = yMaxPoint.y;
				rightBackPoint.y = yMinPoint.y;

				centroidPointCloud->points.push_back(xMaxPoint);
				centroidPointCloud->points.push_back(xMinPoint);
				centroidPointCloud->points.push_back(yMaxPoint);
				centroidPointCloud->points.push_back(yMinPoint);
				centroidPointCloud->points.push_back(leftFrontPoint);
				centroidPointCloud->points.push_back(rightFrontPoint);
				centroidPointCloud->points.push_back(leftBackPoint);
				centroidPointCloud->points.push_back(rightBackPoint);

				if (size.x > 1.0f)
				{
					pcl::PointXYZI leftInterPoints[2] = { GetCenterPoint(leftFrontPoint, yMaxPoint), GetCenterPoint(yMaxPoint, leftBackPoint) };
					pcl::PointXYZI rightInterPoints[2] = { GetCenterPoint(rightFrontPoint, yMinPoint), GetCenterPoint(yMinPoint, rightBackPoint) };

					centroidPointCloud->points.push_back(leftInterPoints[0]);
					centroidPointCloud->points.push_back(leftInterPoints[1]);
					centroidPointCloud->points.push_back(rightInterPoints[0]);
					centroidPointCloud->points.push_back(rightInterPoints[1]);
				}
				if (size.y > 1.0f)
				{
					pcl::PointXYZI frontInterPoints[2] = { GetCenterPoint(leftFrontPoint, xMaxPoint), GetCenterPoint(xMaxPoint, rightFrontPoint) };
					pcl::PointXYZI backInterPoints[2] = { GetCenterPoint(leftBackPoint, xMinPoint), GetCenterPoint(xMinPoint, rightBackPoint) };

					centroidPointCloud->points.push_back(frontInterPoints[0]);
					centroidPointCloud->points.push_back(frontInterPoints[1]);
					centroidPointCloud->points.push_back(backInterPoints[0]);
					centroidPointCloud->points.push_back(backInterPoints[1]);
				}
			}
			else
			{
				centroidPointCloud->points.push_back(centerPoint);
			}
		}

		pcl::copyPointCloud(*centroidPointCloud, **cloudOut);
	}

	static bool IsObject(float xLength, float yLength, float zLength, const SizeLimit& sizeLimit)
	{
		float lengths[] = {xLength, yLength, zLength};
		std::sort(lengths, lengths + 3, std::greater<double>());

		if (xLength > sizeLimit.maxSize.x || xLength < sizeLimit.minSize.x)
		{
			return false;
		}
		if (yLength > sizeLimit.maxSize.y || yLength < sizeLimit.minSize.y)
		{
			return false;
		}
		if (zLength > sizeLimit.maxSize.z || zLength < sizeLimit.minSize.z)
		{
			return false;
		}

		return true;
	}

	static std::vector<int> FindObjectWithSize(const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloudIn,
			pcl::PointCloud<pcl::PointXYZI>::Ptr* cloudOut,
			const std::vector<pcl::PointIndices>& clusterIndices,
			const SizeLimit& sizeLimit)
	{
		std::vector<int> result;
		pcl::ExtractIndices<pcl::PointXYZI> extract;
		for (int i = 0; i < clusterIndices.size(); ++i)
		{
			pcl::PointCloud<pcl::PointXYZI>::Ptr clusterCloud(new pcl::PointCloud<pcl::PointXYZI>());
			pcl::PointCloud<pcl::PointXYZ>::Ptr clusterCloud2(new pcl::PointCloud<pcl::PointXYZ>());
			extract.setInputCloud(cloudIn);
			extract.setIndices(boost::make_shared<pcl::PointIndices>(clusterIndices[i]));
			extract.setNegative(false);
			extract.filter(*clusterCloud);

			pcl::copyPointCloud(*clusterCloud, *clusterCloud2);
			pcl::PointXYZ minPoint, maxPoint;
			pcl::getMinMax3D(*clusterCloud2, minPoint, maxPoint);
			float xLength = maxPoint.x - minPoint.x;
			float yLength = maxPoint.y - minPoint.y;
			float zLength = maxPoint.z - minPoint.z;


			if (IsObject(xLength, yLength, zLength, sizeLimit))
			{
				result.push_back(i);
			}
		}

		for (const int idx : result)
		{
			pcl::PointCloud<pcl::PointXYZI>::Ptr tempCloud(new pcl::PointCloud<pcl::PointXYZI>());
			extract.setInputCloud(cloudIn);
			extract.setIndices(boost::make_shared<pcl::PointIndices>(clusterIndices[idx]));
			extract.setNegative(false);
			extract.filter(*tempCloud);
			**cloudOut += *tempCloud;
		}

		return result;
	}


public:
	static bool mode[7];
	static PubArray pubArray;

	static void callbackLidar(const sensor_msgs::PointCloud2ConstPtr& input)
	{
		// ros::Time stamp = (*input).header.stamp;
		ros::Time stamp = ros::Time::now();
		
		pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(cloudmsg2cloud(*input));

		pcl::PointCloud<pcl::PointXYZI>::Ptr cloudLane(new pcl::PointCloud<pcl::PointXYZI>);
		
		pcl::PointCloud<pcl::PointXYZI>::Ptr cloudOut(new pcl::PointCloud<pcl::PointXYZI>);
		pcl::PointCloud<pcl::PointXYZI>::Ptr ransacResult;

		std::chrono::system_clock::time_point startTime = std::chrono::system_clock::now();

		macaron_06::lidar_info _m_cluster;

		float minY = 100.0f, maxY = -100.0f;
		if (mode[static_cast<int>(EMode::LANE)])
		{
			for (int i = 0; i < cloud->points.size(); ++i)
			{
				pcl::PointXYZI point = cloud->points[i];
				if (point.x > 1.0f || point.x < -1.0f || point.z < 0.3f)
				{
					continue;
				}
				if (point.y < minY)
				{
					minY = point.y;
				}
				if (point.y > maxY)
				{
					maxY = point.y;
				}
			}


			ransacResult = Ransac(cloud, &cloudLane);
			if (ransacResult->points.size() == 0)
			{
				return;
			}

			float maxZ = cloud->points[0].z;
			for (int i = 1; i < cloud->points.size(); ++i)
			{
				if (cloud->points[i].x < 0 || cloud->points[i].x > 10 || cloud->points[i].y < -7 || cloud->points[i].y > 7)
				{
					continue;
				}
				if (cloud->points[i].z > maxZ)
				{
					maxZ = cloud->points[i].z;
				}
			}
			// cout << "maxZ: " << maxZ << "\n";
			std_msgs::Float32 f_msg;
			f_msg.data = maxZ;
			pubArray.GetPub(EPub::MAXZ).publish(f_msg);
			
			sensor_msgs::PointCloud2 _m_ransac;
			_m_ransac = cloud2cloudmsg(cloudLane, stamp);
			pubArray.GetPub(EPub::RANSAC).publish(_m_ransac);
		}
		else
		{
			ransacResult = Ransac(cloud);
		}

		std::chrono::system_clock::time_point laneTime = std::chrono::system_clock::now();

		Voxelize(ransacResult, &ransacResult, 0.2f);
		std::vector<pcl::PointIndices> clusterResult = Cluster(ransacResult);
		std::vector<int> objectIndiceses = FindObjectWithSize(ransacResult, &cloudOut, clusterResult, objectSizeLimit);
		std::vector<pcl::PointIndices> objectIndices;
		for (const int i : objectIndiceses)
		{
			objectIndices.push_back(clusterResult[i]);
		}

		std::chrono::system_clock::time_point clusteringTime = std::chrono::system_clock::now();

		if (mode[static_cast<int>(EMode::MINJAE)])
		{
			pcl::PointCloud<pcl::PointXYZI>::Ptr centroidPointCloud(new pcl::PointCloud<pcl::PointXYZI>);
			LidarManager::Get3dBoundingBox(ransacResult, &centroidPointCloud, objectIndices);

			sensor_msgs::PointCloud2 _m_centroidPoints;
			_m_centroidPoints = cloud2cloudmsg(centroidPointCloud, stamp);
			pubArray.GetPub(EPub::PROCESSED_MINJAE).publish(_m_centroidPoints);
		}

		std::chrono::system_clock::time_point minjaeTime = std::chrono::system_clock::now();

		if (mode[static_cast<int>(EMode::CENTROID)])
		{
			pcl::PointCloud<pcl::PointXYZI>::Ptr centroidPointCloud(new pcl::PointCloud<pcl::PointXYZI>);
			LidarManager::Get3dBoundingBox(ransacResult, &centroidPointCloud, objectIndices, true);

			sensor_msgs::PointCloud2 _m_centroidPoints;
			_m_centroidPoints = cloud2cloudmsg(centroidPointCloud, stamp);
			pubArray.GetPub(EPub::CENTROID).publish(_m_centroidPoints);
		}

		std::chrono::system_clock::time_point centroidTime = std::chrono::system_clock::now();

		
		_m_cluster = cloud2msg(ransacResult, objectIndices, stamp);
		_m_cluster.minY = minY;
		_m_cluster.maxY = maxY;
		if (mode[static_cast<int>(EMode::CONE)])
		{
			pcl::PointCloud<pcl::PointXYZI>::Ptr conesCloud(new pcl::PointCloud<pcl::PointXYZI>);
			std::vector<int> coneIndices = FindObjectWithSize(ransacResult, &conesCloud, objectIndices, coneSizeLimit);
			_m_cluster.cones = coneIndices;
		}
		if (mode[static_cast<int>(EMode::PERSON)])
		{
			pcl::PointCloud<pcl::PointXYZI>::Ptr peopleCloud(new pcl::PointCloud<pcl::PointXYZI>);
			std::vector<int> personIndices = FindObjectWithSize(ransacResult, &peopleCloud, objectIndices, personSizeLimit);
			_m_cluster.person = personIndices;
		}
		if (mode[static_cast<int>(EMode::DELIVERY)])
		{
			pcl::PointCloud<pcl::PointXYZI>::Ptr deliveryCloud(new pcl::PointCloud<pcl::PointXYZI>);
			std::vector<int> deliveryIndices = FindObjectWithSize(ransacResult, &deliveryCloud, objectIndices, deliverySizeLimit);
			_m_cluster.delivery = deliveryIndices;
		}
		if (pubArray.GetPubIndex(EPub::CLUSTER) >= 0)
		{
			pubArray.GetPub(EPub::CLUSTER).publish(_m_cluster);
		}

		std::chrono::system_clock::time_point clusterTime = std::chrono::system_clock::now();

		if (mode[static_cast<int>(EMode::PROCESSED)])
		{
			sensor_msgs::PointCloud2 _m_processedPoints;
			_m_processedPoints = cloud2cloudmsg(cloudOut, stamp);
			pubArray.GetPub(EPub::PROCESSED).publish(_m_processedPoints);
		}

		std::chrono::system_clock::time_point processedTime = std::chrono::system_clock::now();

		std::chrono::duration<double>start2lane = laneTime - startTime;
		std::chrono::duration<double>lane2clustering = clusteringTime - laneTime;
		std::chrono::duration<double>clustering2minjae = minjaeTime - clusteringTime;
		std::chrono::duration<double>minjae2centroid = centroidTime - minjaeTime;
		std::chrono::duration<double>centroid2cluster = clusterTime - centroidTime;
		std::chrono::duration<double>cluster2processed = processedTime - clusterTime;
		std::chrono::duration<double>totalTime = processedTime - startTime;

		cout << "start -> lane: " << start2lane.count() << endl;
		cout << "lane -> clustering: " << lane2clustering.count() << endl;
		cout << "clustering -> minjae: " << clustering2minjae.count() << endl;
		cout << "minjae -> centroid: " << minjae2centroid.count() << endl;
		cout << "centroid -> cluster_msg: " << centroid2cluster.count() << endl;
		cout << "cluster_msg -> processed: " << cluster2processed.count() << endl;
		cout << "start -> processed: " << totalTime.count() << endl;
		cout << "-------------------" << endl;
	}
};

bool LidarManager::mode[7] = {false, false, false, false, false, false, false};
PubArray LidarManager::pubArray = PubArray();


int main(int argc, char** argv)
{
	freopen("/dev/null", "w", stderr);
	ros::init(argc, argv, "lidar_manager");
	ros::NodeHandle nh;

	std::string modeList[7] = { "cone", "person", "delivery", "processed", "minjae", "centroid", "lane" };

	if (argc >= 2)
	{
		std::string modes = argv[1];
		std::string buffer;
		istringstream ss(modes);
		while (getline(ss, buffer, ','))
		{
			for (int i = 0; i < sizeof(LidarManager::mode) / sizeof(bool); ++i)
			{
				if (buffer.compare(modeList[i]) == 0)
				{
					LidarManager::mode[i] = true;
					break;
				}
			}
		}
		cout << "use mode: ";
		for (int i = 0; i < sizeof(LidarManager::mode) / sizeof(bool); ++i)
		{
			if (LidarManager::mode[i])
			{
				cout << modeList[i] << " ";
			}
		}
		cout << endl;
	}
	else
	{
		cout << "Please Select Mode" << endl;

		cout << "List of Mode: ";
		for (int i = 0; i < sizeof(LidarManager::mode) / sizeof(bool); ++i)
		{
			cout << "[" << modeList[i] << "] ";
		}
		cout << endl;
		cout << "ex) cone,person,processed" << endl;

		return 1;
	}

	{ // make pub
		int count = 0;

		if (LidarManager::mode[static_cast<int>(EMode::CONE)]
			|| LidarManager::mode[static_cast<int>(EMode::PERSON)]
			|| LidarManager::mode[static_cast<int>(EMode::DELIVERY)]
			|| LidarManager::mode[static_cast<int>(EMode::LANE)])
		{
			LidarManager::pubArray.AddPub(EPub::CLUSTER, nh.advertise<macaron_06::lidar_info>("cluster", 10));
		}

		if (LidarManager::mode[static_cast<int>(EMode::PROCESSED)])
		{
			LidarManager::pubArray.AddPub(EPub::PROCESSED, nh.advertise<sensor_msgs::PointCloud2>("processed", 10));
		}

		if (LidarManager::mode[static_cast<int>(EMode::MINJAE)])
		{
			LidarManager::pubArray.AddPub(EPub::PROCESSED_MINJAE, nh.advertise<sensor_msgs::PointCloud2>("super_minjae_centroid", 10));
		}

		if (LidarManager::mode[static_cast<int>(EMode::CENTROID)])
		{
			LidarManager::pubArray.AddPub(EPub::CENTROID, nh.advertise<sensor_msgs::PointCloud2>("centroid", 10));
		}

		if (LidarManager::mode[static_cast<int>(EMode::LANE)])
		{
			LidarManager::pubArray.AddPub(EPub::RANSAC, nh.advertise<sensor_msgs::PointCloud2>("minmax_ransac", 10));
			LidarManager::pubArray.AddPub(EPub::MAXZ, nh.advertise<std_msgs::Float32>("ask_to_sebin", 10));
		}
	}

	ros::Subscriber lidarSub = nh.subscribe("/velodyne_points", 10, LidarManager::callbackLidar);

	ros::spin();
}
