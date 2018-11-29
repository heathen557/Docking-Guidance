#ifndef CLUSTER_H_
#define CLUSTER_H_ 
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/common/pca.h>
#include <pcl/features/fpfh_omp.h>
#include <pcl/features/normal_3d_omp.h>




class Cluster
{
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr point_cloud_;
	pcl::PointXYZ min_point_;
	pcl::PointXYZ max_point_;
	//pcl::PointXYZI average_point_;
	pcl::PointXYZ centroid_;
	std::vector<int> point_indices_;
	double orientation_angle_;
	float length_, width_, height_;

	//jsk_recognition_msgs::BoundingBox 	bounding_box_;
	//geometry_msgs::PolygonStamped 		polygon_;

	//std::string							label_;
	int									id_;
	int									r_, g_, b_;

	//Eigen::Matrix3f 					eigen_vectors_;
	//Eigen::Vector3f 					eigen_values_;

	bool								valid_cluster_;

public:
	void setCloud(const pcl::PointCloud<pcl::PointXYZI>::Ptr &in_cloud_ptr, std::vector<int> &in_cluster_indices, int in_id, int in_r, int in_g, int in_b);

	Cluster();
	virtual ~Cluster();

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr getCloud();
	pcl::PointXYZ 						getMinPoint();
	pcl::PointXYZ 						getMaxPoint();
	pcl::PointXYZ 						getAveragePoint();
	pcl::PointXYZ 						getCentroid();
	std::vector<int>                    getPointIndices();


	//jsk_recognition_msgs::BoundingBox	GetBoundingBox();
	//geometry_msgs::PolygonStamped GetPolygon();
	double								getOrientationAngle();
	float								getLength();
	float								getWidth();
	float								getHeight();
	int									getId();
	std::string							getLabel();
	//Eigen::Matrix3f						GetEigenVectors();
	//Eigen::Vector3f						GetEigenValues();
	bool								isValid();
	void								setValidity(bool in_valid);
	pcl::PointCloud<pcl::PointXYZ>::Ptr	joinCloud(const pcl::PointCloud<pcl::PointXYZI>::Ptr in_cloud_ptr);
	//std::vector<float> getFpfhDescriptor(const unsigned int& in_ompnum_threads, const double& in_normal_search_radius, const double& in_fpfh_search_radius);


};
typedef boost::shared_ptr<Cluster> ClusterPtr;

#endif /*CLUSTER_H*/