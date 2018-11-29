#include "Cluster.h"

Cluster::Cluster()
{
	valid_cluster_ = true;
}

/*geometry_msgs::PolygonStamped Cluster::getPolygon()
{
return polygon_;
}*/

/*jsk_recognition_msgs::BoundingBox Cluster::getBoundingBox()
{
return bounding_box_;
}
*/
pcl::PointCloud<pcl::PointXYZRGB>::Ptr Cluster::getCloud()
{
	return point_cloud_;
}

pcl::PointXYZ Cluster::getMinPoint()
{
	return min_point_;
}

pcl::PointXYZ Cluster::getMaxPoint()
{
	return max_point_;
}

pcl::PointXYZ Cluster::getCentroid()
{
	return centroid_;
}

std::vector<int> Cluster::getPointIndices()
{
	return point_indices_;
}

/*pcl::PointXYZ Cluster::getAveragePoint()
{
return average_point_;
}*/

double Cluster::getOrientationAngle()
{
	return orientation_angle_;
}

float Cluster::getHeight()
{
	return height_;
}

float Cluster::getWidth()
{
	return width_;
}

float Cluster::getLength()
{
	return length_;
}
/*Eigen::Matrix3f Cluster::getEigenVectors()
{
return eigen_vectors_;
}*/

//Eigen::Vector3f Cluster::getEigenValues()
//{
//	return eigen_values_;
//}

void Cluster::setCloud(const pcl::PointCloud<pcl::PointXYZI>::Ptr &in_cloud_ptr, std::vector<int> &in_cluster_indices, int in_id, int in_r, int in_g, int in_b)
{
	//label_ = in_label;
	point_indices_ = in_cluster_indices;
	id_ = in_id;
	//r_ = in_r;	g_ = in_g;	b_ = in_b;
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr current_cluster(new pcl::PointCloud<pcl::PointXYZRGB>);
	float min_x = std::numeric_limits<float>::max(); float max_x = -std::numeric_limits<float>::max();
	float min_y = std::numeric_limits<float>::max(); float max_y = -std::numeric_limits<float>::max();
	float min_z = std::numeric_limits<float>::max(); float max_z = -std::numeric_limits<float>::max();
	float average_x = 0, average_y = 0, average_z = 0;

	for (auto pit = in_cluster_indices.begin(); pit != in_cluster_indices.end(); ++pit)
	{
		//fill new colored cluster point by point
		pcl::PointXYZRGB p;
		p.x = in_cloud_ptr->points[*pit].x;
		p.y = in_cloud_ptr->points[*pit].y;
		p.z = in_cloud_ptr->points[*pit].z;
		p.r = in_r;
		p.g = in_g;
		p.b = in_b;

		//average_x += p.x;
		//average_y += p.y;		
		//average_z += p.z;
		centroid_.x += p.x;
		centroid_.y += p.y;
		centroid_.z += p.z;
		current_cluster->push_back(p);

		if (p.x<min_x)	min_x = p.x;
		if (p.y<min_y)	min_y = p.y;
		if (p.z<min_z)	min_z = p.z;
		if (p.x>max_x)	max_x = p.x;
		if (p.y>max_y)	max_y = p.y;
		if (p.z>max_z)	max_z = p.z;
	}
	min_point_.x = min_x;
	min_point_.y = min_y;
	min_point_.z = min_z;
	max_point_.x = max_x;
	max_point_.y = max_y;
	max_point_.z = max_z;

	if (in_cluster_indices.size() > 0)
	{
		centroid_.x /= in_cluster_indices.size();
		centroid_.y /= in_cluster_indices.size();
		centroid_.z /= in_cluster_indices.size();

		//average_x /= in_cluster_indices.size();
		//average_y /= in_cluster_indices.size();
		//average_z /= in_cluster_indices.size();
	}

	//average_point_.x = average_x; 
	//average_point_.y = average_y;
	//average_point_.z = average_z;

	//calculate bounding box
	length_ = max_point_.x - min_point_.x;
	width_ = max_point_.y - min_point_.y;
	height_ = max_point_.z - min_point_.z;

	/*bounding_box_.header = in_ros_header;

	bounding_box_.pose.position.x = min_point_.x + length_ / 2;
	bounding_box_.pose.position.y = min_point_.y + width_ / 2;
	bounding_box_.pose.position.z = min_point_.z + height_ / 2;

	bounding_box_.dimensions.x = ((length_<0) ? -1 * length_ : length_);
	bounding_box_.dimensions.y = ((width_<0) ? -1 * width_ : width_);
	bounding_box_.dimensions.z = ((height_<0) ? -1 * height_ : height_);*/

	//pose estimation
	double rz = 0;

	//{
	//	std::vector<cv::Point2f> points;
	//	for (unsigned int i = 0; i<current_cluster->points.size(); i++)
	//	{
	//		cv::Point2f pt;
	//		pt.x = current_cluster->points[i].x;
	//		pt.y = current_cluster->points[i].y;
	//		points.push_back(pt);
	//	}

	//	if (in_estimate_pose)
	//	{
	//		//pose estimation for the cluster
	//		//test using linear regressionwidth
	//		//Slope(b) = (N¦²XY - (¦²X)(¦²Y)) / (N¦²X2 - (¦²X)2)

	//		//float sum_x=0, sum_y=0, sum_xy=0, sum_xx=0;
	//		//for (unsigned int i=0; i<current_cluster->points.size(); i++)
	//		//{
	//		//	sum_x+= current_cluster->points[i].x;
	//		//	sum_y+= current_cluster->points[i].y;
	//		//	sum_xy+= current_cluster->points[i].x*current_cluster->points[i].y;
	//		//	sum_xx+= current_cluster->points[i].x*current_cluster->points[i].x;
	//		//}
	//		//double slope= (current_cluster->points.size()*sum_xy - (sum_x*sum_y))/(current_cluster->points.size()*sum_xx - sum_x*sum_x);
	//		//rz = atan(-slope);
	//		cv::RotatedRect box = minAreaRect(points);
	//		rz = box.angle*3.14 / 180;
	//		bounding_box_.pose.position.x = box.center.x;
	//		bounding_box_.pose.position.y = box.center.y;
	//		//std::cout << bounding_box_.pose.position.y << " " << bounding_box_.pose.position.x  << std::endl;
	//		bounding_box_.dimensions.x = box.size.width;
	//		bounding_box_.dimensions.y = box.size.height;
	//	}

	//	std::vector<cv::Point2f> hull;
	//	cv::convexHull(points, hull);

	//	polygon_.header = in_ros_header;
	//	for (size_t i = 0; i< hull.size() + 1; i++)
	//	{
	//		geometry_msgs::Point32 point;
	//		point.x = hull[i%hull.size()].x;
	//		point.y = hull[i%hull.size()].y;
	//		point.z = min_point_.z;
	//		polygon_.polygon.points.push_back(point);
	//	}

	//	for (size_t i = 0; i< hull.size() + 1; i++)
	//	{
	//		geometry_msgs::Point32 point;
	//		point.x = hull[i%hull.size()].x;
	//		point.y = hull[i%hull.size()].y;
	//		point.z = max_point_.z;
	//		polygon_.polygon.points.push_back(point);
	//	}

	//	/*cv::Point2f rect_points[4];
	//	box.points(rect_points);*/
	//}

	//set bounding box direction
	/*tf::Quaternion quat = tf::createQuaternionFromRPY(0.0, 0.0, rz);
	tf::quaternionTFToMsg(quat, bounding_box_.pose.orientation);*/

	current_cluster->width = current_cluster->points.size();
	current_cluster->height = 1;
	current_cluster->is_dense = true;

	//Get EigenValues, eigenvectors
	if (current_cluster->points.size() > 0)
	{
		pcl::PCA<pcl::PointXYZI> current_cluster_pca;
		pcl::PointCloud<pcl::PointXYZI>::Ptr current_cluster_mono(new pcl::PointCloud<pcl::PointXYZI>);

		/*pcl::copyPointCloud<pcl::PointXYZRGB, pcl::PointXYZ>(*current_cluster, *current_cluster_mono);

		current_cluster_pca.setInputCloud(current_cluster_mono);
		eigen_vectors_ = current_cluster_pca.getEigenVectors();
		eigen_values_ = current_cluster_pca.getEigenValues();*/
	}

	valid_cluster_ = true;
	point_cloud_ = current_cluster;
}

/*std::vector<float> Cluster::getFpfhDescriptor(const unsigned int& in_ompnum_threads, const double& in_normal_search_radius, const double& in_fpfh_search_radius)
{
	std::vector<float> cluster_fpfh_histogram(33, 0.0);

	pcl::search::KdTree<pcl::PointXYZRGB>::Ptr norm_tree(new pcl::search::KdTree<pcl::PointXYZRGB>);
	if (point_cloud_->points.size() > 0)
	{
		norm_tree->setInputCloud(point_cloud_);
	}

	pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
	pcl::NormalEstimationOMP<pcl::PointXYZRGB, pcl::Normal> normal_estimation;
	normal_estimation.setNumberOfThreads(in_ompnum_threads);
	normal_estimation.setInputCloud(point_cloud_);
	normal_estimation.setSearchMethod(norm_tree);
	normal_estimation.setViewPoint(std::numeric_limits<float>::max(), std::numeric_limits<float>::max(), std::numeric_limits<float>::max());
	normal_estimation.setRadiusSearch(in_normal_search_radius);
	normal_estimation.compute(*normals);

	pcl::PointCloud<pcl::FPFHSignature33>::Ptr fpfh_histograms(new pcl::PointCloud<pcl::FPFHSignature33>());

	pcl::FPFHEstimationOMP<pcl::PointXYZRGB, pcl::Normal, pcl::FPFHSignature33> fpfh;
	fpfh.setNumberOfThreads(in_ompnum_threads);
	fpfh.setInputCloud(point_cloud_);
	fpfh.setInputNormals(normals);
	fpfh.setSearchMethod(norm_tree);
	fpfh.setRadiusSearch(in_fpfh_search_radius);
	fpfh.compute(*fpfh_histograms);

	float fpfh_max = std::numeric_limits<float>::min();
	float fpfh_min = std::numeric_limits<float>::max();

	for (unsigned int i = 0; i<fpfh_histograms->size(); i++) //for each point fpfh
	{
		for (unsigned int j = 0; j< cluster_fpfh_histogram.size(); j++)//sum each histogram's bin for all points, get min/max
		{
			cluster_fpfh_histogram[j] = cluster_fpfh_histogram[j] + fpfh_histograms->points[i].histogram[j];
			if (cluster_fpfh_histogram[j] < fpfh_min)
				fpfh_min = cluster_fpfh_histogram[j];
			if (cluster_fpfh_histogram[j] > fpfh_max)
				fpfh_max = cluster_fpfh_histogram[j];
		}

		float fpfh_dif = fpfh_max - fpfh_min;
		for (unsigned int j = 0; fpfh_dif > 0 && j < cluster_fpfh_histogram.size(); j++)//substract the min from each and normalize
		{
			cluster_fpfh_histogram[j] = (cluster_fpfh_histogram[j] - fpfh_min) / fpfh_dif;
		}
	}

	return cluster_fpfh_histogram;
}
*/
bool Cluster::isValid()
{
	return valid_cluster_;
}

//void Cluster::SetValidity(bool in_valid)
//{
//	valid_cluster_ = in_valid;
//}
//
//int Cluster::GetId()
//{
//	return id_;
//}

Cluster::~Cluster() {
	// TODO Auto-generated destructor stub
}