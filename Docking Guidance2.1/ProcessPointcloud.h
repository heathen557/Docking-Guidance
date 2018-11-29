//
// Created by luffy7n on 18-10-30.
//

#ifndef DOCKING_GUIDANCE2_PROCESSPOINTCLOUD_H
#define DOCKING_GUIDANCE2_PROCESSPOINTCLOUD_H

#include <pcl/console/time.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/console/parse.h>
#include <pcl/common/time.h>
#include <pcl/search/kdtree.h>

#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/features/normal_3d.h>
#include <pcl/segmentation/region_growing.h>
#include <pcl/filters/passthrough.h>
#include <pcl/sample_consensus/sac_model_circle3d.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/features/fpfh.h>
#include <pcl/registration/ia_ransac.h>
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/registration/correspondence_estimation.h>
#include <pcl/registration/correspondence_rejection_features.h>
#include <pcl/registration/correspondence_rejection_sample_consensus.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/approximate_voxel_grid.h>


#include "Cluster.h"
#include "tools_function.h"

typedef pcl::PointXYZI PointType;
typedef pcl::PointCloud<PointType> Cloud;
typedef Cloud::Ptr CloudPtr;
typedef Cloud::ConstPtr CloudConstPtr;
typedef pcl::search::KdTree<PointType> KdTree;
typedef KdTree::Ptr KdtreePtr;
typedef pcl::PointCloud<pcl::Normal> PointNormal;
typedef pcl::PointCloud<pcl::FPFHSignature33> FpfhFeature;
typedef FpfhFeature::Ptr FpfhFeaturePtr;

float calculatePointDistance(PointType p1, pcl::PointXYZ p2); //参考pcl里是如何使用PointT来包含多个种类的
void transformPointCloud(const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr &in_cloud_ptr, CloudPtr out_cloud_ptr);
void passthroughCloud(const CloudPtr &in_cloud_ptr, CloudPtr out_cloud_ptr, float min_limit, float max_limit,
                      std::string &field_name, bool select);
void setTargetColor(const CloudConstPtr &in_cloud_ptr, std::vector<int> &in_cluster_indices,
                              pcl::PointCloud<pcl::PointXYZRGB>::Ptr out_cloud_ptr);
void removePointsUpto(const CloudConstPtr &in_cloud_ptr, CloudPtr out_cloud_ptr,
                                const double remove_points_upto);
void clipCloud(const CloudConstPtr &in_cloud_ptr, CloudPtr out_cloud_ptr, float min_height,
                         float max_height, float right_pos, float left_pos);
void removeFloor(const CloudPtr &in_cloud_ptr, CloudPtr out_nofloor_cloud_ptr,
                           CloudPtr out_onlyfloor_cloud_ptr, float in_max_height = 0.2, float in_floor_max_angle = 0.1);
vector<pcl::PointIndices> regiongrowingSegmentation(const CloudPtr &in_cloud_ptr);
vector<pcl::PointIndices> euclideanCluster(const CloudPtr &in_cloud_ptr, int cluster_size_min,
                                                          int cluster_size_max, float cluster_tolerance);
//std::vector<float> extractClusterFeature(ClusterPtr cluster);
//FpfhFeaturePtr computeFpfhFeature(const CloudPtr &in_cloud_ptr, KdtreePtr tree);
//float calculateSimilarity2(ClusterPtr cluster, CloudPtr model_cloud);
bool detectCircle(const CloudPtr &in_cloud_ptr, float min_radius, float max_radius,
                  Eigen::VectorXf *coefficients, CloudPtr out_cloud_ptr);
bool findTarget(const CloudPtr &in_cloud_ptr, PointType centre, std::vector<int> *pointIdxRadiusSearch);
bool getCloudFromCluster(const CloudPtr &in_cloud_ptr, vector<pcl::PointIndices> cluster_indices,
                                                      float target_min_height, float target_max_height,
                                                      float target_max_length, float target_min_length,
                                                      vector<ClusterPtr> *target); //, vector<int> *target_indices);

#endif //DOCKING_GUIDANCE2_PROCESSPOINTCLOUD_H

