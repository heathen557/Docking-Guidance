//
// Created by luffy7n on 18-6-23.
//

#ifndef DOCKING_GUIDANCE_LIDAR_DETECT_H
#define DOCKING_GUIDANCE_LIDAR_DETECT_H

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
#include "pandar_grabber/pandar_grabber.h"

#include "framework.h"
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

#endif //DOCKING_GUIDANCE_LIDAR_DETECT_H
