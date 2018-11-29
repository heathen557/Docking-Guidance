//
// Created by luffy7n on 18-10-30.
//
#include "ProcessPointcloud.h"
float calculatePointDistance(PointType p1, pcl::PointXYZ p2)
{
    float x_d = p1.x - p2.x;
    float y_d = p1.y - p2.y;
    float z_d = p1.z - p2.z;
    float distance = sqrtf(x_d*x_d + y_d*y_d +z_d*z_d);
    return distance;
}

void transformPointCloud(const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr &in_cloud_ptr, CloudPtr out_cloud_ptr)
{
    out_cloud_ptr->width = in_cloud_ptr->size();
    out_cloud_ptr->height = 1;
    out_cloud_ptr->resize(out_cloud_ptr->width * out_cloud_ptr->height);
    for (int i = 0; i < in_cloud_ptr->size(); ++i)
    {
        out_cloud_ptr->points[i].x = in_cloud_ptr->points[i].x;
        out_cloud_ptr->points[i].y = in_cloud_ptr->points[i].y;
        out_cloud_ptr->points[i].z = in_cloud_ptr->points[i].z;
    }
}

void setTargetColor(const CloudConstPtr &in_cloud_ptr, std::vector<int> &in_cluster_indices,
                    pcl::PointCloud<pcl::PointXYZRGB>::Ptr out_cloud_ptr)
{
    for (int i = 0; i < in_cloud_ptr->size(); ++i)
    {
        pcl::PointXYZRGB p;
        p.x = in_cloud_ptr->points[i].x;
        p.y = in_cloud_ptr->points[i].y;
        p.z = in_cloud_ptr->points[i].z;
        std::vector<int>::iterator iter = std::find(in_cluster_indices.begin(), in_cluster_indices.end(), i);
        if (iter == in_cluster_indices.end())
        {
            p.r = 250;
            p.g = 250;
            p.b = 250;
        }
        else
        {
            p.r = 255;
            p.g = 0;
            p.b = 0;
        }
        out_cloud_ptr->push_back(p);
    }
}

void removePointsUpto(const CloudConstPtr &in_cloud_ptr, CloudPtr out_cloud_ptr,
                      const double remove_points_upto)
{
    out_cloud_ptr->points.clear();
    for (size_t i = 0; i < in_cloud_ptr->points.size(); i++)
    {
        double orign_distance = sqrt(pow(in_cloud_ptr->points[i].x, 2) + pow(in_cloud_ptr->points[i].y, 2));
        if (orign_distance > remove_points_upto)
        {
            out_cloud_ptr->points.push_back(in_cloud_ptr->points[i]);
        }
    }
}

void clipCloud(const CloudConstPtr &in_cloud_ptr, CloudPtr out_cloud_ptr, float min_height,
                         float max_height, float right_pos, float left_pos)
{
    out_cloud_ptr->points.clear();
    for (size_t i = 0; i < in_cloud_ptr->points.size(); i++)
    {
        if (in_cloud_ptr->points[i].z > min_height && in_cloud_ptr->points[i].z < max_height && in_cloud_ptr->points[i].x > right_pos && in_cloud_ptr->points[i].x < left_pos)//
        {
            out_cloud_ptr->points.push_back(in_cloud_ptr->points[i]);
        }
    }
}

void removeFloor(const CloudPtr &in_cloud_ptr, CloudPtr out_nofloor_cloud_ptr,
                           CloudPtr out_onlyfloor_cloud_ptr, float in_max_height, float in_floor_max_angle)
{
    pcl::SACSegmentation<pcl::PointXYZI> seg;
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);

    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_PLANE);//SACMODEL_PERPENDICULAR_PLANE
    seg.setMethodType(pcl::SAC_RANSAC);
    //seg.setDistanceThreshold(0.1);
    seg.setMaxIterations(100);
    seg.setAxis(Eigen::Vector3f(0, 0, 1));
    seg.setEpsAngle(in_floor_max_angle);

    seg.setDistanceThreshold(in_max_height);
    seg.setOptimizeCoefficients(true);
    seg.setInputCloud(in_cloud_ptr);
    seg.segment(*inliers, *coefficients);

    if (inliers->indices.size() == 0)
    {
        std::cout << "Could not estimate a planar model for the given dataset." << std::endl;
    }

    //REMOVE THE FLOOR FROM THE CLOUD
    pcl::ExtractIndices<pcl::PointXYZI> extract;
    extract.setInputCloud(in_cloud_ptr);
    extract.setIndices(inliers);
    extract.setNegative(true);//true removes the indices, false leaves only the indices
    extract.filter(*out_nofloor_cloud_ptr);

    //EXTRACT THE FLOOR FROM THE CLOUD
    extract.setNegative(false);//true removes the indices, false leaves only the indices
    extract.filter(*out_onlyfloor_cloud_ptr);
}

vector<pcl::PointIndices> regiongrowingSegmentation(const CloudPtr &in_cloud_ptr)
{
    pcl::PointCloud<pcl::PointXYZI>::Ptr clustered_cloud(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::search::Search<pcl::PointXYZI>::Ptr tree = boost::shared_ptr<pcl::search::Search<pcl::PointXYZI>>(new pcl::search::KdTree<pcl::PointXYZI>);
    pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
    std::vector<pcl::PointIndices> cluster_indices;

    pcl::NormalEstimation<pcl::PointXYZI, pcl::Normal> normal_estimator;
    normal_estimator.setSearchMethod(tree);
    normal_estimator.setInputCloud(in_cloud_ptr);
    normal_estimator.setKSearch(10); //50
    normal_estimator.compute(*normals);

    pcl::RegionGrowing<pcl::PointXYZI, pcl::Normal> reg;
    reg.setMinClusterSize(30); //100
    reg.setMaxClusterSize(60); //1000
    reg.setSearchMethod(tree);
    reg.setNumberOfNeighbours(10);
    reg.setInputCloud(in_cloud_ptr);
    reg.setInputNormals(normals);
    reg.setSmoothnessThreshold(1.5 * M_PI);
    reg.setCurvatureThreshold(0.5);
    reg.extract(cluster_indices);

    //std::cout << "Number of clusters is equal to " << cluster_indices.size() << std::endl;

    return cluster_indices;
}

vector<pcl::PointIndices> euclideanCluster(const CloudPtr &in_cloud_ptr, int cluster_size_min,
                                                          int cluster_size_max, float cluster_tolerance)
{
    //pcl::PointCloud<pcl::PointXYZI>::Ptr cluster_cloud(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::search::KdTree<pcl::PointXYZI>::Ptr kdtree(new pcl::search::KdTree<pcl::PointXYZI>);
    kdtree->setInputCloud(in_cloud_ptr);
    std::vector<pcl::PointIndices> cluster_indices;

    pcl::EuclideanClusterExtraction<pcl::PointXYZI> ec;
    ec.setClusterTolerance(cluster_tolerance); //2*voxel_size 0.2
    ec.setMinClusterSize(cluster_size_min);
    ec.setMaxClusterSize(cluster_size_max);
    ec.setSearchMethod(kdtree);
    ec.setInputCloud(in_cloud_ptr);
    ec.extract(cluster_indices);
    //int size = cluster_indices.size();

    //std::cout << "The " << cloud_id_ << " frame's number of clusters is equal to " << cluster_indices.size() << std::endl;

    //For every cluster...

    return cluster_indices;
}

bool findTarget(const CloudPtr &in_cloud_ptr, PointType centre, std::vector<int> *pointIdxRadiusSearch)
{
    pcl::KdTreeFLANN<pcl::PointXYZI> kdtree;
    std::vector<float> pointRadiusSquareDistance;
    float radius = 1.2;// 可根据速度设定
    kdtree.setInputCloud(in_cloud_ptr); //确定？
    kdtree.radiusSearch(centre, radius, *pointIdxRadiusSearch, pointRadiusSquareDistance);
    if (pointIdxRadiusSearch->size() > 0)
    {
        std::cout << "search: " << pointIdxRadiusSearch->size() << std::endl;
        return true;
    }
    else
    {
        return false;
    }
}

bool getCloudFromCluster(const CloudPtr &in_cloud_ptr, vector<pcl::PointIndices> cluster_indices,
                                                      float target_min_height, float target_max_height,
                                                      float target_min_length, float target_max_length,
                                                      vector<ClusterPtr> *target)//std::vector<int> *target_indices)  //默认参数变量如何写
{
    int cluster_id = 0;
    //std::vector<Cloud> clustered_clouds;
    //std::vector<ClusterPtr> clusters;
    //std::vector<ClusterPtr> target;
    CloudPtr cluster_cloud(new Cloud);
    CloudPtr all_cluster_cloud(new  Cloud);
    //pcl::PointCloud<pcl::PointXYZRGB>::Ptr drawed_cloud;
    for (std::vector<pcl::PointIndices>::iterator i = cluster_indices.begin(); i != cluster_indices.end(); ++i)
    {
        ClusterPtr cluster(new Cluster());
        cluster->setCloud(in_cloud_ptr, i->indices, cluster_id, 255, 0, 0); //, (int)colors_[cluster_id].val[0], (int)colors_[cluster_id].val[1], (int)colors_[cluster_id].val[2]);
        transformPointCloud(cluster->getCloud(), cluster_cloud);
        *all_cluster_cloud += *cluster_cloud;
        cluster_cloud->clear();
        //CreateInputFile(cluster);   //for train
        if (cluster->getHeight() > target_min_height && cluster->getHeight() < target_max_height && cluster->getLength() > target_min_length & cluster->getLength() < target_max_length)// && (0.7,0.3)
        {
            //*target_indices = i->indices;
            target->push_back(cluster);
            float x = cluster->getCentroid().x;
            float y = abs(cluster->getCentroid().y);
            float z = abs(cluster->getCentroid().z);
            //std::cout << "frame " << cloud_id_ << " x coordinate is: " << x << std::endl;
            std::cout << "The coordinate of target is (" << x << " " << y << " " << z << ")" << std::endl;
        }
        //clusters.push_back(cluster);
        cluster_id++;
    }
    if (target->size() != 0)
    {
        cout << "Number of targets is " << target->size() << endl;
        //all_cluster_cloud_ = all_cluster_cloud;
        return true;
    }
    else
    {
        //cout << cloud_id_ <<"th Frame has None Target :" << endl;
        return false;
    }
}

