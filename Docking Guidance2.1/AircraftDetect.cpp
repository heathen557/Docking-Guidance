//
// Created by luffy7n on 18-6-23.
//

//#include <sqltypes.h>
#include "AircraftDetect.h"
#include "GlobleData.h"
extern bool _run_flag;
extern struct control_msg con_msg;
//extern double _topPointX, _topPointY, _botPointX, _botPointY, _leftPointX, _leftPointY, _rightPointX, _rightPointY;
void AircraftDetect::setParameter()
{
    cloud_id_ = 0;
    distance_ = 50; 
    target_distance_ = -150; 
    mild_x_ = 0.0; 
    endline_y_ = 0.0; 
    x_precision_ = 0.0; 
    y_precision_ = 0.0; 
    //viewer_("Simple Cloud Viewer");
    cluster_size_min_ = 100;
    cluster_size_max_ = 750;
    remove_points_upto_ = 0.0;
    clip_min_height_ = -20;
    clip_max_height_ = 10;
    clip_right_position_ = -22;
    clip_left_position_ = 20;
    cluster_tolerance_ = 0.6;
    target_min_height_ = 2.0;
    target_max_height_ = 3.5;
    target_min_width_ = 2.5;
    target_max_width_ = 3.5;
    max_y_ = 100;
    mild_point0_ = {con_msg.mild_point0_x, con_msg.mild_point0_y};
    mild_point1_ = {con_msg.mild_point1_x, con_msg.mild_point1_y};
    end_point0_ = {con_msg.end_point0_x, con_msg.end_point0_y};
    end_point1_ = {con_msg.end_point1_x, con_msg.end_point1_y};
    mild_p0l_ = {con_msg.mild_point0_x+0.03f, con_msg.mild_point0_y, -6};
    mild_p1l_ = {con_msg.mild_point1_x+0.03f, con_msg.mild_point1_y, -6};
    end_p0u_ = {con_msg.end_point0_x, con_msg.end_point0_y-0.03f, -6};
    end_p1u_ = {con_msg.end_point1_x, con_msg.end_point1_y-0.03f, -6};
    mild_p0r_ = {con_msg.mild_point0_x-0.03f, con_msg.mild_point0_y, -6};
    mild_p1r_ = {con_msg.mild_point1_x-0.03f, con_msg.mild_point1_y, -6};
    end_p0d_ = {con_msg.end_point0_x, con_msg.end_point0_y+0.03f, -6};
    end_p1d_ = {con_msg.end_point1_x, con_msg.end_point1_y+0.03f, -6};
    mild_p0_ = {con_msg.mild_point0_x, con_msg.mild_point0_y, -6};
    mild_p1_ = {con_msg.mild_point1_x, con_msg.mild_point1_y, -6};
    end_p0_ = {con_msg.end_point0_x, con_msg.end_point0_y, -6};
    end_p1_ = {con_msg.end_point1_x, con_msg.end_point1_y, -6};
}

void AircraftDetect::initializeParameter()
{
    detectflag_ = 0;
    src_cloud_.reset(new Cloud);
    prev_cloud_.reset(new Cloud);  
    //find_target_.reset(new Cloud);
    remove_ground_ = true;
    use_region_growing_ = false;
    nose_y_ = 90;
    sum_wing_ = 0;
    sum_engine_ = 0;
    sum_detect_par_ = 0;
    succed_detect_counter_ = 0;
    succed_detect_ = false;
    succed_find_  = false;
    cloud_viewer_.reset(new pcl::visualization::PCLVisualizer("3D viewer"));
    cloud_viewer_->addLine(mild_p0_, mild_p1_, "mildline");
    cloud_viewer_->addLine(mild_p0l_, mild_p1l_, "mildline1");
    cloud_viewer_->addLine(mild_p0r_, mild_p1r_, "mildline2");
    cloud_viewer_->addLine(end_p0_, end_p1_, "endline");
    cloud_viewer_->addLine(end_p0u_, end_p1u_, "endline1");
    cloud_viewer_->addLine(end_p0d_, end_p1d_, "endline2");
}

void AircraftDetect::loadAircraftData(const char *type_file, const char *aircraft_data) //后可考虑模仿svm.load();
{
    FILE *fp = fopen((type_file), "r");
    int n = 0;
    while (!feof(fp))
    {
        fscanf(fp, "%s\n", type_dict_[n++]);
    }
    fclose(fp);
    ifstream fin(aircraft_data);
    float vec_point;
    std::vector<float> vec;
    std::string line;
    vec_dict_.clear();
    while (!fin.eof())
    {
        std::getline(fin, line);
        std::stringstream stringin(line);
        vec.clear();
        while (stringin >> vec_point)
        {
            vec.push_back(vec_point);
        }
        vec_dict_.push_back(vec);
    }
}


AircraftDetect::AircraftDetect(std::string &calibFile, std::string &pcapFile, const char *type_file, const char *aircraft_data, pcl::visualization::PointCloudColorHandler<pcl::PointXYZI> &handler) //考虑如何把handler放在类内实现
: calibrationFile_(calibFile)
, pcapFile_(pcapFile)
//, viewer_("Simple Cloud Viewer")
//, cloud_viewer_ (new pcl::visualization::PCLVisualizer("3D viewer"))
, handler_(handler)
, interface_(calibrationFile_, pcapFile_)
{
    setParameter();
    loadAircraftData(type_file, aircraft_data);
    //cv::generateColors(colors_, 100);
    initializeParameter();

}

AircraftDetect::AircraftDetect(std::string &calibFile, const char *type_file, const char *aircraft_data, std::string &ip, int port, pcl::visualization::PointCloudColorHandler<pcl::PointXYZI> &handler)
: calibrationFile_(calibFile)
//, cloud_viewer_ (new pcl::visualization::PCLVisualizer("3D viewer"))
, handler_(handler)
, ip_(ip)
, port_(port)
//, viewer_("Simple Cloud Viewer")
, interface_(calibrationFile_, pcapFile_)
{
    setParameter();
    loadAircraftData(type_file, aircraft_data);
    //cv::generateColors(colors_, 100);
    initializeParameter();
}

pcl::PointXYZ AircraftDetect::findMaxValues(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud) 
{
    pcl::PointXYZ point;
    point.x = cloud->points[0].x;
    point.y = cloud->points[0].y;
    point.z = cloud->points[0].z;

    for (size_t i = 1; i < cloud->points.size(); ++i)
    {
        if (cloud->points[i].z > point.z)
            point.z = cloud->points[i].z;
        if (cloud->points[i].y > point.y)
            point.y = cloud->points[i].y;
        if (cloud->points[i].x > point.x)
            point.x = cloud->points[i].x;
    }

    return point;
}

pcl::PointXYZ AircraftDetect::findMinValues(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud)
{
    pcl::PointXYZ point;
    point.x = cloud->points[0].x;
    point.y = cloud->points[0].y;
    point.z = cloud->points[0].z;

    for (size_t i = 1; i < cloud->points.size(); ++i)
    {
        if (cloud->points[i].z < point.z)
            point.z = cloud->points[i].z;
        if (cloud->points[i].y < point.y)
            point.y = cloud->points[i].y;
        if (cloud->points[i].x < point.x)
            point.x = cloud->points[i].x;
    }

    return point;
}

void AircraftDetect::findTarget(const CloudConstPtr &in_cloud_ptr, CloudPtr out_cloud_ptr, PointType centre, float radius)
{
    pcl::KdTreeFLANN<pcl::PointXYZI> kdtree;
    std::vector<int> pointIdxRadiusSearch;
    std::vector<float> pointRadiusSquareDistance;
    kdtree.setInputCloud(in_cloud_ptr);
    kdtree.radiusSearch(centre, radius, pointIdxRadiusSearch, pointRadiusSquareDistance);
    out_cloud_ptr->width = pointIdxRadiusSearch.size();
    out_cloud_ptr->height = 1;
    out_cloud_ptr->points.resize(out_cloud_ptr->width * out_cloud_ptr->height);
    for (int i = 0; i < pointIdxRadiusSearch.size(); ++i)
    {
        out_cloud_ptr->points[i].x = in_cloud_ptr->points[pointIdxRadiusSearch[i]].x;
        out_cloud_ptr->points[i].y = in_cloud_ptr->points[pointIdxRadiusSearch[i]].y;
        out_cloud_ptr->points[i].z = in_cloud_ptr->points[pointIdxRadiusSearch[i]].z;
        out_cloud_ptr->points[i].intensity = in_cloud_ptr->points[pointIdxRadiusSearch[i]].intensity;
    }
}

void AircraftDetect::setTargetColor(const CloudConstPtr &in_cloud_ptr, std::vector<int> &in_cluster_indices,
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

void AircraftDetect::clipCloud(const CloudConstPtr &in_cloud_ptr, CloudPtr out_cloud_ptr, float min_height,
                               float max_height, float left_position, float right_position)
{
    out_cloud_ptr->points.clear(); 
    for (size_t i = 0; i < in_cloud_ptr->points.size(); i++)
    {
        if ( in_cloud_ptr->points[i].z < max_height &&
             in_cloud_ptr->points[i].x < left_position &&
             in_cloud_ptr->points[i].x > right_position)
        {
            out_cloud_ptr->points.push_back(in_cloud_ptr->points[i]);
        }
    }
}

void AircraftDetect::removeFloor(const CloudPtr &in_cloud_ptr, CloudPtr out_nofloor_cloud_ptr,
                                 CloudPtr out_onlyfloor_cloud_ptr, float in_max_height, float in_floor_max_angle)
{
    pcl::SACSegmentation<pcl::PointXYZI> seg;
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);

    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_PERPENDICULAR_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
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

std::vector<pcl::PointIndices> AircraftDetect::regiongrowingSegmentation(const CloudPtr &in_cloud_ptr)
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

std::vector<pcl::PointIndices> AircraftDetect::euclideanCluster(const CloudPtr &in_cloud_ptr, int cluster_size_min,
                                                                int cluster_size_max, float cluster_tolerance)
{
    pcl::PointCloud<pcl::PointXYZI>::Ptr cluster_cloud(new pcl::PointCloud<pcl::PointXYZI>);
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

    //std::cout << "The " << cloud_id_ << " frame's number of clusters is equal to " << cluster_indices.size() << std::endl;

    //For every cluster...

    return cluster_indices;
}

void AircraftDetect::passthroughCloud(const CloudPtr &in_cloud_ptr, CloudPtr out_cloud_ptr, float min_limit,
                                      float max_limit, std::string &field_name, bool select) 
{
    pcl::PassThrough<PointType> pass;
    pass.setFilterFieldName(field_name);
    pass.setFilterLimits(min_limit, max_limit); 
    pass.setFilterLimitsNegative(select); 
    pass.setInputCloud(in_cloud_ptr);
    pass.filter(*out_cloud_ptr);
}

bool AircraftDetect::detectCircle(const CloudPtr &in_cloud_ptr, float min_radius, float max_radius,
                                  Eigen::VectorXf *coefficients, CloudPtr out_cloud_ptr) 
{
    std::vector<int> inliers_indicies;
    pcl::SampleConsensusModelCircle3D<PointType>::Ptr model_circle(
            new pcl::SampleConsensusModelCircle3D<PointType>(in_cloud_ptr));
    pcl::RandomSampleConsensus<PointType>ransac_sphere(model_circle);
    ransac_sphere.setDistanceThreshold(0.1);
    model_circle->setRadiusLimits(min_radius, max_radius);
    ransac_sphere.computeModel();
    inliers_indicies.clear();
    ransac_sphere.getInliers(inliers_indicies);
    if (inliers_indicies.size() == 0)
    {
        std::cout << "Can not detect circle" << std::endl;
        return false;
    }
    else
    {
        ransac_sphere.getModelCoefficients(*coefficients);
        pcl::copyPointCloud<PointType>(*in_cloud_ptr, inliers_indicies, *out_cloud_ptr);
        return true;
    }
}

std::vector<ClusterPtr> AircraftDetect::getCloudFromCluster(const CloudPtr &in_cloud_ptr, std::vector<pcl::PointIndices> cluster_indices)
{
    int cluster_id = 0;
    std::vector<ClusterPtr> clusters;
    //std::vector<ClusterPtr> target;
    for (std::vector<pcl::PointIndices>::iterator i = cluster_indices.begin(); i != cluster_indices.end(); ++i)
    {
        ClusterPtr cluster(new Cluster());
        cluster->setCloud(in_cloud_ptr, i->indices, cluster_id, 255, 0, 0);
        if (abs(cluster->getMaxPoint().y) > 50)
        {
            clusters.push_back(cluster);
        }
        cluster_id++;
    }
    if (clusters.size() != 0)
    {
        succed_find_ = true;
    }
    else
    {
        succed_find_ = false;
    }
    return clusters;
}

std::vector<ClusterPtr> AircraftDetect::getCloudFromCluster(const CloudPtr &in_cloud_ptr,
                                                            std::vector<pcl::PointIndices> cluster_indices,
                                                            pcl::PointCloud<pcl::PointXYZRGB>::Ptr drawed_cloud,
                                                            float target_min_height, float target_max_height,
                                                            float target_min_width, float target_max_width)
{
    //char fname[50];
    int cluster_id = 0;
    //std::vector<Cloud> clustered_clouds;
    std::vector<ClusterPtr> clusters;
    std::vector<ClusterPtr> target;
    //pcl::PointCloud<pcl::PointXYZRGB>::Ptr drawed_cloud;
    for (std::vector<pcl::PointIndices>::iterator i = cluster_indices.begin(); i != cluster_indices.end(); ++i) 
    {
        ClusterPtr cluster(new Cluster());
        cluster->setCloud(in_cloud_ptr, i->indices, cluster_id, 255, 0, 0); 
        //CreateInputFile(cluster);   //for train

        
        


        if (abs(cluster->getMaxPoint().y) > 30 && cluster->getLength() > 3.0 && cluster->getLength() < 4.0) 
        {
            setTargetColor(in_cloud_ptr, i->indices, drawed_cloud); 
            target.push_back(cluster);
            float x = cluster->getCentroid().x;
            float y = cluster->getCentroid().y;
            ncen_x_ = x;  
            ncen_y_ = y;
            float nmax_y = cluster->getMaxPoint().y; 
            //nose_y_ = ncen_y_ + 20;
            nose_y_ = nmax_y;
            //float y = abs(cluster->getCentroid().y);
            //float z = abs(cluster->getCentroid().z);
            std::cout << "x coordinate is :" << x << std::endl;
            std::cout << "y coordinate is :" << y << std::endl;
            
        }
        clusters.push_back(cluster);
        cluster_id++;
    }
    if (target.size()!= 0)
    {
        succed_detect_ = true;
        succed_detect_counter_++;
    }
    else
    {
        succed_detect_ = false;
    }
    return target;  
}

std::vector<ClusterPtr> AircraftDetect::detectTarget(const CloudConstPtr &in_cloud_ptr)
{
    CloudPtr removed_points_cloud(new Cloud);
    CloudPtr clipped_cloud(new Cloud);
    nofloor_cloud_.reset(new Cloud);
    CloudPtr onlyfloor_cloud(new Cloud);
    std::vector<pcl::PointIndices> cluster_indices;
    std::vector<ClusterPtr> target;
    double start = pcl::getTime();
    
    clipCloud(in_cloud_ptr, clipped_cloud, clip_min_height_, clip_max_height_, clip_left_position_, clip_right_position_);
    std::cout << clipped_cloud->size() << std::endl;
    if (remove_ground_)
    {
        removeFloor(clipped_cloud, nofloor_cloud_, onlyfloor_cloud);
    }
    else
    {
        nofloor_cloud_ = clipped_cloud;
    }
    cluster_indices.clear();
    if (use_region_growing_)
    {
        cluster_indices = regiongrowingSegmentation(nofloor_cloud_);
    }
    else
    {
        cluster_indices = euclideanCluster(nofloor_cloud_, cluster_size_min_, cluster_size_max_, cluster_tolerance_);
    }
    target.clear();
    drawed_cloud_.reset(new pcl::PointCloud<pcl::PointXYZRGB>);
    target = getCloudFromCluster(nofloor_cloud_, cluster_indices, drawed_cloud_, target_min_height_, target_max_height_, target_min_width_, target_max_width_);//后考虑此函数写到此再processCloud里接着getCluster
    double end = pcl::getTime();
    double process_time = end - start;
    //std::cout << "It takes " << process_time << "s to process the frame" << std::endl;
    return target;
}

std::vector<ClusterPtr> AircraftDetect::detectAircraft(const CloudConstPtr &in_cloud_ptr)
{
    CloudPtr cloud(new Cloud);
    CloudPtr passthrough_cloud(new Cloud);
    nofloor_cloud2_.reset(new Cloud); 
    CloudPtr onlyfloor_cloud(new Cloud);
    std::vector<pcl::PointIndices> cluster_indices;
    std::vector<ClusterPtr> aircraft;
    std::string field_name = "y";
    pcl::copyPointCloud(*in_cloud_ptr, *cloud);
    passthroughCloud(cloud, passthrough_cloud, -150, -50, field_name, false);
    removeFloor(passthrough_cloud, nofloor_cloud2_, onlyfloor_cloud);
    cluster_indices.clear();
    //cluster_indices = regiongrowingSegmentation(nofloor_cloud2_);
    cluster_indices = euclideanCluster(nofloor_cloud2_, 100, 2000, 0.8);
    aircraft.clear();
    aircraft = getCloudFromCluster(nofloor_cloud2_, cluster_indices);
    return aircraft;
}

bool AircraftDetect::detectAircraftParameter(const CloudPtr &in_cloud_ptr, float *wing_span, float *engine_distance)
{
    double min_radius = 0.9, max_radius = 1.0;
    pcl::PointXYZ min_point = findMinValues(in_cloud_ptr);
    pcl::PointXYZ max_point = findMaxValues(in_cloud_ptr);
    *wing_span = max_point.x - min_point.x; 
    CloudPtr left_cloud(new Cloud);
    CloudPtr right_cloud(new Cloud);
    CloudPtr left_circle(new Cloud); 
    CloudPtr right_circle(new Cloud);
    CloudPtr circle(new Cloud);
    std::string field_name = "x";
    passthroughCloud(in_cloud_ptr, left_cloud, (ncen_x_+2), max_point.x, field_name, false); 
    passthroughCloud(in_cloud_ptr, right_cloud, min_point.x, (ncen_x_-2), field_name, false);
    Eigen::VectorXf left_circle_coefficients, right_circle_coefficients;
    if (detectCircle(left_cloud, min_radius, max_radius, &left_circle_coefficients, left_circle)
        && detectCircle(right_cloud, min_radius, max_radius, &right_circle_coefficients, right_circle))
    {
        *engine_distance = left_circle_coefficients(0) - right_circle_coefficients(0);
        return true;
    }

}

char* AircraftDetect::recognizeType(std::vector<float> feature)
{
    float min_distance = calculateSimilarity(feature, vec_dict_[0]);
    int min_index = 0;
    for (int i = 1; i < vec_dict_.size()-1; ++i)
    {
        float sim_d = calculateSimilarity(feature, vec_dict_[i]);
        if (sim_d < min_distance)
        {
            min_distance = sim_d;
            min_index = i;
        }
    }
    return type_dict_[min_index];
}



void AircraftDetect::processCloud(const CloudConstPtr &cloud)
{
    std::vector<float> feature;
    char fname[100];
    cloud_id_++;
    if (cloud_id_ >= 2 && cloud_id_%1 == 0)
    {
        if (prev_cloud_->empty())  
        {
            pcl::copyPointCloud(*cloud, *prev_cloud_);
        }
        else
        {
            target_ = detectTarget(cloud);
            if (succed_detect_)
            {
                detectflag_ = 1;
                //std::cout << cloud_id_ << std::endl;
                if (succed_detect_counter_ == 1)
                {
                    //cout << "length size: " << length_.size() << endl;
                    aircraft_length_ = calculateLength(length_);
                    cout << "aircraft length is: " << aircraft_length_ << endl;
                }
                float moving_distance = abs(pre_distance_ - target_[0]->getCentroid().y);
                double time = cloud->header.stamp - pre_time_;
                //double velocity_ = 1000000 * moving_distance / time;
                pre_time_ = cloud->header.stamp;
                pre_distance_ = target_[0]->getCentroid().y;
                distance_ = target_[0]->getCentroid().y;
                target_distance_ = target_[0]->getMaxPoint().y;
                aircraft_location_.push_back(distance_);
                if (aircraft_location_.size() == 5)
                {
                    velocity_ = calculateVelocity(aircraft_location_);
                    cout << "velocity: " << velocity_ << endl;
                    std::vector<double>::iterator First_Ele = aircraft_location_.begin();
                    aircraft_location_.erase(First_Ele);
                }
                Input_Num cen = {ncen_x_, ncen_y_}; 
//                POSINFO status = Plane_Straight_line(mild_point0_, mild_point1_, cen);
                position_ = status.position;
                offset_ = status.offset;
                
                aircraft_cloud_.reset(new Cloud);
                std::string field_name = "y";
                passthroughCloud(nofloor_cloud_, aircraft_cloud_, (nose_y_+0.5-35), (nose_y_+0.5), field_name, false); 
                if ((abs(nose_y_) <=  55 && abs(nose_y_) >= 50) )
                {
                    float wing_span = 0;
                    float engine_distance = 0;
                    if (detectAircraftParameter(aircraft_cloud_, &wing_span, &engine_distance)); 
                    {
                        
                        sum_wing_ += wing_span;
                        sum_engine_ += engine_distance;
                        sum_detect_par_++;
                        if (abs(nose_y_) >= 50 && abs(nose_y_) <= 55) 
                        {
                            feature.clear();
                            wing_span_ = sum_wing_ / sum_detect_par_;
                            engine_distance_ = sum_engine_ / sum_detect_par_;
                            std::cout << "wing_span : " << wing_span_ << "m" << std::endl;
                            std::cout << "engine_distance : " << engine_distance_ << "m" << std::endl;
                            feature.push_back(wing_span_);
                            feature.push_back(engine_distance_);
                            feature.push_back(aircraft_length_);
                            type_ = recognizeType(feature);
                        }
                    }

                }

            }
            else
            {
                aircraft_ = detectAircraft(cloud);
                if (succed_find_)
                {
                    type_ = "Aircraft";
                    find_target_.reset(new Cloud);
                    float aircaft_distance = aircraft_[0]->getCentroid().y;
                    target_distance_ = aircaft_distance;
                    PointType cluster_centre;
                    cluster_centre.x = aircraft_[0]->getCentroid().x;
                    cluster_centre.y = aircraft_[0]->getCentroid().y;
                    cluster_centre.z = aircraft_[0]->getCentroid().z;
                    float radius = 15.0f;
                    findTarget(nofloor_cloud2_, find_target_, cluster_centre, radius);
                    pcl::PointXYZ min_point = findMinValues(find_target_);
                    pcl::PointXYZ max_point = findMaxValues(find_target_);
                    float aircraft_length = sqrt((max_point.x - min_point.x) * (max_point.x - min_point.x) +
                                                 (max_point.y - min_point.y) * (max_point.y - min_point.y));
                    length_.push_back(aircraft_length);
                }
                else
                {
                    //std::cout << "missing..." << std::endl;
                    sprintf(fname, "./%s/%05d.pcd", "LOST-FRAME", cloud_id_); 
                    pcl::io::savePCDFile(fname, *cloud);
                }
            }
            pcl::copyPointCloud(*cloud, *prev_cloud_);
        }
    }
    else
    {
        printf(".");
    }
}


void AircraftDetect::rawCloud(const boost::shared_ptr<const pcl::PointCloud<pcl::PointXYZI> > &cloud)
{
    //this->viewer_.showCloud(cloud);
    pcl::copyPointCloud(*cloud, *src_cloud_);
    processCloud(cloud);
    //sleep(1);
    _mtx.lock();
    _READYTOSENDFLAG = 1;
    _displayinfo.craft = type_;//pedestrian
    _displayinfo.distance = (double)(-target_distance_); //(double)abs(distance_);
    _displayinfo.speed = velocity_; //velocity_;
    _displayinfo.position = position_; //position_; 
    _displayinfo.offset = offset_; //offset_;
    _displayinfo.detectflag = detectflag_;
    _mtx.unlock();
}

void AircraftDetect::run()
{
    boost::function<void(const boost::shared_ptr<const Cloud> &)> f1 =
            boost::bind(&AircraftDetect::rawCloud, this, _1);
    boost::signals2::connection c1 = interface_.registerCallback(f1);
    // start receving point clouds
    interface_.start();
    std::cout << "<Esc>, \'q\', \'Q\': quit the program" << std::endl; 
    while (!cloud_viewer_->wasStopped())
    {
        if (aircraft_cloud_)
        {
            handler_.setInputCloud(aircraft_cloud_);
            if (!cloud_viewer_->updatePointCloud(aircraft_cloud_, handler_))
            {
                cloud_viewer_->addPointCloud<pcl::PointXYZI>(aircraft_cloud_, handler_);
            }
        }
        else
        {
            if (find_target_)
            {
                handler_.setInputCloud(find_target_);
                if (!cloud_viewer_->updatePointCloud(find_target_, handler_))
                {
                    cloud_viewer_->addPointCloud<PointType>(find_target_, handler_);
                }
            }
            else
            {
                if (src_cloud_)
                {
                    handler_.setInputCloud(src_cloud_);
                    if (!cloud_viewer_->updatePointCloud(src_cloud_, handler_))
                    {
                        cloud_viewer_->addPointCloud<pcl::PointXYZI>(src_cloud_, handler_);
                    }
                }
            }
        }
        cloud_viewer_->spinOnce();
        if (!interface_.isRunning())
        {
            cloud_viewer_->spin();
        }
        boost::this_thread::sleep(boost::posix_time::microseconds(1000));
        if (_run_flag == false)
        {
            break;
        }
    }
    _run_flag = false;
    cout<<"air read finish"<<endl;
    interface_.stop();
    c1.disconnect();
}

AircraftDetect::~AircraftDetect()
{
    // TODO Auto-generated destructor stub
}