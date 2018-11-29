//
// Created by luffy7n on 18-6-23.
//

#ifndef DOCKING_GUIDANCE_AIRCRAFTDETECT_H
#define DOCKING_GUIDANCE_AIRCRAFTDETECT_H

#include "lidar_detect.h"



class AircraftDetect
{
    int port_,cloud_id_;
    int detectflag_;
    int cluster_size_min_, cluster_size_max_, aircraft_cloud_size_;
    float remove_points_upto_, clip_min_height_, clip_max_height_, clip_left_position_, clip_right_position_, cluster_tolerance_,
            target_min_height_, target_max_height_, target_min_width_, target_max_width_, max_y_;
    float nose_y_, ncen_x_, ncen_y_;
    float sum_wing_, sum_engine_;
    int sum_detect_par_;
    int succed_detect_counter_;
    float wing_span_, engine_distance_, aircraft_length_;
    double pre_time_;
    double pre_distance_;
    double mild_x_, endline_y_;
    double x_precision_, y_precision_;
    double distance_;
    double target_distance_;
    double velocity_;
    double offset_;
    std::string position_;
    std::string type_;
    bool remove_ground_;
    bool use_region_growing_;
    std::string calibrationFile_, pcapFile_, ip_;
    bool succed_detect_;
    bool succed_find_;
    char type_dict_[20][15];
    Input_Num mild_point0_, mild_point1_, end_point0_, end_point1_;
    pcl::PointXYZ mild_p0_, mild_p1_, end_p0_, end_p1_,  mild_p0l_, mild_p1l_, end_p0u_, end_p1u_,  mild_p0r_, mild_p1r_, end_p0d_, end_p1d_;
    std::vector<std::vector<float>> vec_dict_;

    CloudPtr src_cloud_;
    CloudPtr prev_cloud_;
    CloudPtr nofloor_cloud_;
    CloudPtr nofloor_cloud2_;
    CloudPtr aircraft_cloud_;
    CloudPtr find_target_;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr drawed_cloud_;
    std::vector<ClusterPtr> target_;
    std::vector<ClusterPtr> aircraft_;
    std::vector<float> length_;
    std::vector<double> aircraft_location_;
    //pcl::visualization::CloudViewer viewer_;
    boost::shared_ptr<pcl::visualization::PCLVisualizer> cloud_viewer_;
    pcl::visualization::PointCloudColorHandler<pcl::PointXYZI> &handler_;
    pcl::PandarGrabber interface_;

public:
    AircraftDetect(std::string &calibFile, std::string &pcapFile, const char *type_file, const char *aircraft_data, pcl::visualization::PointCloudColorHandler<pcl::PointXYZI> &handler);
    AircraftDetect(std::string &calibFile, const char *type_file, const char *aircraft_data, std::string &ip, int port, pcl::visualization::PointCloudColorHandler<pcl::PointXYZI> &handler);
    void setParameter();
    void initializeParameter();
    void loadAircraftData(const char *type_file, const char *aircraft_data);
    virtual ~AircraftDetect();
    pcl::PointXYZ findMaxValues(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud);
    pcl::PointXYZ findMinValues(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud);
    void findTarget(const CloudConstPtr &in_cloud_ptr, CloudPtr out_cloud_ptr, PointType centre, float radius);
    void setTargetColor(const CloudConstPtr &in_cloud_ptr, std::vector<int> &in_cluster_indices, pcl::PointCloud<pcl::PointXYZRGB>::Ptr out_cloud_ptr);
    void clipCloud(const CloudConstPtr &in_cloud_ptr, CloudPtr out_cloud_ptr, float min_height, float max_height, float left_position, float right_position);
    void removeFloor(const CloudPtr &in_cloud_ptr, CloudPtr out_nofloor_cloud_ptr, CloudPtr out_onlyfloor_cloud_ptr,
                     float in_max_height = 0.2, float in_floor_max_angle = 0.1);
    std::vector<pcl::PointIndices> regiongrowingSegmentation(const CloudPtr &in_cloud_ptr);
    std::vector<pcl::PointIndices> euclideanCluster(const CloudPtr &in_cloud_ptr, int cluster_size_min, int cluster_size_max, float cluster_tolerance);
    void passthroughCloud(const CloudPtr &in_cloud_ptr, CloudPtr out_cloud_ptr, float min_limit, float max_limit,
                          std::string &field_name, bool select);
    bool detectCircle(const CloudPtr &in_cloud_ptr, float min_radius, float max_radius,
                      Eigen::VectorXf *coefficients, CloudPtr out_cloud_ptr);
    std::vector<ClusterPtr> getCloudFromCluster(const CloudPtr &in_cloud_ptr, std::vector<pcl::PointIndices> cluster_indices, pcl::PointCloud<pcl::PointXYZRGB>::Ptr drawed_cloud, float target_min_height,
                                                float target_max_height, float target_min_width, float target_max_width);
    std::vector<ClusterPtr> getCloudFromCluster(const CloudPtr &in_cloud_ptr, std::vector<pcl::PointIndices> cluster_indices);
    std::vector<ClusterPtr> detectTarget(const CloudConstPtr &in_cloud_ptr);
    std::vector<ClusterPtr> detectAircraft(const CloudConstPtr &in_cloud_ptr);
    bool detectAircraftParameter(const CloudPtr &in_cloud_ptr, float *wing_span, float *engine_distance);
    char *recognizeType(std::vector<float> feature);
    void processCloud(const CloudConstPtr &cloud);
    void rawCloud(const boost::shared_ptr<const pcl::PointCloud<pcl::PointXYZI> >& cloud);
    void run();
};


#endif //DOCKING_GUIDANCE_AIRCRAFTDETECT_H
