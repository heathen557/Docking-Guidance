//
// Created by luffy7n on 18-11-5.
//

#ifndef DOCKING_GUIDANCE2_WALKTEST_H
#define DOCKING_GUIDANCE2_WALKTEST_H


#include "pandar_grabber/pandar_grabber.h"
#include "ProcessPointcloud.h"


class WalkTest
{

    int port_, cloud_id_;
    int lost_counter_;
    int cluster_size_min_, cluster_size_max_;
    int succed_detect_counter_;
    float remove_points_upto_, clip_min_height_, clip_max_height_, clip_right_pos_, clip_left_pos_, cluster_tolerance_, target_min_height_, target_max_height_, target_min_length_, target_max_length_;
    float pre_time_, pre_distance_;
    //float mild_x_, endline_y_;
    float x_precision_, y_precision_;
    float origin_distance_, end_distance_, offset_;
    float velocity_;
    bool remove_ground_;
    bool use_region_growing_;
    bool succed_detect_;
    bool succed_target_;
    bool addline_;
    bool detect_flag_;
    string position_;
    string calibration_file_, pcap_file_, ip_;
    Input_Num mild_point0_, mild_point1_, end_point0_, end_point1_;
    vector<float> model_feature_;
    vector<ClusterPtr> target_;
    //vector<cv::Scalar> colors_;
    pcl::PointXYZ mild_p0_, mild_p1_, end_p0_, end_p1_,  mild_p0l_, mild_p1l_, end_p0u_, end_p1u_,  mild_p0r_, mild_p1r_, end_p0d_, end_p1d_;//后考虑如何自动确定中心线;
    PointType pre_target_centroid_;
    CloudPtr model_cloud_;
    CloudPtr src_cloud_;
    CloudPtr prev_cloud_;
    CloudPtr nofloor_cloud_;
    CloudPtr all_cluster_cloud_;
    CloudPtr pre_target_cloud_;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr drawed_cloud_;
    ClusterPtr track_target_cluster_;
    //pcl::visualization::CloudViewer cloud_viewer_;
    boost::shared_ptr<pcl::visualization::PCLVisualizer> cloud_viewer_;
    pcl::visualization::PointCloudColorHandler<pcl::PointXYZI> &handler_;
    pcl::PandarGrabber interface_;
    boost::mutex mtx_;

    //typedef ParticleFilterTracker<PointType, ParticleT> ParticleFilter;
    //typedef ParticleFilter::CoherencePtr CoherencePtr;

public:
    WalkTest(string &calib_file, string &pcap_file, pcl::visualization::PointCloudColorHandler<pcl::PointXYZI> &handler);
    WalkTest(string &calib_file, string &ip, int port, pcl::visualization::PointCloudColorHandler<pcl::PointXYZI> &handler);
    void setParameter();
    void initializeParameter();
    virtual ~WalkTest();
    vector<float> extractClusterFeature(ClusterPtr cluster);
    FpfhFeaturePtr computeFpfhFeature(const CloudPtr &in_cloud_ptr, KdtreePtr tree);
    float calculateSimilarity2(ClusterPtr cluster, CloudPtr model_cloud);
    ClusterPtr detectTarget(const CloudConstPtr &in_cloud_ptr);//, ClusterPtr likelihood_target);//std::vector<int> *target_indices);
    void processCloud(const CloudConstPtr &cloud);
    void rawCloud(const boost::shared_ptr<const Cloud > &cloud);
    //void viewPointCloud(pcl::visualization::PCLVisualizer &viz);
    void viewPointCloud();
    void run();
};
#endif //DOCKING_GUIDANCE2_WALKTEST_H