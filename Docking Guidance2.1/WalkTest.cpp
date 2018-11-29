//
// Created by luffy7n on 18-11-5.
//

#include "WalkTest.h"

#include "GlobleData.h"

extern struct control_msg con_msg;

extern bool _run_flag;
void WalkTest::setParameter()
{
    origin_distance_ = 150;
    end_distance_= 120;
    offset_ = 0;
    x_precision_ = 0.0; //偏离中线的容忍度
    y_precision_ = 0.0; //距离终止线的容忍度
    remove_points_upto_ = 0.0;
    clip_min_height_ = -20;
    clip_max_height_ = 2;
    clip_right_pos_ = -1.5;
    clip_left_pos_ = 1.5;
    cluster_size_min_ = 200;//_minPoints;
    cluster_size_max_ = 2000;//_maxPoints;
    cluster_tolerance_ = 0.2;
    target_min_height_ = 0.5;
    target_max_height_ = 1.9;
    target_min_length_ = 0.3;
    target_max_length_ = 0.7;
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
    model_feature_.push_back(1.7);
    model_feature_.push_back(0.5);
}

void WalkTest::initializeParameter()
{
    cloud_id_ = 0;
    lost_counter_ = 0;
    detect_flag_ = false;
    src_cloud_.reset(new Cloud);
    prev_cloud_.reset(new Cloud);
    nofloor_cloud_.reset(new Cloud);
    pre_target_cloud_.reset(new Cloud);
    model_cloud_.reset(new Cloud);
    remove_points_upto_ = 0; // 設定2？
    remove_ground_ = true;
    use_region_growing_ = false;
    succed_detect_ = false;
    succed_target_ = false;
    succed_detect_counter_ = 0;
    addline_ = true;
    track_target_cluster_.reset(new Cluster());
    pcl::io::loadPCDFile("model.pcd", *model_cloud_);

    //cloud_viewer_->reset(new pcl::visualization::CloudViewer("3D Viewer"));
    //cloud_viewer_("3D Viewer");
    cloud_viewer_.reset(new pcl::visualization::PCLVisualizer("3D Viewer"));
    cloud_viewer_->addLine(mild_p0_, mild_p1_, "mildline");
    cloud_viewer_->addLine(mild_p0l_, mild_p1l_, "mildline1");
    cloud_viewer_->addLine(mild_p0r_, mild_p1r_, "mildline2");
    cloud_viewer_->addLine(end_p0_, end_p1_, "endline");
    cloud_viewer_->addLine(end_p0u_, end_p1u_, "endline1");
    cloud_viewer_->addLine(end_p0d_, end_p1d_, "endline2");
}

WalkTest::WalkTest(std::string &calib_file, std::string &pcap_file,
                   pcl::visualization::PointCloudColorHandler<pcl::PointXYZI> &handler)
        : calibration_file_(calib_file)
        , pcap_file_(pcap_file)
        , handler_(handler)
        //, cloud_viewer_("3D viewer")
        , interface_(calibration_file_, pcap_file_)
{
    //cv::generateColors(colors_, 100);
    setParameter();
    initializeParameter();
}

WalkTest::WalkTest(std::string &calib_file, std::string &ip, int port,
                   pcl::visualization::PointCloudColorHandler<pcl::PointXYZI> &handler)
        : calibration_file_(calib_file)
        , ip_(ip)
        , port_(port)
        , handler_(handler)
//, cloud_viewer_("3D viewer")
        , interface_(boost::asio::ip::address::from_string(ip_), port_, calibration_file_)
{
    //cv::generateColors(colors_, 100);
    setParameter();
    initializeParameter();
}

vector<float> WalkTest::extractClusterFeature(ClusterPtr cluster)
{
    vector<float> feature;
    feature.clear();
    float height = cluster->getHeight();
    float length = cluster->getLength();
    feature.push_back(height);
    feature.push_back(length);
    return feature;
}

FpfhFeaturePtr WalkTest::computeFpfhFeature(const CloudPtr &in_cloud_ptr, KdtreePtr tree)
{
    //法向量
    PointNormal::Ptr point_normal(new PointNormal);
    pcl::NormalEstimation<PointType, pcl::Normal> est_normal;
    est_normal.setInputCloud(in_cloud_ptr);
    est_normal.setSearchMethod(tree);
    est_normal.setKSearch(10);
    //est_normal.setSearchSurface();
    est_normal.compute(*point_normal);

    //fpfh估计
    FpfhFeaturePtr fpfh(new FpfhFeature);
    //pcl::FPFHEstimation<pcl::PointXYZ, pcl::Normal, pcl::FPFHSignature33> est_target_fpfh;
    pcl::FPFHEstimationOMP<PointType, pcl::Normal, pcl::FPFHSignature33> est_fpfh;
    est_fpfh.setNumberOfThreads(4);//指定4核计算

    est_fpfh.setInputCloud(in_cloud_ptr);
    est_fpfh.setInputNormals(point_normal);
    est_fpfh.setSearchMethod(tree);
    est_fpfh.setKSearch(10);
    est_fpfh.compute(*fpfh);

    return fpfh;
}

float WalkTest::calculateSimilarity2(ClusterPtr cluster, CloudPtr model_cloud)
{
    CloudPtr cluster_cloud(new Cloud);
    CloudPtr filtered_cluster_cloud(new Cloud);
    CloudPtr filtered_model_cloud(new Cloud);
    CloudPtr final_cloud(new Cloud);
    transformPointCloud(cluster->getCloud(), cluster_cloud);
    // pcl::io::savePCDFile("x.pcd",*cluster_cloud);
    pcl::VoxelGrid<PointType> sor;
    sor.setInputCloud(model_cloud);
    sor.setLeafSize(0.01, 0.01, 0.01);
    sor.filter(*filtered_model_cloud);
    sor.setInputCloud(cluster_cloud);
    sor.setLeafSize(0.01, 0.01, 0.01);
    sor.filter(*filtered_cluster_cloud); // 此可否省去。因为点数本来就少

    KdtreePtr tree;
    FpfhFeaturePtr model_fpfh = computeFpfhFeature(filtered_model_cloud, tree);
    FpfhFeaturePtr cluster_fpfh = computeFpfhFeature(filtered_cluster_cloud, tree);
    pcl::SampleConsensusInitialAlignment<PointType, PointType, pcl::FPFHSignature33> sac_ia;
    sac_ia.setInputTarget(filtered_model_cloud);
    sac_ia.setTargetFeatures(model_fpfh);
    sac_ia.setInputSource(filtered_cluster_cloud);
    sac_ia.setSourceFeatures(cluster_fpfh);
    sac_ia.setNumberOfSamples(10);
    sac_ia.setCorrespondenceRandomness(6);
    sac_ia.setEuclideanFitnessEpsilon(0.001);
    sac_ia.setTransformationEpsilon(1e-4);
    sac_ia.setRANSACIterations(20);
    sac_ia.align(*final_cloud);
    float score = sac_ia.getFitnessScore();
    cout <<score << "okd" << endl;
    return score;
}

ClusterPtr WalkTest::detectTarget(const CloudConstPtr &in_cloud_ptr)//, ClusterPtr likelihood_target)
{
    CloudPtr removed_points_cloud(new Cloud);
    CloudPtr clipped_cloud(new Cloud);
    CloudPtr onlyfloor_cloud(new Cloud);
    vector<pcl::PointIndices> cluster_indices;
    vector<ClusterPtr> likelihood_clusters;
    //ClusterPtr likelihood_target;
    if (remove_points_upto_ > 0)
    {
        removePointsUpto(in_cloud_ptr, removed_points_cloud, remove_points_upto_);
    }
    else
    {
        pcl::copyPointCloud(*in_cloud_ptr, *removed_points_cloud);
    }
    //downsampleCloud
    //denoise
    clipCloud(removed_points_cloud, clipped_cloud, clip_min_height_, clip_max_height_, clip_right_pos_, clip_left_pos_);
    if (remove_ground_)
    {
        removeFloor(clipped_cloud, nofloor_cloud_, onlyfloor_cloud);
    }
    else
    {
        nofloor_cloud_ = clipped_cloud;
    }
    if (use_region_growing_)
    {
        cluster_indices = regiongrowingSegmentation(nofloor_cloud_);
    }
    else
    {
        cluster_indices = euclideanCluster(nofloor_cloud_, cluster_size_min_, cluster_size_max_, cluster_tolerance_);
    }
    //differenceNormalSegmentation
    if (getCloudFromCluster(nofloor_cloud_, cluster_indices, target_min_height_, target_max_height_, target_min_length_, target_max_length_, &likelihood_clusters))
    {
        //succed_detect_ = true;
        succed_detect_counter_++;
        vector<float> feature;
        float min_dist = 100; //考虑用系统语言定义的max代替
        int min_id = 0;
        for (int i = 0; i < likelihood_clusters.size(); ++i)
        {
            feature = extractClusterFeature(likelihood_clusters[i]);
            float similarity = calculateSimilarity(feature, model_feature_);
            //float similarity2 = calculateSimilarity2(likelihood_clusters[i], model_cloud_);  //model_cloud后可先处理好，不每次都处理
            cout << "Similarity: " << similarity << endl;
            if (similarity < min_dist)
            {
                min_dist = similarity;
                min_id = i;
            }
        }
        //likelihood_target = likelihood_clusters[min_id];
        //return true;
        succed_detect_ = true;
        return likelihood_clusters[min_id];
    }
    else
    {
        //succed_detect_ = false;
        //return false;
        succed_detect_ = false;
    }
    //return likelihood_target; //
}

void WalkTest::processCloud(const CloudConstPtr &cloud)
{




    boost::mutex::scoped_lock lock(mtx_);
    //cloud_id_++;
    if (cloud_id_ > 10 )//&& cloud_id_ % 1 == 0)
    {
        drawed_cloud_.reset(new pcl::PointCloud<pcl::PointXYZRGB>);
        vector<int> detect_target_indices;
        vector<int> track_target_indices;
        ClusterPtr detect_target_cluster(new Cluster()); //
        float detect_target_distance, track_target_distance;
        Input_Num detect_cen, track_cen;
        POSINFO status_ud, status_lr;
        bool choose_detect = false;
        if (prev_cloud_->empty())
        {
            pcl::copyPointCloud(*cloud, *prev_cloud_);
            pre_distance_ = 0;
            pre_time_ = prev_cloud_->header.stamp;
        }
        else
        {
            detect_target_cluster = detectTarget(cloud);
            if (succed_detect_ == true)//detectTarget(cloud, detect_target_cluster)) //此處是指針傳參成功了嗎
            {
                detect_target_indices = detect_target_cluster->getPointIndices();
                //int test = detect_target_indices.size();
                //cout << "****************test:" << test << endl;
                vector<float> detect_feature = extractClusterFeature(detect_target_cluster);
                vector<float> track_feature = extractClusterFeature(track_target_cluster_);
                float detect_similarity = calculateSimilarity(detect_feature, model_feature_);
                float track_similarity = calculateSimilarity(track_feature, model_feature_);
                float similarity_difference = fabs(detect_similarity - track_similarity);
                cout <<"*************************" << "detect: " << detect_similarity << "  " << "track: " << track_similarity << endl;
                if (pre_target_cloud_->empty())
                {
                    //pcl::copyPointCloud(*detect_target_cluster->getCloud(), *pre_target_cloud_);
                    pre_target_centroid_.x = detect_target_cluster->getCentroid().x;
                    pre_target_centroid_.y = detect_target_cluster->getCentroid().y;
                    pre_target_centroid_.z = detect_target_cluster->getCentroid().z;
                }

                detect_flag_ = true;
                float moving_distance = fabs(pre_distance_ - detect_target_cluster->getCentroid().y);
                float time = cloud->header.stamp - pre_time_;
                float velocity = 1000000 * moving_distance / time;
                pre_time_ = cloud->header.stamp;
                pre_distance_ = detect_target_cluster->getCentroid().y;
                //origin_distance_ = detect_target_cluster->getCentroid().y;
                detect_cen = {detect_target_cluster->getCentroid().x, detect_target_cluster->getCentroid().y};
                float inteval_distance = calculatePointDistance(pre_target_centroid_, detect_target_cluster->getCentroid());
                cout << "interval distance: " << inteval_distance << endl;

                if (inteval_distance < 0.3 || lost_counter_ >= 20 || similarity_difference > 0.25 || track_target_cluster_->getLength() < target_min_length_ ||
                    track_target_cluster_->getLength() > target_max_length_  || track_target_cluster_->getHeight() < target_min_height_ || track_target_cluster_->getHeight() > target_max_height_)
                {
                    choose_detect = true;
                    detect_target_distance = detect_target_cluster->getCentroid().y;;
                    //pcl::copyPointCloud(*detect_target_cluster->getCloud(), *pre_target_cloud_);
                    pre_target_centroid_.x = detect_target_cluster->getCentroid().x;
                    pre_target_centroid_.y = detect_target_cluster->getCentroid().y;
                    pre_target_centroid_.z = detect_target_cluster->getCentroid().z;
                    succed_target_ = true;
                    lost_counter_ = 0;
                }
            }

            cout << "track search for target..." << endl;
            if (findTarget(nofloor_cloud_, pre_target_centroid_, &track_target_indices))
            {
                cout << "track search succed..." << endl;
                cout << "detect: " <<"(" << pre_target_centroid_.x << " " << pre_target_centroid_.y << " " << pre_target_centroid_.z << ")" << endl;
                int cluster_id = 0;
                ClusterPtr cluster(new Cluster());
                cluster->setCloud(nofloor_cloud_, track_target_indices, cluster_id, 255, 0 ,0);
                track_target_cluster_ = cluster;
                track_target_distance = cluster->getCentroid().y;
                track_cen = {cluster->getCentroid().x, cluster->getCentroid().y};
                pre_target_centroid_.x = cluster->getCentroid().x;
                pre_target_centroid_.y = cluster->getCentroid().y;
                pre_target_centroid_.z = cluster->getCentroid().z;
                cout << "track: " << "(" << pre_target_centroid_.x << " " << pre_target_centroid_.y << " " << pre_target_centroid_.z << ")" << endl;
                succed_target_ = true;
            }
            else
            {
                succed_target_ = false;
                lost_counter_++;
            }

            if (choose_detect)
            {
                origin_distance_ = detect_target_distance;
                status_ud = Plane_Straight_line_UD(end_point0_, end_point1_, detect_cen);
                end_distance_ = status_ud.offset;
                status_lr = Plane_Straight_line_LR(mild_point0_, mild_point1_, detect_cen);
                position_ = status_lr.position;
                offset_ = status_lr.offset;
                setTargetColor(nofloor_cloud_, detect_target_indices, drawed_cloud_);
            }
            else
            {
                origin_distance_ = track_target_distance;
                status_ud = Plane_Straight_line_UD(end_point0_, end_point1_, track_cen);
                end_distance_ = status_ud.offset;
                status_lr = Plane_Straight_line_LR(mild_point0_, mild_point1_, track_cen);
                position_ = status_lr.position;
                offset_ = status_lr.offset;
                setTargetColor(nofloor_cloud_, track_target_indices, drawed_cloud_);
            }

            pcl::copyPointCloud(*cloud, *prev_cloud_);
        }
    }
    else
    {
        printf(". ");
    }
}

void WalkTest::rawCloud(const boost::shared_ptr<const Cloud> &cloud)
{
    cloud_id_++;
    pcl::copyPointCloud(*cloud, *src_cloud_);
    //saveSrcCloud(cloud);
    pcl::console::TicToc time;

    if (cloud_id_ % 2 == 0)
    {
        time.tic();
        processCloud(cloud);
        cout << "Process "<<cloud_id_ << "th frame cost " << time.toc() << "ms" << endl;
        cout << "***************distance:" << origin_distance_ << endl;
    }

    _mtx.lock();
    _READYTOSENDFLAG = 1;// fail时需再回0吗？
    _displayinfo.craft = "WalkTest";
    _displayinfo.distance = end_distance_; //fabs(distance_);
    _displayinfo.speed = 0.0; //velocity_;
    _displayinfo.position = position_; //position_; //后再上判断
    _displayinfo.offset = offset_; //offset_;
    _displayinfo.detectflag = detect_flag_;
    _mtx.unlock();
}

//void WalkTest::viewPointCloud(pcl::visualization::PCLVisualizer &viz)
//{
//    boost::mutex::scoped_lock lock (mtx_);
//    std::cout << "<Esc>, \'q\', \'Q\': quit the program" << std::endl;
//    if (addline_)
//    {
//        viz.addLine(mild_p0_, mild_p1_, "mildline");
//        viz.addLine(mild_p0l_, mild_p1l_, "mildline1");
//        viz.addLine(mild_p0r_, mild_p1r_, "mildline2");
//        viz.addLine(end_p0_, end_p1_, "endline");
//        viz.addLine(end_p0u_, end_p1u_, "endline1");
//        viz.addLine(end_p0d_, end_p1d_, "endline2");
//        addline_ = false;
//    }
//    if (succed_target_)
//    {
//        if (!viz.updatePointCloud(drawed_cloud_, "drawed_cloud"))
//        {
//            viz.addPointCloud(drawed_cloud_, "drawed_cloud");
//            viz.resetCameraViewpoint("drawed_cloud");
//        }
//    }
//    else
//    {
//        if (src_cloud_)
//        {
//            handler_.setInputCloud(src_cloud_);
//            if (!viz.updatePointCloud<pcl::PointXYZI>(src_cloud_, handler_, "src_cloud"))
//            {
//                viz.addPointCloud(src_cloud_, handler_, "src_cloud");
//                viz.resetCameraViewpoint("src_cloud");
//            }
//        }
//        //調試看前面代碼有無必要添加
//    }
//}

void WalkTest::viewPointCloud() //參照github上相關例程優化顯示
{
    std::cout << "<Esc>, \'q\', \'Q\': quit the program" << std::endl;

//    pcl::io::savePCDFile("cloud.pcd",*drawed_cloud_);

    while (!cloud_viewer_->wasStopped())
    {

        if (succed_target_)
        {
            if (!cloud_viewer_->updatePointCloud(drawed_cloud_))
            {
                cloud_viewer_->addPointCloud(drawed_cloud_);
            }
        }
        else
        {
            if (src_cloud_)
            {
                handler_.setInputCloud(src_cloud_);
                if (!cloud_viewer_->updatePointCloud<pcl::PointXYZI>(src_cloud_, handler_))
                {
                    cloud_viewer_->addPointCloud<pcl::PointXYZI>(src_cloud_, handler_);
                }
            }
        }
        cloud_viewer_->spinOnce(1, true);
        // if (!interface_.isRunning())
            // {
            //    cloud_viewer_->spin();
            // }

        // boost::this_thread::sleep(boost::posix_time::microseconds(3000 *10 ));

        if (_run_flag == false)
        {
            break;
        }
    }
}

void WalkTest::run()
{
    boost::function<void(const boost::shared_ptr<const Cloud> &)> f =
            boost::bind(&WalkTest::rawCloud, this, _1);
    boost::signals2::connection c = interface_.registerCallback(f);

    //cloud_viewer_.runOnVisualizationThread(boost::bind(&WalkTest::viewPointCloud, this, _1), "viewPointCloud");

    interface_.start();
    //controlKey(cloud_id_);


    viewPointCloud();    //点云显示



//    while (!cloud_viewer_.wasStopped())
//    {
//        boost::this_thread::sleep(boost::posix_time::microseconds(1000));
//        if (_run_flag == false)
//        {
//            break;
//        }
//    }

    _run_flag == false;
    cout<<"walktest read finish"<<endl;
    interface_.stop();
    c.disconnect();
}

WalkTest::~WalkTest()
{
    // TODO Auto-generated destructor stub
}