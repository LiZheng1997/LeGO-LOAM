#include "imageProjection.h"
#include "featureAssociation.h"
#include "mapOptimization.h"
#include "transformFusion.h"
#include <chrono>

#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <rosgraph_msgs/Clock.h>

class Main{

    public:
        Main();
        ~Main();
        void run(ImageProjection& IP,FeatureAssociation& FA,MapOptimization& MO,TransformFusion& TF);
        int imageProjectionM( ImageProjection& IP);
        int featureAssociationM( FeatureAssociation& FA);
        int mapOptimizationM( MapOptimization& MO);
        int transformFusionM( TransformFusion& TF);

    public:
        pcl::PointCloud<PointType>::Ptr _outlier_cloud; //outlier_cloud"
        pcl::PointCloud<PointType>::Ptr _segmented_cloud; //"/segmented_cloud"
        cloud_msgs::cloud_info _seg_msg; //segmented_cloud_info
        pcl::PointCloud<PointType>::Ptr _laser_cloudIn;

        pcl::PointCloud<PointType>::Ptr _laser_cloud_corner_last;//"/laser_cloud_corner_last"
        pcl::PointCloud<PointType>::Ptr _laser_loud_surf_last;//"/laser_cloud_surf_last"
        pcl::PointCloud<PointType>::Ptr _outlier_cloud_last;//"/outlier_cloud_last"
        nav_msgs::Odometry _laser_odometry; //"/laser_odom_to_init"
        nav_msgs::Odometry _odom_aftMapped;//"/aft_mapped_to_init",
};

    int Main::imageProjectionM( ImageProjection& IP){
        _laser_cloudIn  = IP.laserCloudIn; //original point cloud data from ROS message
        ROS_INFO("\033[1;32m---->\033[0m Image Projection Started.");

        if (_laser_cloudIn->points.size() != 0){

            // 2. Start and end angle of a scan
            IP.findStartEndAngle();
            // 3. Range image projection
            IP.projectPointCloud();
            // 4. Mark ground points
            IP.groundRemoval();
            // 5. Point cloud segmentation
            IP.cloudSegmentation();
            // 6. Publish all clouds
            IP.publishClouds();
            // 7. Reset parameters for next iteration
            IP.resetParameters();

            _outlier_cloud = IP.outlierCloud;

            _segmented_cloud = IP.segmentedCloud;

            _seg_msg = IP.segMsg;
        }
        else{
            ROS_INFO("No read data from Lidar.");
            return 0;
        }
        return 1;
    }

    int Main::featureAssociationM( FeatureAssociation& FA){
        if (_segmented_cloud->points.size() != 0){
            sensor_msgs::PointCloud2 msgIn;
            pcl::toROSMsg(*_segmented_cloud,msgIn);
            FA.cloudHeader = msgIn.header;
            FA.timeScanCur = FA.cloudHeader.stamp.toSec();
            FA.timeNewSegmentedCloud = FA.timeScanCur;
            FA.segmentedCloud->clear();
            FA.segmentedCloud = _segmented_cloud;
            FA.newSegmentedCloud = true;
        }
        else{
            ROS_INFO("No read data for _segmented_cloud.");
            return 0;
        }

        if ( _seg_msg.segmentedCloudRange.size() != 0){
            FA.timeNewSegmentedCloudInfo = _seg_msg.header.stamp.toSec();
            FA.segInfo = _seg_msg;
            FA.newSegmentedCloudInfo = true;
        }
        else{
            ROS_INFO("No read data for _seg_msg.");
            return 0;
        }

        if(_outlier_cloud->points.size() != 0){
            sensor_msgs::PointCloud2 msgIn;
            pcl::toROSMsg(*_outlier_cloud,msgIn);
            FA.timeNewOutlierCloud = msgIn.header.stamp.toSec();
            FA.outlierCloud->clear();
            FA.outlierCloud = _outlier_cloud;
            FA.newOutlierCloud = true;
        }
        else{
            ROS_INFO("No read data for _outlier_cloud.");
            return 0;
        }

        ROS_INFO("\033[1;32m---->\033[0m Feature Association Started.");
        FA.runFeatureAssociation();
        _laser_cloud_corner_last = FA.laserCloudCornerLast;
        _laser_loud_surf_last = FA.laserCloudSurfLast;
        _outlier_cloud_last = FA.outlierCloud;
        _laser_odometry = FA.laserOdometry;
        return 1;
    }

    int Main::mapOptimizationM( MapOptimization& MO){

        if(_laser_cloud_corner_last->points.size() != 0){
            sensor_msgs::PointCloud2 msgIn;
            pcl::toROSMsg(*_laser_cloud_corner_last,msgIn);
            MO.timeLaserCloudCornerLast = msgIn.header.stamp.toSec();
            MO.laserCloudCornerLast->clear();
            MO.laserCloudCornerLast = _laser_cloud_corner_last;
            MO.newLaserCloudCornerLast = true;
        }
        else{
            ROS_INFO("No read data for _laser_cloud_corner_last.");
            return 0;
        }

        if (_laser_loud_surf_last->points.size() != 0) {
            sensor_msgs::PointCloud2 msgIn;
            pcl::toROSMsg(*_laser_loud_surf_last,msgIn);                
            MO.timeLaserCloudSurfLast = msgIn.header.stamp.toSec();
            MO.laserCloudSurfLast->clear();
            MO.laserCloudSurfLast = _laser_loud_surf_last;
            MO.newLaserCloudSurfLast = true;
        }
        else{
            ROS_INFO("No read data for _laser_loud_surf_last.");
            return 0;
        }

        if(_outlier_cloud->points.size() != 0){
            sensor_msgs::PointCloud2 msgIn;
            pcl::toROSMsg(*_outlier_cloud,msgIn);
            MO.timeLaserCloudOutlierLast = msgIn.header.stamp.toSec();
            MO.laserCloudOutlierLast->clear();
            MO.laserCloudOutlierLast = _outlier_cloud;
            MO.newLaserCloudOutlierLast = true;
        }
        else{
            ROS_INFO("No read data for _outlier_cloud.");
            return 0;
        }

        if(_laser_odometry.pose.pose.position.x != 0){
            // const nav_msgs::Odometry::ConstPtr& laserOdometry;

            MO.timeLaserOdometry = _laser_odometry.header.stamp.toSec();
            double roll, pitch, yaw;
            geometry_msgs::Quaternion geoQuat = _laser_odometry.pose.pose.orientation;
            tf::Matrix3x3(tf::Quaternion(geoQuat.z, -geoQuat.x, -geoQuat.y, geoQuat.w)).getRPY(roll, pitch, yaw);
            MO.transformSum[0] = -pitch;
            MO.transformSum[1] = -yaw;
            MO.transformSum[2] = roll;
            MO.transformSum[3] = _laser_odometry.pose.pose.position.x;
            MO.transformSum[4] = _laser_odometry.pose.pose.position.y;
            MO.transformSum[5] = _laser_odometry.pose.pose.position.z;
            MO.newLaserOdometry = true;
        }
        else{
            ROS_INFO("No read data for _laser_odometry.");
            return 0;
        }

        ROS_INFO("\033[1;32m---->\033[0m Map Optimization Started.");
        std::thread loopthread(&MapOptimization::loopClosureThread, &MO);
        std::thread visualizeMapThread(&MapOptimization::visualizeGlobalMapThread, &MO);
        // std::thread loopthread()
        
        MO.run();
        _odom_aftMapped = MO.odomAftMapped;
        // *_global_mapKey_framesDS = M->.globalMapKeyFramesDS;
        // *_cloud_key_poses3D = MO.cloudKeyPoses3D;
        loopthread.join();
        visualizeMapThread.join();
        return 1;
    }

    int Main::transformFusionM( TransformFusion& TF){

        ROS_INFO("\033[1;32m---->\033[0m Transform Fusion Started.");

        if (_laser_odometry.pose.pose.position.x != 0) {
            TF.currentHeader = _laser_odometry.header;

            double roll, pitch, yaw;
            geometry_msgs::Quaternion geoQuat = _laser_odometry.pose.pose.orientation;
            tf::Matrix3x3(tf::Quaternion(geoQuat.z, -geoQuat.x, -geoQuat.y, geoQuat.w)).getRPY(roll, pitch, yaw);

            TF.transformSum[0] = -pitch;
            TF.transformSum[1] = -yaw;
            TF.transformSum[2] = roll;

            TF.transformSum[3] = _laser_odometry.pose.pose.position.x;
            TF.transformSum[4] = _laser_odometry.pose.pose.position.y;
            TF.transformSum[5] = _laser_odometry.pose.pose.position.z;

            TF.transformAssociateToMap();

            geoQuat = tf::createQuaternionMsgFromRollPitchYaw
                    (TF.transformMapped[2], -TF.transformMapped[0], -TF.transformMapped[1]);

            TF.laserOdometry2.header.stamp = _laser_odometry.header.stamp;
            TF.laserOdometry2.pose.pose.orientation.x = -geoQuat.y;
            TF.laserOdometry2.pose.pose.orientation.y = -geoQuat.z;
            TF.laserOdometry2.pose.pose.orientation.z = geoQuat.x;
            TF.laserOdometry2.pose.pose.orientation.w = geoQuat.w;
            TF.laserOdometry2.pose.pose.position.x = TF.transformMapped[3];
            TF.laserOdometry2.pose.pose.position.y = TF.transformMapped[4];
            TF.laserOdometry2.pose.pose.position.z = TF.transformMapped[5];
            TF.pubLaserOdometry2.publish(TF.laserOdometry2);

            TF.laserOdometryTrans2.stamp_ = _laser_odometry.header.stamp;
            TF.laserOdometryTrans2.setRotation(tf::Quaternion(-geoQuat.y, -geoQuat.z, geoQuat.x, geoQuat.w));
            TF.laserOdometryTrans2.setOrigin(tf::Vector3(TF.transformMapped[3], TF.transformMapped[4], TF.transformMapped[5]));
            TF.tfBroadcaster2.sendTransform(TF.laserOdometryTrans2);
        }
        else{
            ROS_INFO("No read data for _laser_odometry.");
            return 0;
        }

        if (_odom_aftMapped.pose.pose.position.x != 0){
            double roll, pitch, yaw;
            geometry_msgs::Quaternion geoQuat = _odom_aftMapped.pose.pose.orientation;
            tf::Matrix3x3(tf::Quaternion(geoQuat.z, -geoQuat.x, -geoQuat.y, geoQuat.w)).getRPY(roll, pitch, yaw);

            TF.transformAftMapped[0] = -pitch;
            TF.transformAftMapped[1] = -yaw;
            TF.transformAftMapped[2] = roll;

            TF.transformAftMapped[3] = _odom_aftMapped.pose.pose.position.x;
            TF.transformAftMapped[4] = _odom_aftMapped.pose.pose.position.y;
            TF.transformAftMapped[5] = _odom_aftMapped.pose.pose.position.z;

            TF.transformBefMapped[0] = _odom_aftMapped.twist.twist.angular.x;
            TF.transformBefMapped[1] = _odom_aftMapped.twist.twist.angular.y;
            TF.transformBefMapped[2] = _odom_aftMapped.twist.twist.angular.z;

            TF.transformBefMapped[3] = _odom_aftMapped.twist.twist.linear.x;
            TF.transformBefMapped[4] = _odom_aftMapped.twist.twist.linear.y;
            TF.transformBefMapped[5] = _odom_aftMapped.twist.twist.linear.z;
        }
        else{
            ROS_INFO("No read data for _odom_aftMapped.");
            return 0;
        }
        return 1;
    }
    
    void Main::run(ImageProjection& IP,FeatureAssociation& FA,MapOptimization& MO,TransformFusion& TF){
        imageProjectionM(IP);
        featureAssociationM(FA);
        mapOptimizationM(MO);
        transformFusionM(TF);
    }

int main(int argc, char** argv){

    ros::init(argc, argv, "lego_loam");
    
    ros::NodeHandle nh("~");
    // ros::NodeHandle nh;

    ImageProjection IP(nh); 
    // ImageProjection *IP = new ImageProjection(nh); //initiate a ImageProjection instance 
    FeatureAssociation FA(nh);
    // FeatureAssociation *FA = new FeatureAssociation(nh);//initiate a FeatureAssociation Instance
    MapOptimization MO(nh); 
    // MapOptimization *MO = new MapOptimization(nh);//initiate a MapOptimization instance
    TransformFusion TF(nh); 
    // TransformFusion *TF = new TransformFusion(nh);//initiate a TransformFusion instance
    Main main_module;
    // Main *main_module = new Main();
    // main_module

    std::string rosbag;
    // std::string imu_topic;
    std::string lidar_topic;

    nh.getParam("rosbag", rosbag);
    // nh.getParam("imu_topic", imu_topic);
    nh.getParam("lidar_topic", lidar_topic);
    
    bool use_rosbag = false;
    rosbag::Bag bag;

    if (!rosbag.empty()) {
        try {
        bag.open(rosbag, rosbag::bagmode::Read);
        use_rosbag = true;
        } catch (std::exception& ex) {
        ROS_FATAL("Unable to open rosbag [%s]", rosbag.c_str());
        return 1;
        }
    }

    ROS_INFO("\033[1;32m---->\033[0m LeGO-LOAM Started.");

    //set parameters for reading data from rosbag
    ROS_INFO("ROSBAG");
    std::vector<std::string> topics;
    // topics.push_back(imu_topic);
    topics.push_back(lidar_topic);

    rosbag::View view(bag, rosbag::TopicQuery(topics));

    auto start_real_time = std::chrono::high_resolution_clock::now();
    auto start_sim_time = view.getBeginTime();

    auto prev_real_time = start_real_time;
    auto prev_sim_time = start_sim_time;

    auto clock_publisher = nh.advertise<rosgraph_msgs::Clock>("/clock",1);

    for(const rosbag::MessageInstance& m: view)
    {
        const sensor_msgs::PointCloud2ConstPtr &cloud = m.instantiate<sensor_msgs::PointCloud2>(); 
        if (cloud != NULL){
            IP.cloudHandler(cloud);
            ROS_INFO("cloud");
        }

        rosgraph_msgs::Clock clock_msg;
        clock_msg.clock = m.getTime();
        clock_publisher.publish( clock_msg );

        auto real_time = std::chrono::high_resolution_clock::now();
        if( real_time - prev_real_time > std::chrono::seconds(5) )
        {
            auto sim_time = m.getTime();
            auto delta_real = std::chrono::duration_cast<std::chrono::milliseconds>(real_time-prev_real_time).count()*0.001;
            auto delta_sim = (sim_time - prev_sim_time).toSec();
            ROS_INFO("Processing the rosbag at %.1fX speed.", delta_sim / delta_real);
            prev_sim_time = sim_time;
            prev_real_time = real_time;
        }
        ros::spinOnce();
    }

    bag.close();

    auto real_time = std::chrono::high_resolution_clock::now();
    auto delta_real = std::chrono::duration_cast<std::chrono::milliseconds>(real_time-start_real_time).count()*0.001;
    auto delta_sim = (view.getEndTime() - start_sim_time).toSec();
    ROS_INFO("Entire rosbag processed at %.1fX speed", delta_sim / delta_real);
    
    ros::Rate rate(200);
    while (ros::ok())
    {
        ros::spinOnce();
        // main_module.imageProjectionM(IP);
        // main_module.featureAssociationM(FA);
        // main_module.mapOptimizationM(MO);
        // main_module.transformFusionM(TF);
        // main_module.run(&IP, &FA, &MO, &TF)
        // main_module->run(*IP,*FA,*MO,*TF);
        main_module.run(IP,FA,MO, TF);
        rate.sleep();
    }

    return 0;
}