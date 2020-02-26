#include "lego_loam/utility.h"
#include "imageProjection.h"
#include "featureAssociation.h"
#include "mapOptmization.h"
#include "transformFusion.h"
#include <chrono>

#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <rosgraph_msgs/Clock.h>

class Main(){

    private:
        double _time_new_segmented_cloud;
        double _time_new_segmented_cloudInfo;

        bool _new_segmented_cloud;
        bool _new_segmented_cloudInfo;
        bool _new_outlier_cloud;

        pcl::PointCloud<PointType>::Ptr _outlier_cloud; //outlier_cloud"
        pcl::PointCloud<PointType>::Ptr _segmented_cloud; //"/segmented_cloud"
        cloud_msgs::cloud_info _seg_msg //segmented_cloud_info
        pcl::PointCloud<PointType>::Ptr _laser_cloudIn;


        pcl::PointCloud<PointType>::Ptr _laser_cloud_corner_last;//"/laser_cloud_corner_last"
        pcl::PointCloud<PointType>::Ptr _laser_loud_surf_last;//"/laser_cloud_surf_last"
        pcl::PointCloud<PointType>::Ptr _outlier_cloud_last;//"/outlier_cloud_last"
        nav_msgs::Odometry _laser_odometry; //"/laser_odom_to_init"

        nav_msgs::Odometry _odom_aftMapped;//"/aft_mapped_to_init",
        // pcl::PointCloud<PointType>::Ptr _global_mapKey_framesDS;//"/laser_cloud_surround"
        // pcl::PointCloud<PointType>::Ptr _cloud_key_poses3D;///key_pose_origin

    int imageProjectionM(){

        ImageProjection IP(nh); //initiate a ImageProjection instance
        *_laser_cloudIn  = IP.laserCloudIn; //original point cloud data from ROS message
        ROS_INFO("\033[1;32m---->\033[0m Image Projection Started.");

        if (*_laser_cloudIn != nullptr){

            // 2. Start and end angle of a scan
            IP.findStartEndAngle();
            // 3. Range image projection
            IP.projectPointCloud();
            // 4. Mark ground points
            IP.groundRemoval();
            // 5. Point cloud segmentation
            IP.cloudSegmentation();
            // 6. Publish all clouds
            IP.publishCloud();
            // 7. Reset parameters for next iteration
            IP.resetParameters();

            *_outlier_cloud = IP.outlierCloud;

            *_segmented_cloud = IP.segmentedCloud;

            _seg_msg = IP.segMsg;
        }
        else{
            return 0;
        }
    }
    
    int featureAssociationM(){

        FeatureAssociation FA(nh);//initiate a FeatureAssociation Instance

        if (*_segmented_cloud != nullptr){
            FA.cloudHeader = _segmented_cloud->header;
            FA.timeScanCur = FA.cloudHeader.stamp.toSec();
            FA.timeNewSegmentedCloud = FA.timeScanCur;
            FA.segmentedCloud->clear();
            FA.segmentedCloud = *_segmented_cloud;
            FA.newSegmentedCloud = true;
        }
        else{
            return ;
        }

        if (_seg_msg != NULL){
            FA.timeNewSegmentedCloudInfo = _seg_msg->header.stamp.toSec();
            FA.segInfo = _seg_msg;
            FA.newSegmentedCloudInfo = true;
        }
        else{
            return ;
        }

        if(*_outlier_cloud != nullptr){
            FA.timeNewOutlierCloud = _outlier_cloud->header.stamp.toSec();
            FA.outlierCloud->clear();
            FA.outlierCloud = *_outlier_cloud;
            FA.newOutlierCloud = true;
        }
        else{
            return ;
        }

        ROS_INFO("\033[1;32m---->\033[0m Feature Association Started.");
        FA.runFeatureAssociation();
        *_laser_cloud_corner_last = FA.laserCloudCornerLast;
        *_laser_loud_surf_last = FA.laserCloudSurfLast;
        *_outlier_cloud_last = FA.outlierCloud;
        _laser_odometry = FA.laserOdometry;
        return 0;
    }

    int mapOptimizationM(){

        MapOptimization MO(nh);//initiate a MapOptimization instance

        if(*_laser_cloud_corner_last != nullptr){
            MO.timeLaserCloudCornerLast = _laser_cloud_corner_last->header.stamp.toSec();
            MO.laserCloudCornerLast->clear();
            MO.laserCloudCornerLast = *_laser_cloud_corner_last;
            MO.newLaserCloudCornerLast = true;
        }
        else{
            return;
        }

        if (*_laser_loud_surf_last != nullptr) {
            MO.timeLaserCloudSurfLast = _laser_loud_surf_last->header.stamp.toSec();
            MO.laserCloudSurfLast->clear();
            MO.laserCloudSurfLast = *_laser_loud_surf_last;
            MO.newLaserCloudSurfLast = true;
        }
        else{
            return;
        }

        if(*_outlier_cloud != nullptr){
            MO.timeLaserCloudOutlierLast = *_outlier_cloud->header.stamp.toSec();
            MO.laserCloudOutlierLast->clear();
            MO.laserCloudOutlierLast = *_outlier_cloud;
            newLaserCloudOutlierLast = true;
        }
        else{
            return;
        }

        if(_laser_odometry != NULL){
            MO.timeLaserOdometry = _laser_odometry->header.stamp.toSec();
            double roll, pitch, yaw;
            geometry_msgs::Quaternion geoQuat = _laser_odometry->pose.pose.orientation;
            tf::Matrix3x3(tf::Quaternion(geoQuat.z, -geoQuat.x, -geoQuat.y, geoQuat.w)).getRPY(roll, pitch, yaw);
            MO.transformSum[0] = -pitch;
            MO.transformSum[1] = -yaw;
            MO.transformSum[2] = roll;
            MO.transformSum[3] = _laser_odometry->pose.pose.position.x;
            MO.transformSum[4] = _laser_odometry->pose.pose.position.y;
            MO.transformSum[5] = _laser_odometry->pose.pose.position.z;
            MO.newLaserOdometry = true;
        }
        else{
            return ;
        }

        ROS_INFO("\033[1;32m---->\033[0m Map Optimization Started.");
        std::thread loopthread(&mapOptimization::loopClosureThread, &MO);
        std::thread visualizeMapThread(&mapOptimization::visualizeGlobalMapThread, &MO);
        
        MO.run();
        _odom_aftMapped = MO.odomAftMapped;
        // *_global_mapKey_framesDS = MO.globalMapKeyFramesDS;
        // *_cloud_key_poses3D = MO.cloudKeyPoses3D;
        loopthread.join();
        visualizeMapThread.join();
        return 0;
    };

    int transformFusionM(){

        TransformFusion TF(nh);//initiate a TransformFusion instance
        ROS_INFO("\033[1;32m---->\033[0m Transform Fusion Started.");

        if (_laser_odometry != NULL) {
            TF.currentHeader = _laser_odometry->header;

            double roll, pitch, yaw;
            geometry_msgs::Quaternion geoQuat = _laser_odometry->pose.pose.orientation;
            tf::Matrix3x3(tf::Quaternion(geoQuat.z, -geoQuat.x, -geoQuat.y, geoQuat.w)).getRPY(roll, pitch, yaw);

            TF.transformSum[0] = -pitch;
            TF.transformSum[1] = -yaw;
            TF.transformSum[2] = roll;

            TF.transformSum[3] = _laser_odometry->pose.pose.position.x;
            TF.transformSum[4] = _laser_odometry->pose.pose.position.y;
            TF.transformSum[5] = _laser_odometry->pose.pose.position.z;

            TF.transformAssociateToMap();

            geoQuat = tf::createQuaternionMsgFromRollPitchYaw
                    (transformMapped[2], -transformMapped[0], -transformMapped[1]);

            TF.laserOdometry2.header.stamp = _laser_odometry->header.stamp;
            TF.laserOdometry2.pose.pose.orientation.x = -geoQuat.y;
            TF.laserOdometry2.pose.pose.orientation.y = -geoQuat.z;
            TF.laserOdometry2.pose.pose.orientation.z = geoQuat.x;
            TF.laserOdometry2.pose.pose.orientation.w = geoQuat.w;
            TF.laserOdometry2.pose.pose.position.x = transformMapped[3];
            TF.laserOdometry2.pose.pose.position.y = transformMapped[4];
            TF.laserOdometry2.pose.pose.position.z = transformMapped[5];
            TF.pubLaserOdometry2.publish(TF.laserOdometry2);

            TF.laserOdometryTrans2.stamp_ = _laser_odometry->header.stamp;
            TF.laserOdometryTrans2.setRotation(tf::Quaternion(-geoQuat.y, -geoQuat.z, geoQuat.x, geoQuat.w));
            TF.laserOdometryTrans2.setOrigin(tf::Vector3(transformMapped[3], transformMapped[4], transformMapped[5]));
            tfBroadcaster2.sendTransform(TF.laserOdometryTrans2);
        }
        else{
            return 0;
        }

        if (_odom_aftMapped != NULL){
            double roll, pitch, yaw;
            geometry_msgs::Quaternion geoQuat = _odom_aftMapped->pose.pose.orientation;
            tf::Matrix3x3(tf::Quaternion(geoQuat.z, -geoQuat.x, -geoQuat.y, geoQuat.w)).getRPY(roll, pitch, yaw);

            TF.transformAftMapped[0] = -pitch;
            TF.transformAftMapped[1] = -yaw;
            TF.transformAftMapped[2] = roll;

            TF.transformAftMapped[3] = _odom_aftMapped->pose.pose.position.x;
            TF.transformAftMapped[4] = _odom_aftMapped->pose.pose.position.y;
            TF.transformAftMapped[5] = _odom_aftMapped->pose.pose.position.z;

            TF.transformBefMapped[0] = _odom_aftMapped->twist.twist.angular.x;
            TF.transformBefMapped[1] = _odom_aftMapped->twist.twist.angular.y;
            TF.transformBefMapped[2] = _odom_aftMapped->twist.twist.angular.z;

            TF.transformBefMapped[3] = _odom_aftMapped->twist.twist.linear.x;
            TF.transformBefMapped[4] = _odom_aftMapped->twist.twist.linear.y;
            TF.transformBefMapped[5] = _odom_aftMapped->twist.twist.linear.z;
        }
        else{
            return 0;
        }
    }

    void run(){
        imageProjectionM();
        featureAssociationM();
        mapOptimizationM();
        transformFusionM();
    }
};


    int main(int argc, char** argv){

        ros::init(argc, argv, "lego_loam");
        Main _m;

        ros::NodeHandle nh("~");
        std::string rosbag;
        std::string imu_topic;
        std::string lidar_topic;

        nh.getParam("rosbag", rosbag);
        nh.getParam("imu_topic", imu_topic);
        nh.getParam("lidar_topic", lidar_topic);

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
        topics.push_back(imu_topic);
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
                //ROS_INFO("cloud");
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
            _m.run();
            rate.sleep();
        }

        return 0;
    }