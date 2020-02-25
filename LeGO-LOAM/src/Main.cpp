#include "utility.h"

int main(int argc, char** argv){

    pcl::PointCloud<PointType>::Ptr _outlier_cloud;

    pcl::PointCloud<PointType>::Ptr _segmented_cloud;

    cloud_msgs::cloud_info _seg_msg

    pcl::PointCloud<PointType>::Ptr _laser_cloudIn;

    // ros::init(argc, argv, "lego_loam");
    
    ImageProjection IP;

    _laser_cloudIn  = IP::laserCloudIn; 

    // 2. Start and end angle of a scan
    IP::findStartEndAngle();
    // 3. Range image projection
    IP::projectPointCloud();
    // 4. Mark ground points
    IP::groundRemoval();
    // 5. Point cloud segmentation
    IP::cloudSegmentation();
    // 6. Publish all clouds
    IP::publishCloud();
    // 7. Reset parameters for next iteration
    IP::resetParameters();

    _outlier_cloud = IP::outlierCloud;

    _segmented_cloud = IP::segmentedCloud;

    _seg_msg = IP::segMsg;

    

    // ROS_INFO("\033[1;32m---->\033[0m Image Projection Started.");

    // ros::spin();
    return 0;
}
