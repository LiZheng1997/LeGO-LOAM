#ifndef TRANSFORMFUSION_H
#define TRANSFORMFUSION_H

#include "lego_loam/utility.h"

class TransformFusion{

    public:
        TransformFusion(ros::NodeHandle& node);

        ~TransformFusion();

        void transformAssociateToMap();
        void laserOdometryHandler(const nav_msgs::Odometry::ConstPtr& laserOdometry);
        void odomAftMappedHandler(const nav_msgs::Odometry::ConstPtr& odomAftMapped);
        std_msgs::Header currentHeader;
        float transformSum[6];
        nav_msgs::Odometry laserOdometry2;
        ros::Publisher pubLaserOdometry2;
        float transformMapped[6];
        tf::TransformBroadcaster tfBroadcaster2;
        float transformBefMapped[6];
        float transformAftMapped[6];

    // private:
    public:

        ros::NodeHandle& nh;

        
        ros::Subscriber subLaserOdometry;
        ros::Subscriber subOdomAftMapped;
    

        
        tf::StampedTransform laserOdometryTrans2;
        

        tf::StampedTransform map_2_camera_init_Trans;
        tf::TransformBroadcaster tfBroadcasterMap2CameraInit;

        tf::StampedTransform camera_2_base_link_Trans;
        tf::TransformBroadcaster tfBroadcasterCamera2Baselink;

        float transformIncre[6];
        
        
        

    
};

#endif // TRANSFORMFUSION_H