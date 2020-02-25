#ifndef IMAGEPROJECTION_H
#define IMAGEPROJECTION_H

#include "utility.h"

class ImageProjection{

    public:

        ImageProjection(ros::NodeHandle& nh);

        ~ImageProjection() = default;
        void cloudHandler(const sensor_msgs::PointCloud2ConstPtr &laserCloudMsg);


    private:

        void allocateMemory();
        void resetParameters();
        void copyPointCloud();
        void findStartEndAngle();
        void projectPointCloud();
        void groundRemoval();
        void cloudSegmentation();
        void labelComponents(int row, int col);
        void publishClouds();

        ros::NodeHandle nh;

        ros::Subscriber subLaserCloud;
        
        ros::Publisher pubFullCloud;
        ros::Publisher pubFullInfoCloud;

        ros::Publisher pubGroundCloud;
        ros::Publisher pubSegmentedCloud;
        ros::Publisher pubSegmentedCloudPure;
        ros::Publisher pubSegmentedCloudInfo;
        ros::Publisher pubOutlierCloud;

        pcl::PointCloud<PointType>::Ptr laserCloudIn;

        pcl::PointCloud<PointType>::Ptr fullCloud; // projected velodyne raw cloud, but saved in the form of 1-D matrix
        pcl::PointCloud<PointType>::Ptr fullInfoCloud; // same as fullCloud, but with intensity - range

        pcl::PointCloud<PointType>::Ptr groundCloud;
        pcl::PointCloud<PointType>::Ptr segmentedCloud;
        pcl::PointCloud<PointType>::Ptr segmentedCloudPure;
        pcl::PointCloud<PointType>::Ptr outlierCloud;

        PointType nanPoint; // fill in fullCloud at each iteration

        cv::Mat rangeMat; // range matrix for range image
        cv::Mat labelMat; // label matrix for segmentaiton marking
        cv::Mat groundMat; // ground matrix for ground cloud marking
        int labelCount;

        float startOrientation;
        float endOrientation;

        cloud_msgs::cloud_info segMsg; // info of segmented cloud
        std_msgs::Header cloudHeader;

        std::vector<std::pair<int8_t, int8_t> > neighborIterator; // neighbor iterator for segmentaiton process

        uint16_t *allPushedIndX; // array for tracking points of a segmented object
        uint16_t *allPushedIndY;

        uint16_t *queueIndX; // array for breadth-first search process of segmentation
        uint16_t *queueIndY;
};

#endif  // IMAGEPROJECTION_H