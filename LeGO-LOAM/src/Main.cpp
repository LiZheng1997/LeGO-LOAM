#include "utility.h"
#include "imageProjection.h"
#include "featureAssociation.h"

int main(int argc, char** argv){

    double _time_new_segmented_cloud;
    double _time_new_segmented_cloudInfo;

    bool _new_segmented_cloud;
    bool _new_segmented_cloudInfo;
    bool _new_outlier_cloud;

    pcl::PointCloud<PointType>::Ptr _outlier_cloud;

    pcl::PointCloud<PointType>::Ptr _segmented_cloud;

    cloud_msgs::cloud_info _seg_msg

    pcl::PointCloud<PointType>::Ptr _laser_cloudIn;


    // ros::init(argc, argv, "lego_loam");
    
    ImageProjection IP; //initiate a ImageProjection instance

    

    void imageProjectionM(){
        _laser_cloudIn  = IP::laserCloudIn; //original point cloud data from ROS message

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
    }
    
    void featureAssociationM(){

        FeatureAssociation FA;

        if (newSegmentedCloud && newSegmentedCloudInfo && newOutlierCloud &&
            std::abs(_time_new_segmented_cloudInfo - _time_new_segmented_cloud) < 0.05 &&
            std::abs(_time_new_segmented_cloud - _time_new_segmented_cloud) < 0.05){

            _new_segmented_cloud = false;
            _new_segmented_cloudInfo = false;
            _new_outlier_cloud = false;
        }else{
            return;
        }
        /**
        	1. Feature Extraction
        */
        adjustDistortion();

        calculateSmoothness();

        markOccludedPoints();

        extractFeatures();

        publishCloud(); // cloud for visualization
	
        /**
		2. Feature Association
        */
        if (!systemInitedLM) {
            checkSystemInitialization();
            return;
        }

        updateInitialGuess();

        updateTransformation();

        integrateTransformation();

        publishOdometry();

        publishCloudsLast(); // cloud to mapOptimization
    
    }







    // ROS_INFO("\033[1;32m---->\033[0m Image Projection Started.");

    // ros::spin();
    return 0;
}
