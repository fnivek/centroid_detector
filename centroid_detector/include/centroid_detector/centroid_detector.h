#ifndef INCLUDE_ACT_ON_BINDER_DETECT_BINDERS_H_
#define INCLUDE_ACT_ON_BINDER_DETECT_BINDERS_H_

// This class detects binders using pcl thresholding and euclidean segmentation
#include <vector>

#include "ros/ros.h"
#include "actionlib/server/simple_action_server.h"
#include "tf/tf.h"
#include "pcl_conversions/pcl_conversions.h"
#include "pcl/common/io.h"
#include "pcl/point_types.h"
#include "pcl_ros/transforms.h"
#include "pcl/filters/crop_box.h"
#include "pcl/filters/voxel_grid.h"
#include "pcl/PCLPointCloud2.h"
#include "pcl/filters/passthrough.h"
#include "pcl/kdtree/kdtree.h"
#include "pcl/segmentation/extract_clusters.h"
#include "sensor_msgs/PointCloud2.h"
#include "visualization_msgs/Marker.h"
#include "centroid_detector_msgs/DetectCentroidAction.h"

class BinderDetector
{
  public:
    BinderDetector();
    ~BinderDetector();

    // Run the main loop
    void Run();

  private:
    // Private funcs
    void PCCallback(const sensor_msgs::PointCloud2::ConstPtr& msg);
    void DetectCentroidCB(const centroid_detector_msgs::DetectCentroidGoal::ConstPtr& goal);
    bool DetectCentroid(Eigen::Vector4f* result);
    void ExtractPCFromBB(sensor_msgs::PointCloud2& pc);

  private:
    // Private vars
    ros::Rate* loop_rate_;

    // Parameters for binder point cloud crop box
    float min_pc_x_;
    float max_pc_x_;
    float min_pc_y_;
    float max_pc_y_;
    float min_pc_z_;
    float max_pc_z_;
    float min_2d_x_;
    float max_2d_x_;
    float min_2d_y_;
    float max_2d_y_;

    // Parameters for grouping
    float nearest_neighbor_radius_;

    // Distance in the y direction within which two clusters are considered part of the same binder
    float min_centroid_seperation_;

    // Transformations
    tf::TransformListener* tf_listener_;

    // Subscribers and publishers
    ros::Subscriber* pc_sub_;
    ros::Publisher* croped_pc_pub_;
    ros::Publisher* vis_pub_;

    // Node handle
    ros::NodeHandle nh_;

    // Simple action server
    //  ros::NodeHandle must appear first in the .h file see
    //  http://wiki.ros.org/actionlib_tutorials/Tutorials/SimpleActionServer%28ExecuteCallbackMethod%29
    actionlib::SimpleActionServer<centroid_detector_msgs::DetectCentroidAction> as_;

    // Track the lattest pc
    sensor_msgs::PointCloud2 lattest_pc_;
    bool new_pc_;
    sensor_msgs::PointCloud2 cropped_pc_;

    // Feedback
    centroid_detector_msgs::DetectCentroidFeedback feedback_;

  private:
    // Private constants
    static const float kLoopRate;

    // Minimum number in nn search
    static const unsigned int kMinClusterSize;
};

#endif
