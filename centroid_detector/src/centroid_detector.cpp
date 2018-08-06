#include "centroid_detector/centroid_detector.h"

BinderDetector::BinderDetector()
  : loop_rate_(NULL)
  , tf_listener_(NULL)
  , depth_sub_(NULL)
  , croped_pc_pub_(NULL)
  , vis_pub_(NULL)
  , new_pc_(false)
  , as_(nh_, "/centroid_detector", boost::bind(&BinderDetector::DetectCentroidCB, this, _1), false)
{
    // Setup loop rate
    loop_rate_ = new ros::Rate(kLoopRate);

    // Grab ros params
    ros::NodeHandle private_nh("~");
    private_nh.param<float>("min_pc_x", min_pc_x_, 0);
    private_nh.param<float>("max_pc_x", max_pc_x_, 1);
    private_nh.param<float>("min_pc_y", min_pc_y_, -0.3);
    private_nh.param<float>("max_pc_y", max_pc_y_, 0.5);
    private_nh.param<float>("min_pc_z", min_pc_z_, 0.85);
    private_nh.param<float>("max_pc_z", max_pc_z_, 1.016);
    private_nh.param<float>("nearest_neighbor_radius", nearest_neighbor_radius_, 0.03);
    private_nh.param<float>("min_centroid_seperation", min_centroid_seperation_, 0.05);

    // Grab cam_intrinsics_.ra params
    cam_intrinsics_.camera_factor = 1000;
    private_nh.param<float>("/head_cam_intrinsics_.ra/rgb/cam_intrinsics_.ra_info/K[0]", cam_intrinsics_.fx);
    private_nh.param<float>("/head_cam_intrinsics_.ra/rgb/cam_intrinsics_.ra_info/K[2]", cam_intrinsics_.cx);
    private_nh.param<float>("/head_cam_intrinsics_.ra/rgb/cam_intrinsics_.ra_info/K[4]", cam_intrinsics_.fy);
    private_nh.param<float>("/head_cam_intrinsics_.ra/rgb/cam_intrinsics_.ra_info/K[5]", cam_intrinsics_.cy);


    // Setup tf
    tf_listener_ = new tf::TransformListener();

    // Setup publisher
    croped_pc_pub_ = new ros::Publisher();
    *croped_pc_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("centroid_detector_croped_pc", 1);
    vis_pub_ = new ros::Publisher();
    *vis_pub_ = nh_.advertise<visualization_msgs::Marker>("centroid_detector_bbox", 1);

    // Setup service and subscribers
    depth_sub_ = new ros::Subscriber();
    as_.start();
}

BinderDetector::~BinderDetector()
{
    if (loop_rate_)
        delete loop_rate_;
    if (tf_listener_)
        delete tf_listener_;
    if (depth_sub_)
        delete depth_sub_;
}

void BinderDetector::DetectCentroidCB(const centroid_detector_msgs::DetectCentroidGoal::ConstPtr& goal)
{
    ros::NodeHandle private_nh("~");
    private_nh.param<float>("min_pc_x", min_pc_x_, goal->min_x);
    private_nh.param<float>("max_pc_x", max_pc_x_, goal->max_x);
    private_nh.param<float>("min_pc_y", min_pc_y_, goal->min_y);
    private_nh.param<float>("max_pc_y", max_pc_y_, goal->max_y);
    private_nh.param<float>("min_pc_z", min_pc_z_, goal->min_z);
    private_nh.param<float>("max_pc_z", max_pc_z_, goal->max_z);
    min_2d_x_ = goal->min_2d_x;
    max_2d_x_ = goal->max_2d_x;
    min_2d_y_ = goal->min_2d_y;
    max_2d_y_ = goal->max_2d_y;

    ROS_INFO("Centroid detector activated");
    // Try to find a binder
    Eigen::Vector4f centroid;
    if (!DetectCentroid(&centroid))
    {
        ROS_WARN("Failed to find a centroid");
        centroid_detector_msgs::DetectCentroidResult res;


        res.centroid.position.x = 0;
        res.centroid.position.y = 0;
        res.centroid.position.z = 0;
        res.centroid.orientation.x = 0;
        res.centroid.orientation.y = 0;
        res.centroid.orientation.z = 0;
        res.centroid.orientation.w = 1;

        res.success = false;
        // Complete
        as_.setSucceeded(res);
        return;
    }

    // Make sure not preempted
    if (as_.isPreemptRequested())
    {
        as_.setPreempted();
        return;
    }


    // Construct a pose
    centroid_detector_msgs::DetectCentroidResult res;


    res.centroid.position.x = centroid[0];
    res.centroid.position.y = centroid[1];
    res.centroid.position.z = centroid[2];
    res.centroid.orientation.x = 0;
    res.centroid.orientation.y = 0;
    res.centroid.orientation.z = 0;
    res.centroid.orientation.w = 1;

    res.success = true;
    // Complete
    as_.setSucceeded(res);
}

void BinderDetector::DepthCallback(const sensor_msgs::Image::ConstPtr& msg)
{
    ROS_INFO("Got the depth image");
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
        cv_ptr = cv_bridge::toCvCopy(msg, msg->encoding.c_str());
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    cv::Mat depth_image = cv_ptr->image;
    pcl::PointCloud<pcl::PointXYZ>::Ptr temp = depth2PointCloud(depth_image);

    // Convert back to sensor msg
    pcl::toROSMsg(*temp, lattest_pc_);
    lattest_pc_.header.frame_id = "/head_camera_rgb_optical_frame";
    lattest_pc_.header.stamp = ros::Time();

    new_pc_ = true;
}

bool BinderDetector::DetectCentroid(Eigen::Vector4f* result)
{
    // Wait for a new point cloud
    //  Subscribe to depth image topic
    *depth_sub_ = nh_.subscribe("/head_camera/depth_registered/image_raw", 1, &BinderDetector::DepthCallback, this);
    new_pc_ = false;
    while (ros::ok() && !as_.isPreemptRequested())
    {
        feedback_.status.data = "Waiting for point cloud";
        as_.publishFeedback(feedback_);
        ROS_INFO("%s", feedback_.status.data.c_str());
        if (new_pc_)
            break;
        ros::spinOnce();
        loop_rate_->sleep();
    }
    // Unsubscribe from pc topic
    // pc_sub_->shutdown();
    depth_sub_->shutdown();

    // Make sure not preempted
    if (as_.isPreemptRequested())
    {
        as_.setPreempted();
        return false;
    }

    // 2D crop
    // ExtractPCFromBB(lattest_pc_);

    // Transform pc to base_link
    sensor_msgs::PointCloud2 tfed_pc;
    try
    {
        feedback_.status.data = "Waiting for tf";
        as_.publishFeedback(feedback_);
        ROS_INFO("%s", feedback_.status.data.c_str());
        tf_listener_->waitForTransform("/base_link", lattest_pc_.header.frame_id, ros::Time(), ros::Duration(10.0));
        pcl_ros::transformPointCloud("/base_link", lattest_pc_, tfed_pc, *tf_listener_);
    }
    catch (tf::TransformException ex)
    {
        ROS_ERROR("Failed to transform point cloud to /base_link because: %s", ex.what());
        as_.setAborted();
        return false;
    }

    // Make sure not preempted
    if (as_.isPreemptRequested())
    {
        as_.setPreempted();
        return false;
    }

    // Transform to pcl data type
    pcl::PCLPointCloud2::Ptr temp(new pcl::PCLPointCloud2);
    pcl_conversions::toPCL(tfed_pc, *temp);
    pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_tfed_pc(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromPCLPointCloud2(*temp, *pcl_tfed_pc);

    // Crop the pc
    feedback_.status.data = "Cropping Point cloud";
    as_.publishFeedback(feedback_);
    ROS_INFO("%s", feedback_.status.data.c_str());
    pcl::CropBox<pcl::PointXYZ> crop_box(true);
    Eigen::Vector4f min_pt(min_pc_x_, min_pc_y_, min_pc_z_, 1);
    Eigen::Vector4f max_pt(max_pc_x_, max_pc_y_, max_pc_z_, 1);
    std::cout << max_pc_z_ << std::endl;

    crop_box.setMin(min_pt);
    crop_box.setMax(max_pt);
    crop_box.setInputCloud(pcl_tfed_pc);
    std::vector<int> indices;
    crop_box.filter(indices);
    pcl::PointCloud<pcl::PointXYZ>::Ptr croped_pc(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::copyPointCloud(*pcl_tfed_pc, indices, *croped_pc);

    // Publish a representation of the crop box
    visualization_msgs::Marker marker;
    marker.header.frame_id = "base_link";
    marker.header.stamp = ros::Time();
    marker.ns = "centroid_detector";
    marker.id = 0;
    marker.type = visualization_msgs::Marker::CUBE;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.position.x = (min_pc_x_ + max_pc_x_) / 2;
    marker.pose.position.y = (min_pc_y_ + max_pc_y_) / 2;
    marker.pose.position.z = (min_pc_z_ + max_pc_z_) / 2;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = max_pc_x_ - min_pc_x_;
    marker.scale.y = max_pc_y_ - min_pc_y_;
    marker.scale.z = max_pc_z_ - min_pc_z_;
    marker.color.a = 0.5; // Don't forget to set the alpha!
    marker.color.r = 1.0;
    marker.color.g = 1.0;
    marker.color.b = 0.0;
    vis_pub_->publish( marker );

    // Publish the croped cloud for debug
    pcl::PCLPointCloud2::Ptr temp2(new pcl::PCLPointCloud2);
    pcl::toPCLPointCloud2(*croped_pc, *temp2);
    sensor_msgs::PointCloud2 msg;
    pcl_conversions::fromPCL(*temp2, msg);
    msg.header.frame_id = "base_link";
    msg.header.stamp = ros::Time();
    croped_pc_pub_->publish(msg);

    // Make sure not preempted
    if (as_.isPreemptRequested())
    {
        as_.setPreempted();
        return false;
    }

    // Construct kdtree
    feedback_.status.data = "Constructing kdtree";
    as_.publishFeedback(feedback_);
    ROS_INFO("%s", feedback_.status.data.c_str());
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
    tree->setInputCloud(croped_pc);

    // Perform euclidian segmentation
    feedback_.status.data = "Performing euclidian segmentation";
    as_.publishFeedback(feedback_);
    ROS_INFO("%s", feedback_.status.data.c_str());
    std::vector<pcl::PointIndices> cluster_indicies;
    pcl::EuclideanClusterExtraction<pcl::PointXYZ> segmenter;
    segmenter.setClusterTolerance(nearest_neighbor_radius_);
    segmenter.setMinClusterSize(kMinClusterSize);
    segmenter.setMaxClusterSize(999999);
    segmenter.setInputCloud(croped_pc);
    segmenter.extract(cluster_indicies);

    // Make sure not preempted
    if (as_.isPreemptRequested())
    {
        as_.setPreempted();
        return false;
    }

    // Get centroids
    std::vector<Eigen::Vector4f> centroids;
    int i = 0;
    for (std::vector<pcl::PointIndices>::iterator it = cluster_indicies.begin(); it < cluster_indicies.end(); ++it)
    {
        // Make sure not preempted
        if (as_.isPreemptRequested())
        {
            as_.setPreempted();
            return false;
        }

        feedback_.status.data = "Calculating centroid";
        as_.publishFeedback(feedback_);
        ROS_INFO("%s", feedback_.status.data.c_str());
        Eigen::Vector4f centroid;
        pcl::compute3DCentroid(*croped_pc, *it, centroid);

        // Combine centroids who are likely from the same binder
        //  This is done using the centroid y value
        //  No two binders should occupy the same y space
        //  If they are then keep the centroid with the smalles x value
        bool add_centroid = true;
        for (std::vector<Eigen::Vector4f>::iterator it = centroids.begin(); it < centroids.end(); ++it)
        {
            // Check distance in y
            if (std::fabs((*it)[1] - centroid[1]) < min_centroid_seperation_)
            {
                // These are duplicates keep only the one with the smaller x value
                if ((*it)[0] > centroid[0])
                {
                    // Replace the current centroid
                    centroids.erase(it);
                    break;
                }
                else
                {
                    // We already have a better centroid so don't add this one
                    add_centroid = false;
                    break;
                }
            }
        }

        // Add the centroid if there isn't a better one already
        if (add_centroid)
        {
            centroids.push_back(centroid);
        }
    }

    // Make sure not preempted
    if (as_.isPreemptRequested())
    {
        as_.setPreempted();
        return false;
    }

    ROS_INFO("--------------------------------------");
    for (std::vector<Eigen::Vector4f>::iterator it = centroids.begin(); it < centroids.end(); ++it)
    {
        ROS_INFO("Binder centroid: [%f,\t%f,\t%f]", (*it)[0], (*it)[1], (*it)[2]);
    }

    // Return fail if none were found
    if (!centroids.size())
    {
        ROS_WARN("No binder centroids found");
        as_.setAborted();
        return false;
    }

    // Find the center most binder (y value clossest to 0)
    feedback_.status.data = "Getting center centroid";
    as_.publishFeedback(feedback_);
    ROS_INFO("%s", feedback_.status.data.c_str());
    (*result) = centroids[0];
    for (int i = 1; i < centroids.size(); ++i)
    {
        if (std::fabs(centroids[i][1]) < std::fabs((*result)[1]))
        {
            (*result) = centroids[i];
        }
    }

    return true;
}

void BinderDetector::Run()
{
    while (ros::ok())
    {
        // Handle ros callbacks
        ros::spinOnce();

        // Maintain loop rate
        loop_rate_->sleep();
    }
}

void BinderDetector::ExtractPCFromBB(sensor_msgs::PointCloud2& pc)
{
    // Make sure there is a 2d box
    if (!(max_2d_x_ || max_2d_y_ || min_2d_x_ || min_2d_y_)) {
        return;
    }

    // Convert to PCL
    pcl::PCLPointCloud2::Ptr temp(new pcl::PCLPointCloud2);
    pcl_conversions::toPCL(pc, *temp);
    pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_pc(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromPCLPointCloud2(*temp, *pcl_pc);

    // Fill in indicies
    std::vector<int> indices;
    for (int row = min_2d_y_; row <= max_2d_y_; ++row) {
        int row_offset = row * pcl_pc->width;
        for (int col = min_2d_x_; col <= max_2d_x_; ++col) {
            indices.push_back(row_offset + col);
        }
    }

    // Copy pc
    pcl::PointCloud<pcl::PointXYZ>::Ptr cropped(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::copyPointCloud(*pcl_pc, indices, *cropped);

    // Convert back to sensor msg
    pcl::toROSMsg(*cropped, pc);
    pc.header.frame_id = "/head_cam_intrinsics_.ra_rgb_optical_frame";
    pc.header.stamp = ros::Time();
}

pcl::PointCloud<pcl::PointXYZ>::Ptr BinderDetector::depth2PointCloud(cv::Mat depth)
{
    int rows = max_2d_y_ - min_2d_y_ + 1;
    int cols = max_2d_x_ - min_2d_x_ + 1;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud ( new pcl::PointCloud<pcl::PointXYZ>(cols, rows) );

    for(int i=min_2d_y_; i<=max_2d_y_; i++)
    {
        for(int j=min_2d_x_; j<=max_2d_x_; j++)
        {
            float d = depth.at<int16_t>(i,j);

            // transform the homogeneous point
            pcl::PointXYZ p;

            p.z = double(d) / cam_intrinsics_.camera_factor;
            p.x = (j - cam_intrinsics_.cx) * p.z / cam_intrinsics_.fx;
            p.y = (i - cam_intrinsics_.cy) * p.z / cam_intrinsics_.fy;

            cloud->points[cols*i + j] = p;
        }
    }

   return cloud;
}

// 20 Hz
const float BinderDetector::kLoopRate = 20;
// Minimum number in nn search
const unsigned int BinderDetector::kMinClusterSize = 50;

int main(int argc, char** argv)
{
    ros::init(argc, argv, "binder_detector");
    // Once the last nh is destroyed ros is shutdown
    ros::NodeHandle nh;

    BinderDetector binder_detector;
    binder_detector.Run();
    return 0;
}
