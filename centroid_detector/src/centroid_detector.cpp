#include "centroid_detector/centroid_detector.h"

BinderDetector::BinderDetector() : loop_rate_(NULL), tf_listener_(NULL), pc_sub_(NULL), new_pc_(false), server_(NULL)
{
    // Setup loop rate
    loop_rate_ = new ros::Rate(kLoopRate);

    // Grab ros params
    ros::NodeHandle private_nh("~");
    private_nh.param<float>("min_pc_x", min_pc_x_, 0);
    private_nh.param<float>("max_pc_x", max_pc_x_, 1);
    private_nh.param<float>("min_pc_y", min_pc_y_, -1);
    private_nh.param<float>("max_pc_y", max_pc_y_, 1);
    private_nh.param<float>("min_pc_z", min_pc_z_, 0.82);
    private_nh.param<float>("max_pc_z", max_pc_z_, 1.016);
    private_nh.param<float>("nearest_neighbor_radius", nearest_neighbor_radius_, 0.03);
    private_nh.param<float>("min_centroid_seperation", min_centroid_seperation_, 0.05);

    // Setup tf
    tf_listener_ = new tf::TransformListener();

    // Setup service and subscribers
    ros::NodeHandle nh;
    pc_sub_ = new ros::Subscriber();
    server_ = new ros::ServiceServer;
    *server_ = nh.advertiseService("/centroid_detector", &BinderDetector::ServiceCB, this);
}

BinderDetector::~BinderDetector()
{
    if (loop_rate_)
        delete loop_rate_;
    if (tf_listener_)
        delete tf_listener_;
    if (pc_sub_)
        delete pc_sub_;
    if (server_)
        delete server_;
}

bool BinderDetector::ServiceCB(centroid_detector_msgs::DetectCentroid::Request& req, centroid_detector_msgs::DetectCentroid::Response& res)
{
    // Try ten times to find a binder
    Eigen::Vector4f centroid;
    bool found = false;
    for (int i = 0; i < 1000; ++i)
    {
        if (DetectCentroid(&centroid))
        {
            found = true;
            break;
        }
        ROS_WARN("Failed to find a binder %d times", i + 1);
    }

    // Construct a pose
    res.centroid.position.x = centroid[0];
    res.centroid.position.y = centroid[1];
    res.centroid.position.z = centroid[2];
    res.centroid.orientation.x = 0;
    res.centroid.orientation.y = 0;
    res.centroid.orientation.z = 0;
    res.centroid.orientation.w = 1;

    return found;
}

void BinderDetector::PCCallback(const sensor_msgs::PointCloud2::ConstPtr& msg)
{
    lattest_pc_ = *msg;
    new_pc_ = true;
}

bool BinderDetector::DetectCentroid(Eigen::Vector4f* result)
{
    // Wait for a new point cloud
    //  Subscribe to pc topic
    ros::NodeHandle nh;
    *pc_sub_ = nh.subscribe("/head_camera/depth/points", 1, &BinderDetector::PCCallback, this);
    new_pc_ = false;
    while (ros::ok())
    {
        if (new_pc_)
            break;
        ros::spinOnce();
        loop_rate_->sleep();
    }
    // Unsubscribe from pc topic
    pc_sub_->shutdown();

    // Transform pc to base_link
    sensor_msgs::PointCloud2 tfed_pc;
    try
    {
        tf_listener_->waitForTransform("/base_link", lattest_pc_.header.frame_id, ros::Time(0), ros::Duration(10.0));
        pcl_ros::transformPointCloud("/base_link", lattest_pc_, tfed_pc, *tf_listener_);
    }
    catch (tf::TransformException ex)
    {
        ROS_ERROR("Failed to transform point cloud to /base_link because: %s", ex.what());
        return false;
    }

    // Transform to pcl data type
    pcl::PCLPointCloud2::Ptr temp(new pcl::PCLPointCloud2);
    pcl_conversions::toPCL(tfed_pc, *temp);
    pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_tfed_pc(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromPCLPointCloud2(*temp, *pcl_tfed_pc);

    // Crop the pc
    pcl::CropBox<pcl::PointXYZ> crop_box(true);
    Eigen::Vector4f min_pt(min_pc_x_, min_pc_y_, min_pc_z_, 1);
    Eigen::Vector4f max_pt(max_pc_x_, max_pc_y_, max_pc_z_, 1);
    crop_box.setMin(min_pt);
    crop_box.setMax(max_pt);
    crop_box.setInputCloud(pcl_tfed_pc);
    std::vector<int> indices;
    crop_box.filter(indices);
    pcl::PointCloud<pcl::PointXYZ>::Ptr croped_pc(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::copyPointCloud(*pcl_tfed_pc, indices, *croped_pc);

    // Construct kdtree
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
    tree->setInputCloud(croped_pc);

    // Perform euclidian segmentation
    std::vector<pcl::PointIndices> cluster_indicies;
    pcl::EuclideanClusterExtraction<pcl::PointXYZ> segmenter;
    segmenter.setClusterTolerance(nearest_neighbor_radius_);
    segmenter.setMinClusterSize(kMinClusterSize);
    segmenter.setMaxClusterSize(999999);
    segmenter.setInputCloud(croped_pc);
    segmenter.extract(cluster_indicies);

    // Get centroids
    std::vector<Eigen::Vector4f> centroids;
    int i = 0;
    for (auto&& indicies : cluster_indicies)
    {
        Eigen::Vector4f centroid;
        pcl::compute3DCentroid(*croped_pc, indicies, centroid);

        // Combine centroids who are likely from the same binder
        //  This is done using the centroid y value
        //  No two binders should occupy the same y space
        //  If they are then keep the centroid with the smalles x value
        bool add_centroid = true;
        for (auto it = centroids.begin(); it < centroids.end(); ++it)
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

    ROS_INFO("--------------------------------------");
    for (auto&& centroid : centroids)
    {
        ROS_INFO("Binder centroid: [%f,\t%f,\t%f]", centroid[0], centroid[1], centroid[2]);
    }

    // Return fail if none were found
    if (!centroids.size())
    {
        ROS_WARN("No binder centroids found");
        return false;
    }

    // Find the center most binder (y value clossest to 0)
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
