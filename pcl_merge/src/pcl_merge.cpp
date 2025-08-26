#include "pcl_merge.hpp"

PCLMergeNode::PCLMergeNode() : Node("pcl_merge_node")
{
    // Declare parameters
    this->declare_parameter("output_frame", "base_link");
    this->declare_parameter("output_topic", "pcl_merged");
    this->declare_parameter("input_topics", std::vector<std::string>({
        "scan_pointcloud", 
        "/camera_depth_top/camera_depth/points"
    }));

    // Get parameters
    output_frame_ = this->get_parameter("output_frame").as_string();
    output_topic = this->get_parameter("output_topic").as_string();
    input_topics_ = this->get_parameter("input_topics").as_string_array();

    // Resize cloud storage for multiple inputs
    clouds_.resize(input_topics_.size(), nullptr);

    // Subscribe to multiple input topics
    for (size_t i = 0; i < input_topics_.size(); i++) {
        auto callback = [this, i](const sensor_msgs::msg::PointCloud2::SharedPtr msg) { 
            cloud_callback(i, msg); 
        };
        subscribers_.push_back(
            this->create_subscription<sensor_msgs::msg::PointCloud2>(
                input_topics_[i], 10, callback
            )
        );
    }

    // Publisher for merged cloud
    pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(output_topic, 10);

    // TF listener for transforming point clouds
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    // Timer to periodically merge and publish (~30Hz)
    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(33), 
        std::bind(&PCLMergeNode::timer_callback, this)
    );
}

// Callback for each point cloud
void PCLMergeNode::cloud_callback(size_t index, const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
    if (index >= clouds_.size())
        return;

    // Get transform from input frame to output frame
    geometry_msgs::msg::TransformStamped transform;
    try {
        transform = tf_buffer_->lookupTransform(output_frame_, msg->header.frame_id, rclcpp::Time(0));
    } catch (tf2::TransformException &ex) {
        RCLCPP_WARN(this->get_logger(), "Could not transform point cloud [%zu]: %s", index, ex.what());
        return;
    }

    // Directly convert PointCloud2 to pcl::PointCloud<pcl::PointXYZI>
    pcl::PointCloud<pcl::PointXYZI>::Ptr pcl_cloud(new pcl::PointCloud<pcl::PointXYZI>);
    try {
        pcl::fromROSMsg(*msg, *pcl_cloud);
    } catch (std::exception &e) {
        RCLCPP_WARN(this->get_logger(), "Failed to convert PointCloud2 to PointXYZI: %s", e.what());
        return;
    }

    // Transform cloud to output frame
    pcl::PointCloud<pcl::PointXYZI>::Ptr pcl_cloud_transformed(new pcl::PointCloud<pcl::PointXYZI>);
    try {
        pcl_ros::transformPointCloud(*pcl_cloud, *pcl_cloud_transformed, transform);
    } catch (std::exception &ex) {
        RCLCPP_WARN(this->get_logger(), "Error during transform: %s", ex.what());
        return;
    }

    // Store transformed cloud
    clouds_[index] = pcl_cloud_transformed;
}

// Timer callback to merge and publish clouds
void PCLMergeNode::timer_callback() {
    bool any_valid = false;
    for (const auto &cloud : clouds_) {
        if (cloud != nullptr) {
            any_valid = true;
            break;
        }
    }
    if (!any_valid) return;  // no valid clouds yet

    // Merge clouds
    pcl::PointCloud<pcl::PointXYZI>::Ptr merged_cloud(new pcl::PointCloud<pcl::PointXYZI>);
    for (const auto& cloud : clouds_) {
        if (cloud) {
            *merged_cloud += *cloud;
        }
    }

    // Downsample merged cloud
    pcl::VoxelGrid<pcl::PointXYZI> voxel_filter;
    voxel_filter.setInputCloud(merged_cloud);
    voxel_filter.setLeafSize(0.0005f, 0.0005f, 0.0005f);
    pcl::PointCloud<pcl::PointXYZI>::Ptr downsampled_cloud(new pcl::PointCloud<pcl::PointXYZI>);
    voxel_filter.filter(*downsampled_cloud);

    // Publish merged cloud
    sensor_msgs::msg::PointCloud2 output_msg;
    pcl::toROSMsg(*downsampled_cloud, output_msg);
    output_msg.header.frame_id = output_frame_;
    pub_->publish(output_msg);

    RCLCPP_INFO_ONCE(this->get_logger(), "Publishing downsampled merged PointXYZI cloud at ~30Hz.");
}

// Main
int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<PCLMergeNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
