/**
 * @file nav2_localization_adapter_node.cpp
 * @brief Adapter node to bridge robots_localization output with Nav2
 * 
 * This node subscribes to the odometry from robots_localization_node and
 * publishes the map->odom transform required by Nav2.
 */

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_ros/transform_broadcaster.h>

class Nav2LocalizationAdapter : public rclcpp::Node
{
public:
    Nav2LocalizationAdapter() : Node("nav2_localization_adapter")
    {
        // Declare parameters
        this->declare_parameter<std::string>("odom_topic", "odometry");
        this->declare_parameter<std::string>("map_frame", "map");
        this->declare_parameter<std::string>("odom_frame", "odom");
        this->declare_parameter<std::string>("base_frame", "base_link");
        this->declare_parameter<bool>("publish_pose", true);
        this->declare_parameter<double>("tf_rate_hz", 10.0);  // 10Hz 发布 map->odom

        // Get parameters
        odom_topic_ = this->get_parameter("odom_topic").as_string();
        map_frame_ = this->get_parameter("map_frame").as_string();
        odom_frame_ = this->get_parameter("odom_frame").as_string();
        base_frame_ = this->get_parameter("base_frame").as_string();
        publish_pose_ = this->get_parameter("publish_pose").as_bool();
        tf_rate_hz_ = this->get_parameter("tf_rate_hz").as_double();

        tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

        // Subscribe to odometry from robots_localization_node
        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            odom_topic_, 10,
            std::bind(&Nav2LocalizationAdapter::odomCallback, this, std::placeholders::_1));

        // Publisher for pose (Nav2 compatible)
        if (publish_pose_) {
            pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
                "pose", 10);
        }

        // Prepare static map->odom transform (identity) and publish at fixed rate
        static_transform_.header.frame_id = map_frame_;
        static_transform_.child_frame_id = odom_frame_;
        static_transform_.transform.translation.x = 0.0;
        static_transform_.transform.translation.y = 0.0;
        static_transform_.transform.translation.z = 0.0;
        static_transform_.transform.rotation.w = 1.0;
        static_transform_.transform.rotation.x = 0.0;
        static_transform_.transform.rotation.y = 0.0;
        static_transform_.transform.rotation.z = 0.0;

        tf_timer_ = this->create_wall_timer(
            std::chrono::duration<double>(1.0 / tf_rate_hz_),
            std::bind(&Nav2LocalizationAdapter::publishStaticTf, this));

        RCLCPP_INFO(this->get_logger(), "Nav2 Localization Adapter started");
        RCLCPP_INFO(this->get_logger(), "  Subscribing to: %s", odom_topic_.c_str());
        RCLCPP_INFO(this->get_logger(), "  Publishing TF (static): %s -> %s at %.1f Hz", map_frame_.c_str(), odom_frame_.c_str(), tf_rate_hz_);
    }

private:
    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        // Publish odom -> base_link using the odometry pose (odom frame)
        publishOdomToBaseTf(msg);

        // Publish pose (pose in map frame)
        if (publish_pose_) {
            publishPose(msg);
        }
    }

    void publishStaticTf()
    {
        static_transform_.header.stamp = this->get_clock()->now();
        tf_broadcaster_->sendTransform(static_transform_);
    }

    void publishPose(const nav_msgs::msg::Odometry::SharedPtr& odom_msg)
    {
        geometry_msgs::msg::PoseWithCovarianceStamped pose_msg;
        pose_msg.header.stamp = odom_msg->header.stamp;
        pose_msg.header.frame_id = map_frame_;
        pose_msg.pose = odom_msg->pose;
        pose_pub_->publish(pose_msg);
    }

    void publishOdomToBaseTf(const nav_msgs::msg::Odometry::SharedPtr& odom_msg)
    {
        geometry_msgs::msg::TransformStamped tf_msg;
        tf_msg.header.stamp = odom_msg->header.stamp;
        tf_msg.header.frame_id = odom_frame_;
        tf_msg.child_frame_id = base_frame_;
        tf_msg.transform.translation.x = odom_msg->pose.pose.position.x;
        tf_msg.transform.translation.y = odom_msg->pose.pose.position.y;
        tf_msg.transform.translation.z = odom_msg->pose.pose.position.z;
        tf_msg.transform.rotation = odom_msg->pose.pose.orientation;
        tf_broadcaster_->sendTransform(tf_msg);
    }

    // Parameters
    std::string odom_topic_;
    std::string map_frame_;
    std::string odom_frame_;
    std::string base_frame_;
    bool publish_pose_;
    double tf_rate_hz_;

    // TF
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    geometry_msgs::msg::TransformStamped static_transform_;
    rclcpp::TimerBase::SharedPtr tf_timer_;

    // Subscribers and Publishers
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pose_pub_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<Nav2LocalizationAdapter>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
