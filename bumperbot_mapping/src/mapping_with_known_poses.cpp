#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2/utils.h>

using namespace std::chrono_literals;

class Pose{
    public:
        Pose(int x = 0, int y = 0):x(x), y(x){}
        int x, y;
};

Pose coordinateToPose(const double x, const double y, nav_msgs::msg::MapMetaData &map_info)
{
    Pose pose = Pose();
    pose.x = std::round((x - map_info.origin.position.x)/map_info.resolution);
    pose.y = std::round((y - map_info.origin.position.y)/map_info.resolution);

    return pose;
}

bool poseOnMap(const Pose &pose, nav_msgs::msg::MapMetaData &map_info)
{
    return pose.x < static_cast<int>(map_info.width) && pose.x >= 0 && 
        pose.y < static_cast<int>(map_info.height) && pose.y >=0;
}

unsigned int poseToCell(const Pose &pose, nav_msgs::msg::MapMetaData &map_info)
{
    return map_info.width * pose.y + pose.x;
}

class MappingWithKnownPoses: public rclcpp::Node
{
public:
    MappingWithKnownPoses(): Node("mapping_with_known_poses")
    {
        
        declare_parameter<double>("width", 50.0);
        declare_parameter<double>("height", 50.0);
        declare_parameter<double>("resolution", 0.1);

        double width = get_parameter("width").as_double();
        double height = get_parameter("height").as_double();
        double resolution = get_parameter("resolution").as_double();

        map_pub_ = create_publisher<nav_msgs::msg::OccupancyGrid>(
            "map", 
            1
        );

        scan_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
            "scan", 
            10, 
            std::bind(&MappingWithKnownPoses::scanCallback, this, std::placeholders::_1)
        );

        timer_ = create_wall_timer(1s, std::bind(&MappingWithKnownPoses::timerCallback, this));
        
        map_.header.frame_id = "odom";
        map_.info.width = std::round(width * resolution);
        map_.info.height = std::round(height * resolution);
        map_.info.resolution = resolution;
        map_.info.origin.position.x = -std::round(width / 2.0);
        map_.info.origin.position.y = -std::round(height / 2.0);
        map_.data = std::vector<int8_t>(map_.info.width * map_.info.height, -1.0);

        tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);


    }

private:
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr map_pub_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
    rclcpp::TimerBase::SharedPtr timer_;
    nav_msgs::msg::OccupancyGrid map_;
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    

    void scanCallback(const sensor_msgs::msg::LaserScan &msg){
        geometry_msgs::msg::TransformStamped t;
        try{
            t = tf_buffer_->lookupTransform(map_.header.frame_id, msg.header.frame_id, tf2::TimePointZero);
        } catch(const tf2::TransformException &exc){
            RCLCPP_ERROR(get_logger(), "Unable to transform between /odom and /base_footprint");
            return;
        }

        Pose robot_pose = coordinateToPose(t.transform.translation.x, t.transform.translation.y, map_.info);

        if(not poseOnMap){
            RCLCPP_ERROR(get_logger(), "The robot is out of the map!");
            return;
        }

        unsigned int robot_cell = poseToCell(robot_pose, map_.info);
        map_.data.at(robot_cell) = 100;


    }

    void timerCallback(){
        map_.header.stamp = get_clock()->now();
        map_pub_->publish(map_);
    }
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MappingWithKnownPoses>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}