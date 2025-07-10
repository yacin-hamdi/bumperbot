#include "bumperbot_localization/odometry_motion_model.hpp"
#include <geometry_msgs/msg/pose.hpp>
#include <tf2/utils.h>
#include <cmath>
#include <random>

double angle_diff(double a, double b)
{
    a = atan2(sin(a), cos(a));
    b = atan2(sin(b), cos(b));

    double d1 = a - b;
    double d2 = 2 * M_PI - fabs(d1);
    if (d1 > 0){
        d2 *= -1.0;
    }
    if (fabs(d1) < fabs(d2)){
        return d1;
    }else{
        return d2;
    }
}


using std::placeholders::_1;

OdometryMotionModel::OdometryMotionModel(const std::string & name): 
    Node(name),
    alpha1(0.0),
    alpha2(0.0),
    alpha3(0.0),
    alpha4(0.0),
    nb_samples(300),
    last_odom_x_(0.0),
    last_odom_y_(0.0),
    last_odom_theta_(0.0),
    is_first_odom_(true)
{
    declare_parameter("alpha1", 0.1);
    declare_parameter("alpha2", 0.1);
    declare_parameter("alpha3", 0.1);
    declare_parameter("alpha4", 0.1);
    declare_parameter("nb_samples", 300);

    alpha1 = get_parameter("alpha1").as_double();
    alpha2 = get_parameter("alpha2").as_double();
    alpha3 = get_parameter("alpha3").as_double();
    alpha4 = get_parameter("alpha4").as_double();
    nb_samples = get_parameter("nb_samples").as_int();


    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>("bumperbot_controller/odom",
                                                            10,
                                                            std::bind(&OdometryMotionModel::odomCallback, this, _1));
    pose_array_pub_ = create_publisher<geometry_msgs::msg::PoseArray>("odometry_motion_model/samples", 10);

    if (nb_samples >= 0)
    {
        samples.poses = std::vector<geometry_msgs::msg::Pose>(nb_samples, geometry_msgs::msg::Pose());

    }else{
        RCLCPP_FATAL_STREAM(get_logger(), "Invalid number of samples:"<< nb_samples);
        return;
    }

}


void OdometryMotionModel::odomCallback(const nav_msgs::msg::Odometry &odom)
{
    tf2::Quaternion q(odom.pose.pose.orientation.x, 
                        odom.pose.pose.orientation.y,
                        odom.pose.pose.orientation.z,
                        odom.pose.pose.orientation.w);

    tf2::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);

    if (is_first_odom_)
    {
        last_odom_x_ = odom.pose.pose.position.x;
        last_odom_y_ = odom.pose.pose.position.y;
        last_odom_theta_ = yaw;
        samples.header.frame_id = odom.header.frame_id;
        is_first_odom_ = false;
        return;
    }

    double odom_x_increment = odom.pose.pose.position.x - last_odom_x_;
    double odom_y_increment = odom.pose.pose.position.y - last_odom_y_;
    double odom_theta_increment = angle_diff(yaw, last_odom_theta_);
    double delta_rot1 = 0.0;
    if(sqrt(std::pow(odom_x_increment, 2) + std::pow(odom_y_increment, 2)) > 0.01)
    {
        delta_rot1 = angle_diff(atan2(odom_y_increment, odom_x_increment), yaw);
    }
    double delta_trasl = sqrt(std::pow(odom_x_increment, 2) + std::pow(odom_y_increment, 2));
    double delta_rot2 = angle_diff(odom_theta_increment, delta_rot1);

    double rot1_variance = alpha1 * delta_rot1 + alpha2 * delta_trasl;
    double trasl_variance = alpha3 * delta_trasl + alpha4 * (delta_rot1 + delta_rot2);
    double rot2_variance = alpha1 * delta_rot2 + alpha2 * delta_trasl;

    unsigned seed = std::chrono::system_clock::now().time_since_epoch().count();
    std::default_random_engine noise_generator(seed);
    std::normal_distribution<double> rot1_noise(0.0, rot1_variance);
    std::normal_distribution<double> rot2_noise(0.0, rot2_variance);
    std::normal_distribution<double> trasl_noise(0.0, trasl_variance);

    for(auto &sample: samples.poses){
        double delta_rot1_noise = angle_diff(delta_rot1, rot1_noise(noise_generator));
        double delta_rot2_noise = angle_diff(delta_rot2, rot2_noise(noise_generator));
        double delta_trasl_noise = delta_trasl - trasl_noise(noise_generator);

        tf2::Quaternion sample_q(sample.orientation.x, 
                                    sample.orientation.y, 
                                    sample.orientation.z, 
                                    sample.orientation.w);
        tf2::Matrix3x3 sample_m(q);
        double sample_roll, sample_pitch, sample_yaw;
        sample_m.getRPY(sample_roll, sample_pitch, sample_yaw);

        sample.position.x += delta_trasl_noise * std::cos(sample_yaw + delta_rot1_noise);
        sample.position.y += delta_trasl_noise * std::sin(sample_yaw + delta_rot1_noise);

        tf2::Quaternion q;
        q.setRPY(0.0, 0.0, sample_yaw + delta_rot1_noise + delta_rot2_noise);
        sample.orientation.x = q.getX();
        sample.orientation.y = q.getY();
        sample.orientation.z = q.getZ();
        sample.orientation.w = q.getW();
    }

    last_odom_x_ = odom.pose.pose.position.x;
    last_odom_y_ = odom.pose.pose.position.y;
    last_odom_theta_ = yaw;

    pose_array_pub_->publish(samples);

}

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<OdometryMotionModel>("odometry_motion_model");
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}

