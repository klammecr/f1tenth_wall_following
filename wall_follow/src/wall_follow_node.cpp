#include "rclcpp/rclcpp.hpp"
#include <memory>
#include <string>
#include "sensor_msgs/msg/laser_scan.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "ackermann_msgs/msg/ackermann_drive_stamped.hpp"

// Placeholders for binding functions for pub/sub
using std::placeholders::_1;


class WallFollow : public rclcpp::Node {

public:
    WallFollow() : Node("wall_follow_node")
    {
        // Create ROS subscribers and publishers
        m_sub_scan  = this->create_subscription<sensor_msgs::msg::LaserScan>(
        lidarscan_topic, 10, std::bind(&WallFollow::scan_callback, this, _1));
        m_pub_drive = this->create_publisher<ackermann_msgs::msg::AckermannDriveStamped>(
        drive_topic, 10);
    }

private:
    double m_servo_offset = 0.0;

    // Lookahead distance in meters
    double m_L = 2.0;

    // Desired distance from wall in meters
    double m_desiredDist = 1;

    // Information about the angles
    double m_angle_min = 0.0;
    double m_angle_max = 0.0;
    double m_angle_incr = 0.0;
    double m_max_range  = 0.0;

    // PID CONTROL PARAMS
    double m_kp = 1.;
    double m_kd = 0.1;//0.01;
    double m_ki = 0.0;//0.001;

    // PID errors
    double m_prev_error = 0.0;
    double m_error = 0.0;
    double m_integral = 0.0;

    // Topics
    std::string lidarscan_topic = "scan";
    std::string drive_topic = "drive";

    // Publisher and Subscriber
    // We want to publish the control (steering angle)
    // We want to read the sensor measurements from the lidar
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr m_sub_scan;
    rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr m_pub_drive;

    double get_range(const std::vector<float> range_data, double angle)
    {
        /*
        Simple helper to return the corresponding range measurement at a given angle. Make sure you take care of NaNs and infs.

        Args:
            range_data: single range array from the LiDAR
            angle: between angle_min and angle_max of the LiDAR

        Returns:
            range: range measurement in meters at the given angle
        */

        // TODO: Handle edge case of NaNs and infs
        double range = 0.0;
        if (angle >= m_angle_min && angle <= m_angle_max)
        {
            int range_idx = (angle - m_angle_min) / m_angle_incr;
            range = range_data[range_idx];
        }

        if (std::isinf(range))
            range = 0.0;

        return range;
    }

    double get_error(const std::vector<float> ranges, double desiredDist)
    {
        /*
        Calculates the error to the wall. Follow the wall to the left (going counter clockwise in the Levine loop). You potentially will need to use get_range()

        Args:
            range_data: single range array from the LiDAR
            desiredDist: desired distance to the wall

        Returns:
            error: calculated error
        */

        // Set the previous error
        m_prev_error = m_error;

        double right_robot = -90 * M_PI/180;
        double left_robot  = 90 * M_PI/180; 
        bool rightWall = false;

        double theta =  45.0 * M_PI/180;
        double closeAngle = right_robot;
        
        if (!rightWall)
        {
            theta = theta*-1;
            closeAngle = left_robot;
        }
    
        double farAngle = closeAngle + theta;
        double b = get_range(ranges, closeAngle);
        double a = get_range(ranges, farAngle);

        // DEBUG:
        // std::cout<<"a "<<a<<std::endl;
        // std::cout<<"b "<<b<<std::endl;

        // Calculate the angle between the closest part of the wall and b
        double alpha = std::atan2(a*std::cos(std::abs(theta))-b, a * std::sin(std::abs(theta)));
        double dist  = b * std::cos(alpha);

        //DEBUG:
        std::cout << "Alpha: " << alpha << std::endl;
        std::cout << "Distance to Wall: " << dist << std::endl;

        // Since there is some delay in our control, use a lookahead distance
        double lookaheadDist = dist + m_L * std::sin(alpha);

        // Calculate the error as the 
        
        // When we are about to hit right wall, turn left
        if (rightWall)
            m_error = desiredDist - lookaheadDist;
        else
            m_error = lookaheadDist - desiredDist;
        std::cout << m_error << std::endl;

        return m_error;
    }

    void pid_control(double error)
    {
        /*
        Based on the calculated error, publish vehicle control

        Args:
            error: calculated error
            velocity: desired velocity

        Returns:
            None
        */

        // Use kp, ki & kd to implement a PID controller
        double steering_angle = m_kp * error + m_kd * (error - m_prev_error) + m_ki * m_integral;
        auto drive_msg = ackermann_msgs::msg::AckermannDriveStamped();
        drive_msg.drive.steering_angle = steering_angle;

        // Based on the error, calculate speed of the car
        double velocity = 0.0;
        if (steering_angle >= 0 && steering_angle < 10)
            velocity = 1.5;
        else if (steering_angle >= 10 && steering_angle < 20)
            velocity = 1.0;
        else
            velocity = 0.5;

        // std::cout << "Steering Angle: " << steering_angle << std::endl;
        // std::cout << "Car Speed: " <<velocity << std::endl;
        
        // Put the velocity in the message
        drive_msg.drive.speed = velocity;

        // Publish the message
        m_pub_drive->publish(drive_msg);
    }

    void scan_callback(const sensor_msgs::msg::LaserScan::ConstSharedPtr scan_msg) 
    {
        /*
        Callback function for LaserScan messages. Calculate the error and publish the drive message in this function.

        Args:
            msg: Incoming LaserScan message

        Returns:
            None
        */
        // Get some information about the particular laser scan
        m_angle_min  = scan_msg->angle_min;
        m_angle_max  = scan_msg->angle_max;
        m_angle_incr = scan_msg->angle_increment;
        m_max_range  = scan_msg->range_max;

        //RCLCPP_INFO(this->get_logger(), "My log message %d", m_max_range);
        double error = get_error(scan_msg->ranges, m_desiredDist);

        // Actuate the car with PID
        pid_control(error);

    }

};
int main(int argc, char ** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<WallFollow>());
    rclcpp::shutdown();
    return 0;
}