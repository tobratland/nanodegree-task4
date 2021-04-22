#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "ball_chaser/command_robot.h"

class DriveBot
{
public:
    DriveBot(ros::NodeHandle *nodeHandle) : n_(*nodeHandle)
    {
        motor_command_publisher_ = n_.advertise<geometry_msgs::Twist>("/cmd_vel", 10);
        this -> ss = n_.advertiseService("/ball_chaser/command_robot", &DriveBot::handle_drive_request, this);
    }

private:
    ros::Publisher motor_command_publisher_;
    geometry_msgs::Twist motor_command;
    ros::NodeHandle n_;
    ros::ServiceServer ss;

    bool handle_drive_request(ball_chaser::command_robot::Request &req,
                              ball_chaser::command_robot::Response &res)
    {
        ROS_INFO("Command robot request received - linear:%1.2f, angular:%1.2f", (float)req.linear_x, (float)req.angular_z);

        motor_command.linear.x = req.linear_x;
        motor_command.angular.z = req.angular_z;

        motor_command_publisher_.publish(motor_command);

        ros::Duration(0.5).sleep();

        res.angular_z = motor_command.angular.z;
        res.linear_x = motor_command.linear.x;

        return true;
    }
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "drive_bot");
    ros::NodeHandle n;

    DriveBot driveBot(&n);

    ros::spin();

    return 0;
}