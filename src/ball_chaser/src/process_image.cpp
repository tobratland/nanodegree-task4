#include "ros/ros.h"
#include "ball_chaser/command_robot.h"
#include <sensor_msgs/Image.h>

class ImageProcessor
{
public:
    ImageProcessor(ros::NodeHandle *nodeHandle) : n_(*nodeHandle)
    {
        this->client_ = n_.serviceClient<ball_chaser::command_robot>("/ball_chaser/command_robot");
        this->sub1_ = n_.subscribe("/camera/rgb/image_raw", 10, &ImageProcessor::process_image_callback, this);
    }

private:
    enum Position
    {
        left,
        right,
        centre,
        no_ball
    };

    ros::ServiceClient client_;
    ros::NodeHandle n_;
    ros::Subscriber sub1_;

    void drive_robot(float lin_x, float ang_z)
    {
        ball_chaser::command_robot command;
        command.request.linear_x = lin_x;
        command.request.angular_z = ang_z;

        if (!client_.call(command))
            ROS_ERROR("failed to call service command_robot");
    }

    //finds the position of, or lack of, the ball in the image provided.
    Position find_position(const sensor_msgs::Image img)
    {
        uint8_t white_pixel = 255;
        Position pos = no_ball;

        for (int i = 0; i < img.height * img.step; i += 3)
        {
            uint8_t red = img.data[i];
            uint8_t green = img.data[i + 1];
            uint8_t blue = img.data[i + 2];

            if (red == white_pixel && green == white_pixel && blue == white_pixel)
            {
                int col = i % img.step;
                if (col < img.step * 0.4)
                {
                    pos = left;
                }
                else if (col > img.step * 0.6)
                {
                    pos = right;
                }
                else
                {
                    pos = centre;
                }
            }
        }

        return pos;
    }

    void process_image_callback(const sensor_msgs::Image img)
    {

        Position pos = find_position(img);

        switch (pos)
        {
        case left:
            ROS_INFO("Found ball left, requesting turn");
            drive_robot(0.0, 0.3);
            break;
        case right:
            ROS_INFO("Found ball right, requesting turn");
            drive_robot(0.0, -0.3);
            break;
        case centre:
            ROS_INFO("Found ball centre, requesting drive");
            drive_robot(0.5, 0.0);
            break;
        default: //no ball
            ROS_INFO("Found no ball, requesting drive");
            drive_robot(0.0, 0.0);
        }
    }
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "process_image");
    ros::NodeHandle n;

    ImageProcessor imgProcessor(&n);

    ros::spin();

    return 0;
}
