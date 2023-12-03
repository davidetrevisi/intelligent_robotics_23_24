#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "cmath"
bool processed;
ros::Subscriber sub;
int leg_count = 0;
int person_count = 0;
int range_count[7] = {0};
float mid_x[6] = {0.0};
float mid_y[6] = {0.0};
float person_x[3] = {0.0};
float person_y[3] = {0.0};
std::vector<std::pair<float, float>> leg_ranges[6];
int pose_estimating();
void positionCallback(const sensor_msgs::LaserScan::ConstPtr &scan_msg)
{
    std::vector<float> ranges = scan_msg->ranges;
    float angle_min = scan_msg->angle_min;
    float angle_increment = scan_msg->angle_increment;
    for (int i = 0; i < ranges.size(); i++)
    {
        if (std::isinf(ranges[i]))
        {
            continue;
        }
        else
        {
            float angle = angle_min + (i * angle_increment);
            float x = ranges[i] * std::cos(angle);
            float y = ranges[i] * std::sin(angle);

            leg_ranges[leg_count].push_back(std::make_pair(x, y));
            range_count[leg_count]++;

            if ((std::isinf(ranges[i + 1])) || i == ranges.size() - 1)
            {
                leg_count++;
            }
        }
    }
    pose_estimating();
    ros::shutdown();
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "pose_estimator");
    ros::NodeHandle n;
    sub = n.subscribe("/scan", 1, positionCallback);
    ros::spin();
    return 0;
}

int pose_estimating()
{
    for (int l = 0; l < leg_count; l++)
    {
        for (int j = 0; j < range_count[l]; j++)
        {
            mid_x[l] += leg_ranges[l][j].first;
            mid_y[l] += leg_ranges[l][j].second;
        }
        mid_x[l] = mid_x[l] / range_count[l];
        mid_y[l] = mid_y[l] / range_count[l];

        if ((l % 2 == 0))
        {
            person_count++;
        }
    }

    for (int i = 0; i < person_count; ++i)
    {
        if (i == 0)
        {
            person_x[0] = (mid_x[0] + mid_x[5]) / 2.0;
            person_y[0] = (mid_y[0] + mid_y[5]) / 2.0;
        }
        else if (i == 1)
        {
            person_x[1] = (mid_x[1] + mid_x[2]) / 2.0;
            person_y[1] = (mid_y[1] + mid_y[2]) / 2.0;
        }
        else if (i == 2)
        {
            person_x[2] = (mid_x[3] + mid_x[4]) / 2.0;
            person_y[2] = (mid_y[3] + mid_y[4]) / 2.0;
        }
        ROS_INFO("Person %i Position: (x, y) = (%.2f, %.2f)", i + 1, person_x[i], person_y[i]);
    }

    return 0;
}