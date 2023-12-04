#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <opencv2/core.hpp>
#include <cmath>

ros::Subscriber sub;
int leg_count = 0;
int person_count = 0;
int range_count[6] = {0};
float mid_x[6] = {0.0};
float mid_y[6] = {0.0};
float person_x[3] = {0.0};
float person_y[3] = {0.0};
std::vector<std::pair<float, float>> leg_ranges[6];
std::vector<cv::Point2f> leg_ranges_cv;

int pose_estimating();
void array_shifting(float array[], int size);
void positionCallback(const sensor_msgs::LaserScan::ConstPtr &scan_msg)
{
    std::vector<float> ranges = scan_msg->ranges;
    float angle_min = scan_msg->angle_min;
    float angle_increment = scan_msg->angle_increment;
    for (int i = 0; i < ranges.size(); i++)
    {
        if (!std::isinf(ranges[i]))
        {
            float angle = angle_min + (i * angle_increment);
            float x = ranges[i] * std::cos(angle);
            float y = ranges[i] * std::sin(angle);

            leg_ranges[leg_count].push_back(std::make_pair(x, y));

            cv::Point2f point(x, y);
            leg_ranges_cv.push_back(point);

            range_count[leg_count]++;

            if ((std::isinf(ranges[i + 1])) || i == ranges.size() - 1)
            {
                leg_count++;
            }
        }
    }

    cv::Mat centers;
    std::vector<int> labels;

    cv::kmeans(leg_ranges_cv, leg_count / 2, labels, cv::TermCriteria(cv::TermCriteria::MAX_ITER + cv::TermCriteria::EPS, 100, 0.001), 5, cv::KMEANS_PP_CENTERS, centers);

    ROS_INFO("Exercise 4.3 - Create a ROS node to compute the position of the people:");
    pose_estimating();
    ROS_INFO("Exercise 4.4 - Generalize the ROS node to compute the position of any number of people:");
    ROS_INFO_STREAM("Position of each person: (x, y)= \n"
                    << centers);
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

    array_shifting(mid_x, (sizeof(mid_x) / sizeof(mid_x[0])));
    array_shifting(mid_y, (sizeof(mid_x) / sizeof(mid_y[0])));

    for (int i = 0; i < person_count; ++i)
    {
        person_x[i] = (mid_x[i * 2] + mid_x[i * 2 + 1]) / 2.0;
        person_y[i] = (mid_y[i * 2] + mid_y[i * 2 + 1]) / 2.0;
        ROS_INFO("Person %i Position: (x, y) = (%f, %f)", i + 1, person_x[i], person_y[i]);
    }

    return 0;
}

void array_shifting(float array[], int size)
{
    float temp = array[size - 1];
    for (int i = size - 1; i > 0; --i)
    {
        array[i] = array[i - 1];
    }
    array[0] = temp;
}
