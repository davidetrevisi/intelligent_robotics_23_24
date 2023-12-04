#include <ros/ros.h>

#include <sensor_msgs/LaserScan.h>

#include <opencv2/core.hpp>

#include <cmath>

// Global variables
int leg_count = 0;
int person_count = 0;
int range_count[6] = {0};
float mid_x[6] = {0.0};
float mid_y[6] = {0.0};
float person_x[3] = {0.0};
float person_y[3] = {0.0};

std::vector<std::pair<float, float>> leg_ranges[6];
std::vector<cv::Point2f> leg_ranges_cv;

// Utility functions
int pose_estimating();
void array_shifting(float array[], int size);

// Message callback function
void positionCallback(const sensor_msgs::LaserScan::ConstPtr &scan_msg)
{
    // Save the message values
    std::vector<float> ranges = scan_msg->ranges;
    float angle_min = scan_msg->angle_min;
    float angle_increment = scan_msg->angle_increment;

    // Cycle through all the values
    for (int i = 0; i < ranges.size(); i++)
    {
        // If there is a leg (something is detected, so the value is not infinity)
        if (!std::isinf(ranges[i]))
        {
            // Convert the value to [x, y] coordinates
            float angle = angle_min + (i * angle_increment);
            float x = ranges[i] * std::cos(angle);
            float y = ranges[i] * std::sin(angle);

            // Add the point to the corresponding leg
            leg_ranges[leg_count].push_back(std::make_pair(x, y));

            // Add the point to the values for the kmeans algorithm
            cv::Point2f point(x, y);
            leg_ranges_cv.push_back(point);

            // Increment the counter
            range_count[leg_count]++;

            // If we are at the last point of the leg, increase the leg counter
            if ((std::isinf(ranges[i + 1])) || i == ranges.size() - 1)
            {
                leg_count++;
            }
        }
    }

    // Run kmeans clustering on the points
    cv::Mat centers;
    std::vector<int> labels;

    cv::kmeans(leg_ranges_cv, leg_count / 2, labels, cv::TermCriteria(cv::TermCriteria::MAX_ITER + cv::TermCriteria::EPS, 100, 0.001), 5, cv::KMEANS_PP_CENTERS, centers);

    // Show the output
    ROS_INFO("Exercise 4.3 - Create a ROS node to compute the position of the people:");
    pose_estimating();
    ROS_INFO("Exercise 4.4 - Generalize the ROS node to compute the position of any number of people:");
    ROS_INFO_STREAM("Position of each person: (x, y)= \n"
                    << centers);
    ros::shutdown();
}

int main(int argc, char **argv)
{
    // Create and initialize the node
    ros::init(argc, argv, "exercise_4");
    ros::NodeHandle n;

    // Subscribe to the topic
    ros::Subscriber sub = n.subscribe("/scan", 1, positionCallback);

    // Spin forever without rate limit
    ros::spin();

    return 0;
}

int pose_estimating()
{
    // Cycle through all the legs
    for (int l = 0; l < leg_count; l++)
    {
        // Cycle through all the points
        for (int j = 0; j < range_count[l]; j++)
        {
            mid_x[l] += leg_ranges[l][j].first;
            mid_y[l] += leg_ranges[l][j].second;
        }
        
        // Compute the average [x, y] for the leg
        mid_x[l] = mid_x[l] / range_count[l];
        mid_y[l] = mid_y[l] / range_count[l];

        // Every two legs, we have a person
        if ((l % 2 == 0))
        {
            person_count++;
        }
    }

    // Shift the array to account for the person over the 'zero' marker:
    // two legs at both ends of the array but in reality they are close
    array_shifting(mid_x, (sizeof(mid_x) / sizeof(mid_x[0])));
    array_shifting(mid_y, (sizeof(mid_x) / sizeof(mid_y[0])));

    // Print the average for each person
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
