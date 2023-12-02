#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "cmath"
bool processed;
ros::Subscriber sub;
int leg_count;
std::vector<std::pair<float, float>> person_positions;
void positionCallback(const sensor_msgs::LaserScan::ConstPtr &scan_msg)
{
    if (processed) 
    {
        return;
    }
    std::vector<float> ranges = scan_msg->ranges;
    float angle_min = scan_msg->angle_min;
    float angle_increment = scan_msg->angle_increment;

    for (size_t i = 0; i < ranges.size(); ++i) 
    {
        if (std::isinf(ranges[i])) 
        {
            continue;
        }
        float angle = angle_min + (i * angle_increment);
        float x = ranges[i] * std::cos(angle);
        float y = ranges[i] * std::sin(angle);
        
        if (leg_count % 2 == 0) 
        {
            person_positions.push_back(std::make_pair(x, y));
        } 
        else 
        {
            float mid_x = (person_positions.back().first + x) / 2.0;
            float mid_y = (person_positions.back().second + y) / 2.0;

            ROS_INFO("Person %d Position: (x, y) = (%f, %f)", leg_count / 2 + 1, mid_x, mid_y);
        }

        leg_count++;

        if (leg_count == 6) 
        {
            sub.shutdown();
            processed = true;
            break;
        }
    }
    int num_ranges = scan_msg->ranges.size();
    ROS_INFO("Range: %d.", num_ranges);

}

int main(int argc, char **argv)
{
ros::init(argc, argv, "pose_estimator");
ros::NodeHandle n;
sub = n.subscribe("/scan",1, positionCallback);
ros::spin();
return 0;
}