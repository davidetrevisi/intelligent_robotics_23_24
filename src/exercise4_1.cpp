#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "cmath"
bool processed;
ros::Subscriber sub;
int leg_count=0;
int person_count=0;
int range_count[7]={ 0, 0, 0, 0, 0, 0, 0 };
float mid_x[6];
float mid_y[6];
float person_x[3];
float person_y[3];
int pose_estimating();
std::vector<std::pair<float, float>> leg_ranges[6];
void positionCallback(const sensor_msgs::LaserScan::ConstPtr &scan_msg)
{
    if (processed) 
    {
        return;
    }
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

            if ((std::isinf(ranges[i+1]))||(i==719)) 
            {
                leg_count++;
                if (leg_count==6)
                {
                    sub.shutdown();
                    break;
                }
            }
        }
    }
    ROS_INFO("Legs %d ", leg_count);
    pose_estimating();
}

int pose_estimating(){
    for (int l = 0; l <= leg_count; l++)
    {
        if(l!=6)
        {
            for(int j=0; j<= range_count[l];range_count[l]++)
            {
                mid_x[l] += leg_ranges[l][j].first;
                mid_y[l] += leg_ranges[l][j].second;
            }
                mid_x[l]= mid_x[l]/range_count[l];
                mid_y[l]= mid_y[l]/range_count[l];
        }

        if ((l%2==0)&& (l!=0)) 
        {
            person_x[person_count]= (mid_x[l-2]+ mid_x[l-1])/2.0;
            person_y[person_count]= (mid_y[l-2]+ mid_y[l-1])/2.0;
            ROS_INFO("Person %u Position: (x, y) = (%f, %f)", person_count, person_x[person_count], person_y[person_count]);
            person_count++;
        }   
        if (person_count==3)
        {
            processed=true;
            break;
        } 
    }
return 0;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "pose_estimator");
    ros::NodeHandle n;
    sub = n.subscribe("/scan",1, positionCallback);
    ros::spin();
    return 0;
}