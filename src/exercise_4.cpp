#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "cmath"
bool processed;
ros::Subscriber sub;
int leg_count=0;
int person_count=0;
int range_count=0;
float mid_x[2]={0, 0};
float mid_y[2]={0,0};
float person_x[3]{0,0,0};
float person_y[3]{0,0,0};
std::vector<std::pair<float, float>> leg_ranges;
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
            ranges[i]=0.0;
        }
        else
        {
            float angle = angle_min + (i * angle_increment);
            float x = ranges[i] * std::cos(angle);
            float y = ranges[i] * std::sin(angle);
        
            leg_ranges.push_back(std::make_pair(x, y));
            range_count++;

            if ((ranges[i+1]==0.0)||(i == (ranges.size() - 1))||(std::isinf(ranges[i+1])) ) 
            {
                for(int j=0; j< range_count;j++)
                {
                    mid_x[leg_count] += leg_ranges[j].first;
                    mid_y[leg_count] += leg_ranges[j].second;
                }
                    mid_x[leg_count]= mid_x[leg_count]/range_count;
                    mid_y[leg_count]= mid_y[leg_count]/range_count;
                    ROS_INFO("Person %i leg %i X: %f,Y: %f", person_count+1,leg_count+1,mid_x[leg_count],mid_y[leg_count]);
                leg_ranges.clear();
                range_count = 0;
                leg_count++;  
            
                if (leg_count==2) 
                {
                    person_x[person_count]= (mid_x[leg_count-2]+ mid_x[leg_count-1])/2.0;
                    person_y[person_count]= (mid_y[leg_count-2]+ mid_y[leg_count-1])/2.0;
                    ROS_INFO("Person %u Position: (x, y) = (%.2f, %.2f)", person_count + 1, person_x[person_count], person_y[person_count]);
                    mid_x[0]=0.0;
                    mid_x[0]=0.0;
                    mid_x[1]=0.0;
                    mid_x[1]=0.0;
                    leg_count=0;
                    person_count++;
                }
                

                if (person_count==3)
                {
                    sub.shutdown();
                    processed = true;
                    break;
                } 
            }
        }
    }
    //int num_ranges = scan_msg->ranges.size();
    //ROS_INFO("Range: %d.", num_ranges);

}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "pose_estimator");
    ros::NodeHandle n;
    sub = n.subscribe("/scan",1, positionCallback);
    ros::spin();
    return 0;
}