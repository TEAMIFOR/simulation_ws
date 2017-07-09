#include <ros/ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Float32.h>
#include <sensor_msgs/LaserScan.h>

std_msgs::Float32 angMsg;
std_msgs::Float32 distMsg;
ros::Publisher obs_ang_pub;
ros::Publisher obs_dist_pub;


void scanValues(const sensor_msgs::LaserScan::ConstPtr& scan)
{
  sensor_msgs::LaserScan laser;
  laser = *scan;
  float m = 100;
  float thresh = 1.20;
  int ang = -1;

    for (unsigned int i=0; i<laser.ranges.size();i++)
    {
      if( laser.ranges[i]<thresh ){
        if( laser.ranges[i]<m ){
          m = laser.ranges[i];
          ang = i;
        }
      }
    }
    if( ang!=-1 ){
      //printf("Obstacle detected at : %f , %d \n", m, ang);
      angMsg.data=ang/2;
      distMsg.data=m;
      obs_ang_pub.publish(angMsg);
      obs_dist_pub.publish(distMsg);
    }else if( ang==-1 ){
      //printf("Obstacle detected at : %f , %d \n", m, ang);
      angMsg.data=0;
      distMsg.data=0;
      obs_ang_pub.publish(angMsg);
      obs_dist_pub.publish(distMsg);
    }
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "obstacle_node");
  ros::NodeHandle n;
  ros::Subscriber hokuyoSubscriber = n.subscribe<sensor_msgs::LaserScan>("/rawscan", 10, &scanValues);
  obs_ang_pub = n.advertise<std_msgs::Float32>("obstacle/angle", 10);
  obs_dist_pub = n.advertise<std_msgs::Float32>("obstacle/distance", 10);
  ros::spin();
  return 0;
}