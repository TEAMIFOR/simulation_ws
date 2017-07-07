/* let me tell you a story,
ben is a drone, ben is a crazy child, whenever he takesoff he starts 
flying off in random directions unless we control him using offboard
mode, now the problem is for ben to remain stable in offboard mode u
need to keep feeding it setpoints constantly with a max gap of 0.5s, 
ie 50hz. This explains the reason why we are keeping the setpointpos 
publishers under nested while loops. Now we need a have a smooth 
transition from takeoff_n_survey to ellipse_not_found_n_survey becuz
as mentioned earlier ben will go out of control if you stop sending 
setpoints even for a short period of time. 

Ben's daily routine starts by taking off and surveying in a circle, 
looking for ground robots as soon as he see the GB he exits from the
takeoff_n_survey while loop, and transitions to another while loop 
which has many nested loops whith different conditions. So the sub
scriber which receives data from ellipse detection node tells ben if
ellipse has been found or not. Currently when the ellipse has been 
found it stays inside the ellipseFound loop till the img processing 
node tells him no more ellipses. Then ben still staying inside the 
master while loop transitions to another while loop where there is 
ellipseNotFound flag checking.

The pixel coordinates of the midpoints of ellipses from ell_det.py
is divided into four quadrants. In this system ben is in the middle
and based on which quadrant the ellipse mid is in it increments/dec
reents its x and y coordinates by a small amount. this data is publ
ished. 

A hovering phase has been established which works by streaming the 
last known position constantly, this is important when no ellipses 
are found, [TODO: tell ben to survey the remaining portion] in this 
loop. Hovering phase is essential as switching to auto.loiter caused
problems to ben and i needed a way to make it stay at the same 
position for some time to do the selection and still send setpoints
(refer ben's problems para1).
*/

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include "math.h"
#include <chrono>
#include <thread>
#include <geometry_msgs/Vector3.h>

#include <geometry_msgs/Vector3Stamped.h>

double r;
double theta;
double count=0.0;
double wn;
int ellFound=0;

ros::Publisher local_pos_pub;
ros::ServiceClient arming_client;
ros::ServiceClient set_mode_client;
ros::Subscriber mid_sub;
geometry_msgs::PoseStamped pose;

mavros_msgs::State current_state;
geometry_msgs::PoseStamped localPose_;
geometry_msgs::PoseStamped hoverPose_; //generating a new hoverPhase is da reeeel deaal


void state_cb(const mavros_msgs::State::ConstPtr& msg)
{
    current_state = *msg;
}
void cbLocalPosition(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    localPose_ = *msg;
}
/* This is the master brain which takes input from image processing node
and tells ben if an ellipse has been found or not. Further it also sends 
coordinates for ben to move in order to acheive object following 
The 800x800 pixelcordinate frame is divided into 4 quadrants of 400x400
pixels and each quadrant increments/ decrements ben's coordinates acc.
TODO: The quadrant implementation needs to be revoked as ben's cameras 
will be tilted. Good sugesstions: Gradient map 
*/
void genTargets(const geometry_msgs::Vector3 &vector3)
{
    geometry_msgs::Vector3 vec;
    vec.x = vector3.x;
    vec.y = vector3.y;
    vec.z = vector3.z;
    if( vec.x==0 && vec.y==0 && vec.z==0 ) // distance_calulation gotta do its duty
    {
      ellFound = 0;

    }else if( vec.x!=0 || vec.y!=0 || vec.z!=0 )
    {
      ellFound =1;
        if( vec.x>-100 && vec.y>-100 && vec.x<=400 && vec.y<=400 ){
          pose.pose.position.x = localPose_.pose.position.x  + 0.09f;
          pose.pose.position.y = localPose_.pose.position.y  + 0.09f;
          pose.pose.position.z = 2.00f ;

        }else if( vec.x>400 && vec.y>-100 && vec.x<900 && vec.y<=400 ){
          pose.pose.position.x = localPose_.pose.position.x  + 0.09f;
          pose.pose.position.y = localPose_.pose.position.y  - 0.09f;
          pose.pose.position.z = 2.00f ;

        }else if( vec.x>-100 && vec.y>400 && vec.x<400 && vec.y<900){
          pose.pose.position.x = localPose_.pose.position.x  - 0.09f;
          pose.pose.position.y = localPose_.pose.position.y  + 0.09f;
          pose.pose.position.z = 2.00f ;

        }else if( vec.x>400 && vec.y>400 && vec.x<900 && vec.y<900 ){
          pose.pose.position.x = localPose_.pose.position.x  - 0.09f;
          pose.pose.position.y = localPose_.pose.position.y  - 0.09f;
          pose.pose.position.z = 2.00f ;

        }else{ // this shouldnt occur but added just in case for dev purposes
          printf("impossible event occured: %f, %f, %f\n", vec.x, vec.y, vec.z );
        }
    }
}
bool isTargetPos()
{
     if ( abs(pose.pose.position.x - localPose_.pose.position.x) < 0.0005 &&
          abs(pose.pose.position.y - localPose_.pose.position.y) < 0.0005 &&
          abs(pose.pose.position.z - localPose_.pose.position.z) < 0.0005)
     {
        return true;
     }
    return false;   
}

int movetopos()
{
    mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = "OFFBOARD";
    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;
    ros::Time last_request = ros::Time::now();

    while(ros::ok()) 
    {
      if( current_state.mode != "OFFBOARD" && (ros::Time::now() - last_request > ros::Duration(5.0)))
      {
        if( set_mode_client.call(offb_set_mode) && offb_set_mode.response.success)
        {
          ROS_INFO("Offboard enabled");
        }
        last_request = ros::Time::now();
      } 
      else 
      {
        if( !current_state.armed && (ros::Time::now() - last_request > ros::Duration(5.0)))
        {
          if( arming_client.call(arm_cmd) && arm_cmd.response.success)
          {
            ROS_INFO("Vehicle armed");
          }
          last_request = ros::Time::now();
        }
      }
      if (isTargetPos())
      {
        return 0;
      }
      else
      {
        local_pos_pub.publish(pose);
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
      }
    }
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "offboard1");
    ros::NodeHandle nh;

    std::thread t([]() {
         ros::AsyncSpinner spinner(4); // Use 4 threads
         spinner.start();
         ros::waitForShutdown();
    });

    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>
            ("mavros/state", 10, state_cb);
    local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>
            ("mavros/setpoint_position/local", 10);
    arming_client = nh.serviceClient<mavros_msgs::CommandBool>
            ("mavros/cmd/arming");
    set_mode_client = nh.serviceClient<mavros_msgs::SetMode>
            ("mavros/set_mode");
    ros::Subscriber location_sub_ = nh.subscribe<geometry_msgs::PoseStamped>
            ("mavros/local_position/pose", 10, cbLocalPosition);
    mid_sub = nh.subscribe("/mid", 10, genTargets);

    //the setpoint publishing rate MUST be faster than 2Hz
    ros::Rate rate(50.0);
  

    nh.param("offboard1/wn", wn, 1.0);
    nh.param("offboard1/r", r, 1.0);
    // wait for FCU connection
    while(ros::ok() && current_state.connected){
        ros::spinOnce();
        rate.sleep();
    }
    // this is formal mandatory steps to safely go into offb mode
    pose.pose.position.x = 0;
    pose.pose.position.y = 0;
    pose.pose.position.z = 2;
    for(int i = 10; ros::ok() && i > 0; --i){
        local_pos_pub.publish(pose);
        ros::spinOnce();
        rate.sleep();
    }

    /*ben takes off and goes into intial surveillance mode, stays
    here till a GB has been found and then transitions smoothly to
    next while loop */    
    while(ros::ok() && !ellFound ) //ellipse found by first instance, 
    {
        theta = 0.2*count*0.001;
        pose.pose.position.x = 3*sin(theta);
        pose.pose.position.y = 3*cos(theta);
        pose.pose.position.z = 2.00f;
        hoverPose_.pose.position.x = localPose_.pose.position.x; // these help in hoverPhase
        hoverPose_.pose.position.y = localPose_.pose.position.y; // they are like lastknowloc
        hoverPose_.pose.position.z = localPose_.pose.position.z; 
        count++;
        if (!isTargetPos())
        {
          movetopos();
        }
    }
    /* This while loop should never be exited unless mission has ended as
    ben will start behaving abnormally. Further it should contain nexted 
    while loops for different scenarios like GBs_found_selecting, 
    GB_selected_following, GB_landing, GBs_not_found_surverying .. etc  */
    while(ros::ok())
    {
      while(ros::ok() && !ellFound ) //survey to find next groundbot
      {
          mavros_msgs::SetMode offb_set_mode;
          offb_set_mode.request.custom_mode = "OFFBOARD";
          mavros_msgs::CommandBool arm_cmd;
          arm_cmd.request.value = true;
          ros::Time last_request = ros::Time::now();
          for(int i=0;i<54;i++)
          {
            if( current_state.mode != "OFFBOARD" && (ros::Time::now() - last_request > ros::Duration(5.0)))
            {
              if( set_mode_client.call(offb_set_mode) && offb_set_mode.response.success)
              {
                ROS_INFO("Offboard enabled");
              }
              last_request = ros::Time::now();
            } 
            else 
            {
              if( !current_state.armed && (ros::Time::now() - last_request > ros::Duration(5.0)))
              {
                if( arming_client.call(arm_cmd) && arm_cmd.response.success)
                {
                  ROS_INFO("Vehicle armed");
                }
                last_request = ros::Time::now();
              }
            }
              local_pos_pub.publish(hoverPose_);
              std::this_thread::sleep_for(std::chrono::milliseconds(100));
          }
      }
     /* This is where ben follows the GB that has been bound
     TODO: follow until flag land_in_front_of_gb has been set */
      while(ros::ok() && ellFound ) //this is da reeell place 
      {
          mavros_msgs::SetMode offb_set_mode;
          offb_set_mode.request.custom_mode = "OFFBOARD";
          mavros_msgs::CommandBool arm_cmd;
          arm_cmd.request.value = true;
          ros::Time last_request = ros::Time::now();

          for(int i=0;i<54;i++)
          {
            if( current_state.mode != "OFFBOARD" && (ros::Time::now() - last_request > ros::Duration(5.0)))
            {
              if( set_mode_client.call(offb_set_mode) && offb_set_mode.response.success)
              {
                ROS_INFO("Offboard enabled");
              }
              last_request = ros::Time::now();
            } 
            else 
            {
              if( !current_state.armed && (ros::Time::now() - last_request > ros::Duration(5.0)))
              {
                if( arming_client.call(arm_cmd) && arm_cmd.response.success)
                {
                  ROS_INFO("Vehicle armed");
                }
                last_request = ros::Time::now();
              }
            }
              hoverPose_.pose.position.x = localPose_.pose.position.x;
              hoverPose_.pose.position.y = localPose_.pose.position.y;
              hoverPose_.pose.position.z = localPose_.pose.position.z;
              //temporary fixtures
              printf("fromx: %f tox: %f,   fromy: %f toy: %f z: %f\n", localPose_.pose.position.x, pose.pose.position.x, localPose_.pose.position.y, pose.pose.position.y, pose.pose.position.z);
              local_pos_pub.publish(pose); 
              std::this_thread::sleep_for(std::chrono::milliseconds(100));
          }
      }
    }
    return 0;
}
