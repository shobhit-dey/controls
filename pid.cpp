//Normal PID with no backward control.

#include "ros/ros.h"
#include "std_msgs/Float64.h"
#include "std_msgs/Int8.h"
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/Twist.h"
//#include <stl/stl_vector.h>
#include "beginner_tutorials/Control.h" //These files are seen in devel->include->package_name->All the header files 

float maxV = 30.0;  //max_v divides everything with this velocity
#define kp 8
#define ki 0.005
#define kd 0.1
using namespace std;
using namespace ros;
float current_vel,integral = 0.0,derivative = 0.0,error,prev_ref=0,prev_err,vout,sendAngle; //Get this from odom. Figure it out 
float direction = 0;
void reset()  //If change in reference, then all previous must be removed
{
  integral = 0.0;
  derivative = 0.0;
  error = 0.0;
  //prev_err = 0.0;

}
class SubcriberAndPublisher
{ public:   SubcriberAndPublisher()
        {
          pub = n.advertise<beginner_tutorials::Control>("prius",1);
          vel = n.subscribe("cmd_vel",1,&SubcriberAndPublisher::velCallback,this);
          //dir = n.subscribe("fwd_bkwd",1,&SubcriberAndPublisher::dirCallback,this);
          steer = n.subscribe("steercontrol",1,&SubcriberAndPublisher::steerCallback,this);
          odom = n.subscribe("base_pose_ground_truth",1,&SubcriberAndPublisher::posCallback,this);
        }
  

  void velCallback(const std_msgs::Float64::ConstPtr &ref_vel)
  {
    if(abs(ref_vel->data - prev_ref)>0.1 )
    {
      reset();
      prev_ref = ref_vel->data;
    }
    cout<<"ref_vel: "<<ref_vel->data<<" current_vel "<<current_vel<<" vout: "<<vout<<" error: "<<error<<endl;   
    error = ref_vel->data - current_vel;
    integral = integral +error;
    derivative = error - prev_err;
    vout = kp*error + ki*integral + kd*derivative;
    cout<<"integral: "<<integral<<"derivative: "<<derivative<<endl;
    vout=vout/maxV;
    prev_ref = ref_vel->data;
    beginner_tutorials::Control msg;
    if (vout>0 && vout<=1)
    {
      msg.throttle = vout;
      msg.brake = 0;
      msg.shift_gears = 2;
      
    }
    else if (vout<0 && vout>=-1)
    {
      msg.throttle = 0;
      msg.brake = -vout;
      msg.shift_gears = 2;  
    } 
    else if(vout>1)
    {
      msg.throttle = 1;
      msg.brake = 0;
      msg.shift_gears = 2;  //Gear ko mat chedna PID mein change hota rahega
    }
    else if(vout<-1)
    {
      msg.throttle = 0;
      msg.brake = 1;
      msg.shift_gears = 2;
    }
    //msg.shift_gears = 1;    
    //New additions
    // if(direction == -1)
    // {
    //  msg.shift_gears = 3;
    // }
    // else
    // {
    //  msg.shift_gears = 2;
    // }
    // if(fabs(ref_vel->data - 0)<0.1)
    // {
    //  msg.shift_gears = 1;  //Neutral : is at 1
    //  msg.throttle = 0;
    //  msg.brake = 1;
    // }
    msg.steer = sendAngle;
    pub.publish(msg);
  }

  // void dirCallback(const std_msgs::Int8::ConstPtr &dire)
  // {
  //  direction = dire->data;
  // }
  void posCallback(const nav_msgs::Odometry::ConstPtr &msg)
  {
    current_vel = sqrt(pow(msg->twist.twist.linear.x,2)+pow(msg->twist.twist.linear.y,2));
    //cout<<msg->twist.twist.linear.x<<" "<<msg->twist.twist.linear.y<<" "<<msg->twist.twist.linear.z<<endl;
  }
  void steerCallback(const geometry_msgs::Twist::ConstPtr &angle)
  {
    sendAngle = angle->angular.z;
  }


private:
  NodeHandle n;
  Publisher pub;
  Subscriber vel,steer,odom,dir;

};








int main(int argc, char **argv)
{
  ros::init(argc,argv,"pid");
  // ros::NodeHandle n;

  // ros::Subscriber vel = n.subscribe("cmd_vel",1,velCallback);
  // ros::Subscriber steer = n.subscribe("cmd_steer",1,steerCallback);  
  // ros::Subscriber odom = n.subscribe("odom",1,posCallback);
  SubcriberAndPublisher grandobject;
  ros::spin();
  return 0;
}

 //ros::NodeHandle n;
 //ros::Publisher pub = n.advertise<control::Control>("prius",1);



// void velCallback(const std_msgs::Float64::ConstPtr &ref_vel)
// {
//  if(abs(ref_vel->data - prev_ref)<0.1)
//  {
//    reset();
//  }
//    error = ref_vel->data - current_vel;
//    integral = integral +error;
//    derivative = error - prev_err;
//    vout = kp*error + ki*integral + kd*derivative;
//    vout=vout/maxV;
//    prev_ref = ref_vel->data;
//    control::Control msg;
//    if (vout>0 && vout<=1)
//    {
//      msg.throttle = vout;
//      msg.brake = 0;
      
//    }
//    else if (vout<0 && vout>=-1)
//    {
//      msg.throttle = 0;
//      msg.brake = vout; 
//    } 
//    else if(vout>1)
//    {
//      msg.throttle = 1;
//      msg.brake = 0;
//    }
//    else if(vout<-1)
//    {
//      msg.throttle = 0;
//      msg.brake = -1;
//    }
        
//    msg.steer = sendAngle;
//    pub.publish(msg);
// }


