/*

Node Name   - 	LQRnonconstv
Publishers  - 	steercontrol
Subscribers -	base_pose_ground_truth
				astroid_path
				base_pose_ground_truth"
Authors :    	Shobhit Saheb Dey & Kartik Punjabi
*/

// ld = 5.0 k = 0.2  
// 2.5 0.07 /Cr,mCf

#include "ros/ros.h"
#include "std_msgs/Float64.h"
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Twist.h"
#include "nav_msgs/Path.h"
#include <math.h>
#include <unistd.h>
#include "vector"
#include <beginner_tutorials/Control.h>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Eigenvalues>
#include <complex>
#include <iostream>
#define scale 20000
#define constD 1
#define k 0.5
using namespace ros;
using namespace std;
using namespace Eigen;

double M=1356;
double Iz=2681.95008625;
double Cf=155494.663;
double Cr=155494.663;
double vx=0;
double lf=1.1898;
double lr=0.7932;

MatrixXcd A(4,4);
MatrixXcd B(4,1);
MatrixXcd Ad(4,4);
MatrixXcd Bd(4,1);
MatrixXcd diagA(4,4);
MatrixXcd diagB(4,4);
MatrixXcd eigVal(4,1);
MatrixXcd eigVect(4,4);
MatrixXcd P(4,4);
MatrixXcd R(1,1);
MatrixXcd P0(4,4); // Recati soln
MatrixXcd Q(4,4);
MatrixXcd K(1,4);
VectorXd X(4);  
int i = 0;

float j,e ;

float a, b;
float Xpos[1000], Ypos[1000];

//float d = 5.0
void discretise();
void riccati();
//void steer();
beginner_tutorials::Control v;
std_msgs::Float64 msg;
double wheelBase=1.983;
struct path_point{
	double px,py,d; // path points and d= distance
	double slope;	// slope at that point
	int index;		//	
};
int direction=0;
path_point p;   
double x=0,y=0,ayaw=0,ryaw0=0,ryaw1=0,ryawDiff=0,speed=5,error0=0,error1=0,errorDiff=0,angleOffset=0;
int prevIndex=0;
double yawRate=0;
double errorRate=0;
double sec0=0,sec1=0;
vector<geometry_msgs::Pose> incomePath;
std_msgs::Float64 steer;
double minD;
path_point pointSearch(){
	path_point temp,goal;	
	//<<incomePath.size()<<endl;
	////<<"prevIndex entered in the function: "<<index<<endl;

		int index=0;
		temp.px=incomePath[0].position.x;
		////<<"x coord: "<<temp.px<<endl;
		temp.py=incomePath[0].position.y;
		////<<"y coord: "<<temp.py<<endl;
		temp.d=sqrt(pow(x-temp.px,2)+pow(y-temp.py,2));
		////<<"distance from car: "<<temp.d<<endl;
		minD=temp.d;
		////<<"finding closest point"<<endl;

		for(int i=0;i<incomePath.size();i++){
			
			if(sqrt(pow(x-incomePath[i].position.x,2)+pow(y-incomePath[i].position.y,2))<minD)
			{
				index=i;
				minD=sqrt(pow(x-incomePath[i].position.x,2)+pow(y-incomePath[i].position.y,2));
					////<<"if me chala 1212"<<endl;

			}
		}
		////<<"got closest point"<<endl;
		goal.index=index;
		////<<"Index found: "<<goal.index<<endl;
		goal.d=minD;
		cout<<"cross track error: "<<goal.d<<endl;
		msg.data=goal.d;

		goal.px=incomePath[index].position.x;
		goal.py=incomePath[index].position.y;	
		goal.slope=atan2(incomePath[index+1].position.y-goal.py,incomePath[index+1].position.x-goal.px);
		////<<"calculated slope"<<endl;
		if((x*goal.py-y*goal.px)>0)
			direction=-1;
		else
			direction=1;
		prevIndex=goal.index;
		//<<"Direction : "<<direction<<endl;
	
	
	return goal;
}

void LQR(){
	discretise();
	riccati();

	if(sec1!=sec0){
		steer.data=-K(0,0).real()*error1-K(0,1).real()*(errorDiff/(sec1-sec0))-K(0,2).real()*ryaw1-K(0,3).real()*(ryawDiff/(sec1-sec0));
		steer.data=0.3*steer.data;
	}
	else
		steer.data=0;
	steer.data=direction*(steer.data)/3.14;
	if(steer.data>1)
	{
		steer.data=1;

	}
	else if(steer.data<-1)
		{steer.data=-1;}
	cout<<"Steering angle calculated: "<<steer.data<<endl;

	//<<"steering angle: "<<steer.data<<endl;
}
void discretise()
		{
			//<<"discretising the matrices"<<endl;
			ComplexEigenSolver<MatrixXcd> ces;
			ces.compute(A);
			eigVal=ces.eigenvalues();	
			eigVect=ces.eigenvectors();
			for(int i=0;i<4;i++){
				for(int j=0;j<4;j++){
					if(i==j){
						diagA(i,j)=exp(0.01*eigVal(i,0));
						diagB(i,j)=exp(0.01*eigVal(i,0))-complex<double>(1,0);
					}
					else{
						diagA(i,j)=0;
						diagB(i,j)=0;
					}
				}
			}
			Ad=eigVect.inverse()*diagA*eigVect;
			Bd=eigVect.inverse()*diagB*eigVect*B;
			////<<"discretised A: "<<Ad<<endl;
			////<<"discretised B: "<<Bd<<endl;
		}
void riccati(){
	////<<"solving riccati"<<endl;
	P<<0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0;
	P0<<0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0;
Q<<5000000,0,0,0,0,0,0,0,200,0,0,0,0,0,0,0;
R<<1;

			complex<double> error;
			do 
			{P0=P;
				P=Ad.transpose()*P*Ad-Ad.transpose()*P*Bd*(R+Bd.transpose()*P*Bd).inverse()*Bd.transpose()*P*Ad+Q;
				error=sqrt(((P-P0).transpose()*(P-P0)).trace());

			}while(abs(error)<0.0005);

				K=(R+Bd.transpose()*P*Bd).inverse()*Bd.transpose()*P*Ad;
				////<<"Calculated K: "<<K<<endl;
		}

void posCallBack(const nav_msgs::Odometry::ConstPtr &msg){
	double wheelBase=1.98;
	x=msg->pose.pose.position.x;
	y=msg->pose.pose.position.y;
	double q1= msg->pose.pose.orientation.x;
	double q2= msg->pose.pose.orientation.y;
	double q3= msg->pose.pose.orientation.z;
	double q4= msg->pose.pose.orientation.w;
	double siny=2.0*(q4*q3+q1*q2),cosy=1-2*(q2*q2);	
	ayaw=atan2(siny,cosy);
	////<<"got position and orientation"<<endl;
	ryaw0=ryaw1;
	////<<"prevIndex: "<<prevIndex<<endl;
	x=msg->pose.pose.position.x;
	y=msg->pose.pose.position.y;
	// x=x+0.5*(wheelBase)*cos(ayaw);
	// y=y+0.5*(wheelBase)*sin(ayaw);
	if(incomePath.size()>0)
		p=pointSearch();
	else{
		////<<"path yet not recieved"<<endl;
		return;
	}
	////<<"got the closest point"<<endl;
	ryaw0=atan2(p.slope,1)-ayaw;		
	ryawDiff=ryaw1-ryaw0;	
	
	error0=error1;
	error1=p.d;
	errorDiff=error1-error0;
	sec0=0;
	sec1=0.005;
	////<<"sec0: "<<sec0<<" sec1: "<<sec1<<endl;
	if(sec0==sec1){
		yawRate=0;
		errorRate=0;
	}
	else{
		yawRate=ryawDiff/(sec1-sec0);
		errorRate=errorDiff/(sec1-sec0);
	}
	////<<"ready to run LQR"<<endl;
	LQR();
}

void velCallBack(const nav_msgs::Odometry::ConstPtr msg){
vx=sqrt(pow(msg->twist.twist.linear.x,2)+pow(msg->twist.twist.linear.y,2));
//<<"Velocity calculated: "<<vx<<endl;
A << 0,1,0,0,
	0,-(Cr+Cf)/(M*vx),(Cf+Cr)/M,(lr*Cr-lf*Cf)/(M*vx),
	0,0,0,1,
	0,(lr*Cr-lf*Cf)/(Iz*vx),-(lr*Cr-lf*Cf)/Iz,-(lf*lf*Cf+lr*lr*Cr)/(Iz*vx);

////<<"A initialized"<<endl;
//<<A<<endl;
B << 0,Cf/M,0,lf*Cf/Iz;
////<<"B initialized: "<<endl;
////<<B<<endl;
}

void call(const nav_msgs::Path::ConstPtr msg){
	////<<"path liya "<<endl;
	incomePath.clear();
	// incomePath=*msg;
	int i=0;
	////<<"Incoming PATH"<<endl;
	for(i=0;i<msg->poses.size();i++){
		incomePath.push_back(msg->poses[i].pose);
		////<<msg->poses[i].pose.position.x<<endl;
	}

	// for(int i=0;i<incomePath.size();i++){
	// 	//<<" "<<incomePath[i]<<endl;
	// }
	
}

// void callback( nav_msgs::Odometry car_cod ){
	
// 	a = car_cod.pose.pose.position.x;
// 	b = car_cod.pose.pose.position.y;


//     xx = car_cod.pose.pose.orientation.x;
//     yy = car_cod.pose.pose.orientation.y;
// 	zz = car_cod.pose.pose.orientation.z;
// 	ww = car_cod.pose.pose.orientation.w;

// 	distance();

// 	steer();

// 	ROS_INFO("steer = %lf , d = %f, m = %lf, n = %lf, e = %f ", v.steer, d, m, n, e);

	
// }


int main(int argc, char **argv)
{
	ros::init(argc ,argv, "pursuit");

	ros::NodeHandle n1;

	ros::Subscriber s1 = n1.subscribe<nav_msgs::Odometry>("base_pose_ground_truth",1,&posCallBack);

	ros::Subscriber s2 = n1.subscribe<nav_msgs::Path>("astroid_path",1,&call);

	ros::Publisher p = n1.advertise<std_msgs::Float64>("steercontrol",10);
	
	ros::Subscriber sub = n1.subscribe<nav_msgs::Odometry>("base_pose_ground_truth",1,velCallBack);

	ros::Publisher bag_pub = n1.advertise<std_msgs::Float64>("Cross_track_bag",1);

	ros::Rate loop_rate(10);

	
	while(ros::ok())
	{		
		p.publish(steer);
		bag_pub.publish(msg);
		ros::spinOnce();
		
		loop_rate.sleep();
	}

	return 0;
}
