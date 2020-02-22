/*
Authors: Shobhit Saheb Dey, Kartik Punjabi
iterative-Linear Quadratic Regulator
Developed on kinematic model of car
Tested on gazebo(car-demo model) on sinusoidal path at 5m/sec
*/
#include "ros/ros.h"
#include "std_msgs/Float64.h"
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Twist.h"
#include "nav_msgs/Path.h"
#include <unistd.h>
#include "vector"
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Eigenvalues>
#include <complex>
#include <iostream>
#include <stdlib.h>
#include <bits/stdc++.h>
#include <fstream>
#include<iostream>
using namespace std;
using namespace Eigen;
double cot(double);
double sec(double);
double cosec(double);
double L=1.983;
double sampling=0.01;
int NSamples=15;
double v=20;
MatrixXd C(4,4);
MatrixXd c(4,1);

MatrixXd Qt(4,4);
MatrixXd q(4,1);
MatrixXd Vt(3,3);
MatrixXd vt(3,1);
MatrixXd F(3,4);
MatrixXd obs(4,4);
MatrixXd X_goal(3,1);
MatrixXd *K_array = new MatrixXd[NSamples];
MatrixXd *k_array = new MatrixXd[NSamples];
MatrixXd *u_array = new MatrixXd[NSamples];
MatrixXd *u_array_prev = new MatrixXd[NSamples];
MatrixXd *del_u_array = new MatrixXd[NSamples];
MatrixXd *X_array = new MatrixXd[NSamples];
MatrixXd *Action_array = new MatrixXd[NSamples];
MatrixXd *del_X_array = new MatrixXd[NSamples];
MatrixXd *F_array = new MatrixXd[NSamples];
MatrixXd kinematicFunc(MatrixXd X, MatrixXd u);
MatrixXd Action_final(4,1);
double x_current= 155,y_current=282,yaw_current=0,x_goal=165,y_goal=290,yaw_goal=1.04;
struct path_point{
	double px,py; // path points and d= distance
	double slope;	// slope at that point
	int index;		//	
};
int direction=0;
path_point p;   

int prevIndex=0;
double yawRate=0;
double errorRate=0;
double sec0=0,sec1=0;
double minD;
int minindex=0;
int flag=0;
vector<geometry_msgs::Pose> incomePath;
path_point pointSearch(){
	path_point temp,goal;

		int index=0;
		temp.px=incomePath[0].position.x;
		temp.py=incomePath[0].position.y;
		temp.d=sqrt(pow(x_current-temp.px,2)+pow(y_current-temp.py,2));
		minD=temp.d;

		for(int i=0;i<incomePath.size();i++){
			
			if(sqrt(pow(x_current-incomePath[i].position.x,2)+pow(y_current-incomePath[i].position.y,2))<minD)
			{
				index=i;
				minD=sqrt(pow(x_current-incomePath[i].position.x,2)+pow(y_current-incomePath[i].position.y,2));
			}
		}
		cout<<"found the point at min distance"<<endl;
		if(index<incomePath.size()-19){
			cout<<"no problem in setpoint"<<endl;
			goal.index=index+20;
		}
		else
			goal.index=index;
		goal.px=incomePath[goal.index].position.x;
		goal.py=incomePath[goal.index].position.y;	
		goal.slope=atan2(incomePath[goal.index+1].position.y-goal.py,incomePath[goal.index+1].position.x-goal.px);
		cout<<"done1"<<endl;	
	return goal;
}


void call(const nav_msgs::Path::ConstPtr msg){	
	cout<<"call for path"<<endl;
	incomePath.clear();	
	int i=0;	
	for(i=0;i<msg->poses.size();i++){
		incomePath.push_back(msg->poses[i].pose);		
	}	
	cout<<"done path"<<endl;
}

MatrixXd jacobian(MatrixXd X, MatrixXd u){
	MatrixXd F(3,4);
	MatrixXd col1(3,1); MatrixXd col2(3,1); MatrixXd col3(3,1); MatrixXd col4(3,1);
	MatrixXd delx(3,1); MatrixXd dely(3,1); MatrixXd deltheta(3,1); MatrixXd delu(1,1);
	delx<<X(0,0)+0.001,X(1,0),X(2,0);
	dely<<X(0,0),X(1,0)+0.001,X(2,0);
	deltheta<<X(0,0),X(1,0),X(2,0)+0.001;
	delu<<u(0,0)+0.001;
	col1=(kinematicFunc(delx,u)-kinematicFunc(X,u))*1000;
	col2=(kinematicFunc(dely,u)-kinematicFunc(X,u))*1000;
	col3=(kinematicFunc(deltheta,u)-kinematicFunc(X,u))*1000;
	col4=(kinematicFunc(X,delu)-kinematicFunc(X,u))*1000;
	F<<col1(0,0),col2(0,0),col3(0,0),col4(0,0),
	   col1(1,0),col2(1,0),col3(1,0),col4(1,0),
	   col1(2,0),col2(2,0),col3(2,0),col4(2,0);
	return F;
}
void update(){
	for(int i=0;i<NSamples;i++){
		del_u_array[i]<<0;
		del_X_array[i]<<0,0,0;
		F_array[i]=jacobian(X_array[i],u_array[i]);
		Action_array[i]<<X_array[i](0,0),X_array[i](1,0),X_array[i](2,0),u_array[i](0,0);
	}
	Vt=MatrixXd::Zero(3,3);
	vt=MatrixXd::Zero(3,1);
	Qt=MatrixXd::Zero(4,4);
}

double mod_angle(MatrixXd delta){
	if(delta(0,0)<0)
		return -(-delta(0,0)-((int)(-delta(0,0))/6.28)*6.28);
	else
		return delta(0,0)-((int)delta(0,0)/6.28)*6.28;
}

double cot(double data)
{ 
	if (data==0) return 0;
return 1/tan(data);
}	

double sec(double data)
{if (data==0) return 0;
return 1/cos(data);
}

double cosec(double data)
{if (data==0) return 0;
return 1/sin(data);
}

MatrixXd kinematicFunc(MatrixXd X, MatrixXd u){
	MatrixXd X_temp(3,1);
	u<<mod_angle(u);
	double delta=u(0,0), x=X(0,0), y=X(1,0), theta=X(2,0);
	double phi = (v*tan(delta)*sampling)/L;
	X_temp<<x+2*L*cot(delta)*sin(phi/2)*cos(theta+phi/2),y+2*L*cot(delta)*sin(phi/2)*sin(theta+phi/2),theta+phi;
	return X_temp;
}

void backwardPass(){
	C<<
	  600,0,0,0,
	  0,600,0,0,
	  0,0,3000,0,
	  0,0,0,600;
	
	double ellipseVal;

	for(int i=NSamples-1;i>-1;i--){
		

		Qt = C + F_array[i].transpose()*Vt*F_array[i];
		q = C*(Action_array[i]-Action_final)+ F_array[i].transpose()*vt;		
		C(2,2)*=0.95;
		C(1,1)*=1; C(0,0)*=1;
		
		K_array[i]=-0.5*(Qt.block<1,1>(3,3)).inverse()*(Qt.block<3,1>(0,3).transpose()+Qt.block<1,3>(3,0));
		k_array[i]=-(Qt.block<1,1>(3,3)).inverse()*q.block<1,1>(3,0);
		Vt = Qt.block<3,3>(0,0)+Qt.block<3,1>(0,3)*K_array[i]+K_array[i].transpose()*(Qt.block<1,3>(3,0)+Qt.block<1,1>(3,3)*K_array[i]);
		vt = (Qt.block<3,1>(0,3)+Qt.block<1,3>(3,0).transpose())*k_array[i]+K_array[i].transpose()*(Qt.block<1,1>(3,3)+Qt.block<1,1>(3,3).transpose())*k_array[i]+q.block<3,1>(0,0)+K_array[i].transpose()*q.block<1,1>(3,0);
	}
}

void forwardPass(){
	MatrixXd temp(3,1);
	for(int i=0;i<NSamples;i++){
		u_array[i]=u_array[i]+ K_array[i]*del_X_array[i]+k_array[i];
		if(i<NSamples-1){
			
			temp=X_array[i+1];
			X_array[i+1]=kinematicFunc(X_array[i],u_array[i]);
			del_X_array[i+1]+=X_array[i+1]-temp;
			Action_array[i]<<X_array[i](0,0),X_array[i](1,0),X_array[i](2,0),u_array[i];
		}
	}
	
}

void iLQR(){
	cout<<"iLQR called"<<endl;
	Action_final<<x_goal,y_goal,yaw_goal,0;
	for(int i=0;i<NSamples;i++){
		u_array[i]<<0.1;
		X_array[i]<<x_current,y_current,yaw_current;		
		del_u_array[i]<<0;
		del_X_array[i]<<0,0,0;
		Action_array[i]<<X_array[i](0,0),X_array[i](1,0),X_array[i](2,0),u_array[i](0,0);
		F_array[i]=jacobian(X_array[i],u_array[i]);
	}
	cout<<"Current state: "<<endl<<X_array[0]<<endl;
	cout<<"Final Action"<<Action_final<<endl;
	Vt=MatrixXd::Zero(3,3);
	vt=MatrixXd::Zero(3,1);
	double sum = 0;
	double temp;
	MatrixXd diff;
	do{
		sum=0;
		for(int i=0;i<NSamples;i++)
			u_array_prev[i]=u_array[i];
		backwardPass();
		cout<<"backwardPass done"<<endl;
		forwardPass();  
		cout<<"forwardPass done"<<endl;  
		for(int i=0;i<NSamples;i++){
			temp = (mod_angle(u_array[i])- mod_angle(u_array_prev[i]))*(mod_angle(u_array[i])-mod_angle(u_array_prev[i]));
			sum = sum + temp;
		}

		if(sum<24)
			break;
		update();
	}while(1);
	
}

void posCallBack(const nav_msgs::Odometry::ConstPtr &msg){
	cout<<"posCallBack called"<<endl;
	double wheelBase=1.98;
	x_current=msg->pose.pose.position.x;
	y_current=msg->pose.pose.position.y;
	double q1= msg->pose.pose.orientation.x;
	double q2= msg->pose.pose.orientation.y;
	double q3= msg->pose.pose.orientation.z;
	double q4= msg->pose.pose.orientation.w;
	double siny=2.0*(q4*q3+q1*q2),cosy=1-2*(q2*q2);	
	yaw_current=atan2(siny,cosy);
	if(incomePath.size()>0){
		p=pointSearch();
		flag=1;
		cout<<"path received"<<endl;
	}
	else{
		return;
	}
	x_goal=p.px;
	y_goal=p.py;
	yaw_goal=p.slope;
	iLQR();
}

int main(int argc, char **argv)
{
	for(int i=0;i<NSamples;i++)
	{
	K_array[i]=MatrixXd::Zero(3,1);
	k_array[i]=MatrixXd::Zero(3,1);
    u_array[i]=MatrixXd::Zero(1,1);
    u_array_prev[i]=MatrixXd::Zero(1,1);
    del_u_array[i]=MatrixXd::Zero(1,1);
    X_array[i]=MatrixXd::Zero(3,1);
    Action_array[i]=MatrixXd::Zero(4,1);
    del_X_array[i]=MatrixXd::Zero(3,1);
    F_array[i]=MatrixXd::Zero(4,4);	    
	}
	X_goal<<x_goal,y_goal,yaw_goal;
	std_msgs::Float64 steer;
	ros::init(argc ,argv, "iLQR");
	ros::NodeHandle n1;
	ros::Publisher p = n1.advertise<std_msgs::Float64>("cmd_steer",10);
	ros::Subscriber s1 = n1.subscribe<nav_msgs::Odometry>("base_pose_ground_truth",1,&posCallBack);
	ros::Subscriber s2 = n1.subscribe<nav_msgs::Path>("astroid_path",1,&call);
	ros::Rate actuation_rate(100);
	int i=0;
	while(ros::ok())
	{	
		ros::spinOnce();
		if(flag==1){
		cout<<"actuation array computed: "<<endl;
		for(int j=0;j<NSamples;j++)
			cout<<u_array[j](0,0)<<" ";	
		
		do{
			steer.data=mod_angle(u_array[i++])/3.14;
			p.publish(steer);
			actuation_rate.sleep();
		}while(i<8);
	}
}
	return 0;
}
