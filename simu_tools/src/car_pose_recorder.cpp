#include <ros/ros.h>
#include "std_msgs/String.h"
#include <tf/transform_listener.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <sstream>
#include <iostream>
#include <vector>
#include <string>
#include <fstream>
#include <signal.h>
#include <cstdlib>
#include <stdlib.h>
#include <stdio.h>
#include <cmath>
//pose recorder，录制车辆位姿真值，保存成tum格式txt文件。
using namespace std;

struct Test_Result{
  geometry_msgs::PoseStamped pose;
  //geometry_msgs::Twist twist;
};

vector <Test_Result> result_output;
Test_Result result_one;
const char *output_filename; //pose and twist stamped;

int Print_output()
{
    double eps = 0.000000002;
    //print out pose and twist stamped:
    FILE * fp1;
    if((fp1 = fopen(output_filename,"wb"))==NULL){
      printf("cant open the output file");
      ROS_INFO("cant open the output file");
      exit(0);
    }
    if (result_output.empty())
    {
       fprintf(fp1," \n");
    }
    else
    {
      int i;
      for(i=0;i<result_output.size();i++){
        double stamp = (double)result_output[i].pose.header.stamp.sec+(double)result_output[i].pose.header.stamp.nsec/1000000000;
        double result_x = result_output[i].pose.pose.position.x;
        double result_y = result_output[i].pose.pose.position.y;
        if (abs(result_x)+abs(result_y)>eps)
        {
          fprintf(fp1,"%.9f %.9f %.9f %.9f %.9f %.9f %.9f %.9f\n",stamp,result_x,result_y,result_output[i].pose.pose.position.z,result_output[i].pose.pose.orientation.x,result_output[i].pose.pose.orientation.y,result_output[i].pose.pose.orientation.z,result_output[i].pose.pose.orientation.w);
        }      
      }
    }
    fclose(fp1);
    return 0;
}

void DoShutdown(int sig)
{
    //这里主要进行退出前的数据保存、内存清理、告知其他节点等工作
    ROS_INFO("shutting down!");
    Print_output();
    ros::shutdown();
    exit(sig);
}

void recorderCallback(const nav_msgs::Odometry& msg)
{
    result_one.pose.header.frame_id = "vins_world";
    result_one.pose.header.stamp = msg.header.stamp;
    result_one.pose.pose = msg.pose.pose;
    result_output.push_back(result_one);

}

int main(int argc, char** argv){
    ros::init(argc, argv, "car_pose_recorder");
    if(argc !=3 ){
            cerr << endl << "Usage: output_filename rostopic_name" << endl;
    }
    const char *rostopic_name;
    output_filename = argv[1];
    rostopic_name = argv[2];
    
    ROS_INFO("Pose recorder starts.");
    
    ros::NodeHandle n1; 
    ros::Subscriber sub = n1.subscribe(rostopic_name,1,&recorderCallback);

    ros::Rate rate(500.0);

    signal(SIGINT, DoShutdown);
     //The next two lines are important because this node should sync with the ros system time:
    ros::spinOnce();
    ros::Duration(0.1).sleep(); 
    while (ros::ok()){
        ros::spinOnce();
        rate.sleep();
    }
    Print_output();
    return 0;
}
