#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/PointStamped.h>
#include <visualization_msgs/Marker.h>
#include <eigen3/Eigen/Dense>
#include <stdio.h>
#include <math.h>
using namespace Eigen; 

double time_difference(ros::Time now, ros::Time last_time);
void  path_vis (MatrixXf current_pos, MatrixXf last_pos);
int strange=0;
ros::Publisher path_vis_publisher;
sensor_msgs::Imu::ConstPtr initial_measure ;
ros::Time current_timestamp;
ros::Time last_timestamp;

MatrixXf last_position(3,1);
MatrixXf position(3,1);
MatrixXf velocity(3,1);
Matrix3f angular_ac=Matrix3f::Identity(); ;
Matrix3f I=Matrix3f::Identity();
MatrixXf initial_linear_ac(3,1); 
MatrixXf linear_ac(3,1);  
MatrixXf rectified_linear_ac(3,1);  
visualization_msgs::Marker line_list;
void  imu_cb (const sensor_msgs::Imu::ConstPtr& input)
{
	double time_diff ;
  double theta;
  Matrix3f B;

  
  current_timestamp = input->header.stamp;
  time_diff = time_difference(current_timestamp,last_timestamp );
  
  //printf("Time:%lf,Augular Velocity:%lf,%lf,%lf Linear acceleration: %lf %lf %lf",time_diff,input->angular_velocity.x,input->angular_velocity.y,input->angular_velocity.z,input->linear_acceleration.x,input->linear_acceleration.y,input->linear_acceleration.z);
  if(time_diff>0.01 || time_diff<0.002){
    time_diff=0.005;
    strange=1;
    printf("Strange");
  }
  //Renew orientation
  B(0,0)=0; B(0,1) = -1.0*time_diff * input->angular_velocity.z; B(0,2) = 1.0*time_diff * input->angular_velocity.y;
  B(1,0)=1.0*time_diff * input->angular_velocity.z; B(1,1) = 0; B(1,2) = -1.0*time_diff * input->angular_velocity.x;
  B(2,0)=-1.0*time_diff * input->angular_velocity.y; B(2,1) = 1.0*time_diff * input->angular_velocity.x; B(2,2) = 0;
  //printf("Finish B\n");
  theta = sqrt( (input->angular_velocity.x)*(input->angular_velocity.x) + (input->angular_velocity.y)*(input->angular_velocity.y) + (input->angular_velocity.z)*(input->angular_velocity.z))*time_diff;
  //printf("sine(theta): %lf,cos(theta):%lf",sin(theta),cos(theta));
  angular_ac = angular_ac * (I + sin(theta)/theta * B + (1-cos(theta))/(theta*theta) * B *B);
  linear_ac(0,0)= input->linear_acceleration.x;linear_ac(1,0)= input->linear_acceleration.y;linear_ac(2,0)= input->linear_acceleration.z;
  linear_ac =  angular_ac * linear_ac;
  //printf("Linear ac : %lf %lf %lf\n",linear_ac(0,0),linear_ac(1,0),linear_ac(2,0));

  //printf("Finish linear_ac\n");
  rectified_linear_ac = linear_ac - initial_linear_ac;
  velocity = velocity + time_diff * rectified_linear_ac ;
  position = position + time_diff * velocity;
  printf("Position:%lf %lf %lf\n",position(0,0),position(1,0),position(2,0));
  //printf("Finish position\n");
  path_vis(position,last_position);

}
void  path_vis (MatrixXf current_pos, MatrixXf last_pos){
    
    line_list.header.frame_id = "/map";
    line_list.header.stamp = ros::Time::now();
    line_list.action = visualization_msgs::Marker::ADD;
    line_list.pose.orientation.w = 1.0;
    line_list.id = 2;
    line_list.type = visualization_msgs::Marker::LINE_STRIP;
    line_list.scale.x = 0.06; 
    line_list.scale.y = 0.06;
    line_list.color.r=1.0;
    line_list.color.a=1.0;
    geometry_msgs::Point p1; 
    p1.x = last_pos(0,0);
    p1.y = last_pos(1,0);
    p1.z = last_pos(2,0);
    line_list.points.push_back(p1);
    last_position = position;
    last_timestamp = current_timestamp;
    path_vis_publisher.publish(line_list);
    /*
    line_list.type = visualization_msgs::Marker::LINE_LIST;
    // LINE_STRIP/LINE_LIST markers use only the x component of scale, for the line width
    line_list.scale.x = 0.01;
    // Line list is red
    line_list.color.r = 1.0;
    line_list.color.g = strange;
    line_list.color.a = 1.0;
    // Create the vertices for the points and lines
    
    //printf("Start point\n");
    geometry_msgs::Point p1; 
    p1.x = last_pos(0,0);
    p1.y = last_pos(1,0);
    p1.z = last_pos(2,0);
    geometry_msgs::Point p2;
    p2.x = current_pos(0,0);
    p2.y = current_pos(1,0);
    p2.z = current_pos(2,0);

 
    // The line list needs two points for each line
    line_list.points.push_back(p1);
    line_list.points.push_back(p2);
    //printf("Finish point\n");
    last_position = position;
    last_timestamp = current_timestamp;
    path_vis_publisher.publish(line_list);
    */
   // printf("Publish Successfully\n");

}

double time_difference(ros::Time now, ros::Time last_time){
    double difference =0;
    difference = now.sec-last_time.sec + (now.nsec-last_time.nsec)/1000000000.0;

    return difference;
}

int   main (int argc, char** argv)
{
     // Initialize ROS
     ros::init (argc, argv, "hw3_0660027");
     ros::NodeHandle nh;   
     initial_measure = ros::topic::waitForMessage<sensor_msgs::Imu>("/imu/data", ros::Duration(10));
     last_timestamp = initial_measure->header.stamp;
     position(0,0)=0; position(1,0)=0; position(2,0)=0;
     velocity(0,0)=0; velocity(1,0)=0; velocity(2,0)=0;
     initial_linear_ac(0,0) = initial_measure->linear_acceleration.x; initial_linear_ac(1,0) = initial_measure->linear_acceleration.y; initial_linear_ac(2,0) = initial_measure->linear_acceleration.z;
     last_position(0,0) = 0; last_position(1,0) = 0; last_position(2,0) = 0; 
     // Create a ROS subscriber for the input point cloud
     ros::Subscriber model_subscriber = nh.subscribe<sensor_msgs::Imu> ("/imu/data", 10, imu_cb);
     path_vis_publisher = nh.advertise<visualization_msgs::Marker>("/map/imu_path", 10);
     // Spin
     ros::spin ();
}

