#include <ros/ros.h>
#include <iostream>

int main(int argc, char **argv){
  ros::init(argc,argv,"test_node");
  ros::NodeHandle node_handle("~");
  ros::Rate rate(0.5);
  while(ros::ok()){
    std::cout<<"test~"<<std::endl;
    rate.sleep();
  }
  return 0;
}