#include <ros/ros.h>
#include <iostream>
#include "rrt_star_rs/rrt_star_rs_flow.h"

int main(int argc, char **argv){
  ros::init(argc,argv,"rrt_search");
  ros::NodeHandle node_handle("~");
  RRTStarRSFlow kinodynamic_rrt_flow(node_handle);
  
  ros::Rate rate(0.5); //0.5Hz
  kinodynamic_rrt_flow.Run();
  while(ros::ok()){
    kinodynamic_rrt_flow.PublishRviz();
    rate.sleep();
  }

  return 0;
}