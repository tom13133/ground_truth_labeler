#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2_msgs/TFMessage.h>
#include <geometry_msgs/Point.h>
#include <std_msgs/String.h>
#include <vector>
#include <Eigen/Dense>
#include "Eigen/Eigen"
#include <interactive_markers/interactive_marker_server.h>

using namespace visualization_msgs;

interactive_markers::InteractiveMarkerServer *_server;
geometry_msgs::TransformStamped transform;
std::string time_stamp;
std::vector<geometry_msgs::Pose> points;

void processFeedback(const InteractiveMarkerFeedbackConstPtr &feedback) {
  ROS_INFO_STREAM(feedback->marker_name << " is now at "
                  << feedback->pose.position.x << ", "
                  << feedback->pose.position.y << ", "
                  << feedback->pose.position.z << ", "
                  << feedback->pose.orientation.x << ", "
                  << feedback->pose.orientation.y << ", "
                  << feedback->pose.orientation.z << ", "
                  << feedback->pose.orientation.w);
  geometry_msgs::Pose point;
  point = feedback->pose;
  int index = std::stoi(feedback->marker_name);
  points[index] = point;
}

void tfCallback(const tf2_msgs::TFMessage &msg) {
  transform = msg.transforms[0];
}
  

void makeBoxControl(InteractiveMarker &msg) {
  int idx = points.size() - 1;
  Marker marker, line_marker, text_marker;

  // center sphere
  marker.type = Marker::SPHERE;
  marker.scale.x = 0.05;
  marker.scale.y = 0.05;
  marker.scale.z = 0.05;
  marker.color.r = 0.0;
  marker.color.g = 1.0;
  marker.color.b = 0.0;
  marker.color.a = 1.0;

  // marker ID
  text_marker.type = Marker::TEXT_VIEW_FACING;
  text_marker.scale.z = 0.45;
  text_marker.text = std::to_string(idx);
  text_marker.color.r = 1.0;
  text_marker.color.g = 1.0;
  text_marker.color.b = 1.0;
  text_marker.color.a = 1.0;
  text_marker.pose.position.x = 0;
  text_marker.pose.position.y = 0;
  text_marker.pose.position.z = 1.5;
  text_marker.pose.orientation.x = 0.0;
  text_marker.pose.orientation.y = 0.0;
  text_marker.pose.orientation.z = 0.0;
  text_marker.pose.orientation.w = 1.0;

  // Triangle model
  line_marker.type = Marker::LINE_STRIP;
  line_marker.action = visualization_msgs::Marker::ADD;

  line_marker.scale.x = 0.02;

  line_marker.color.r = 1.0;
  line_marker.color.g = 1.0;
  line_marker.color.b = 1.0;
  line_marker.color.a = 1.0;

  Eigen::Vector4d p1 = Eigen::Vector4d(0, 0, 0.6, 1);
  Eigen::Vector4d p2 = Eigen::Vector4d(+0.5196, 0, -0.3, 1);
  Eigen::Vector4d p3 = Eigen::Vector4d(-0.5196, 0, -0.3, 1);

  geometry_msgs::Point p1_, p2_, p3_;
  p1_.x = p1.x();
  p1_.y = p1.y();
  p1_.z = p1.z();

  p2_.x = p2.x();
  p2_.y = p2.y();
  p2_.z = p2.z();

  p3_.x = p3.x();
  p3_.y = p3.y();
  p3_.z = p3.z();

  line_marker.points.push_back(p1_);
  line_marker.points.push_back(p2_);
  line_marker.points.push_back(p3_);
  line_marker.points.push_back(p1_);


  InteractiveMarkerControl box_control;
  box_control.always_visible = true;

  box_control.markers.push_back( marker );
  box_control.markers.push_back( text_marker );
  box_control.markers.push_back( line_marker );
  msg.controls.push_back(box_control);

  InteractiveMarkerControl control;
  control.orientation.w = 1;
  control.orientation.x = 0;
  control.orientation.y = 1;
  control.orientation.z = 0;

  control.name = "move_xy";
  control.interaction_mode = InteractiveMarkerControl::MOVE_PLANE;

  Marker color_marker;
  color_marker.type = Marker::CYLINDER;
  color_marker.color.r = 1.0;
  color_marker.color.r = 1.0;
  color_marker.color.g = 1.0;
  color_marker.color.b = 1.0;
  color_marker.color.a = 0.75;
  color_marker.scale.x = 0.5;
  color_marker.scale.y = 0.5;
  color_marker.scale.z = 0.005;
  control.markers.push_back(color_marker);

  msg.controls.push_back(control);

  InteractiveMarkerControl z_control;
  z_control.orientation.w = 1;
  z_control.orientation.x = 0;
  z_control.orientation.y = 1;
  z_control.orientation.z = 0;
  z_control.name = "move_z";
  z_control.interaction_mode = InteractiveMarkerControl::MOVE_AXIS;

  msg.controls.push_back(z_control);

  InteractiveMarkerControl rotx_control;
  rotx_control.always_visible = true;
  rotx_control.interaction_mode = visualization_msgs::InteractiveMarkerControl::ROTATE_AXIS;
  rotx_control.orientation.x = 1;
  rotx_control.orientation.y = 0;
  rotx_control.orientation.z = 0;
  rotx_control.orientation.w = 1;
  rotx_control.name = "rot_x";
  msg.controls.push_back(rotx_control);

  InteractiveMarkerControl roty_control;
  roty_control.always_visible = true;
  roty_control.interaction_mode = visualization_msgs::InteractiveMarkerControl::ROTATE_AXIS;
  roty_control.orientation.x = 0;
  roty_control.orientation.y = 1;
  roty_control.orientation.z = 0;
  roty_control.orientation.w = 1;
  roty_control.name = "rot_y";
  msg.controls.push_back(roty_control);

  InteractiveMarkerControl rotz_control;
  rotz_control.always_visible = true;
  rotz_control.interaction_mode = visualization_msgs::InteractiveMarkerControl::ROTATE_AXIS;
  rotz_control.orientation.x = 0;
  rotz_control.orientation.y = 0;
  rotz_control.orientation.z = 1;
  rotz_control.orientation.w = 1;
  rotz_control.name = "rot_z";
  msg.controls.push_back(rotz_control);
}

void navCallback(const geometry_msgs::PoseStamped msg) {
  std::cout << "(i)   Input 1 to add markers" << std::endl;
  std::cout << "(ii)  Input 2 to erase marker" << std::endl;
  std::cout << "(iii) Input 3 to show positions" << std::endl;
  std::cout << "(iv)  Else to do nothing, or \"CTRL+C\" to stop." << std::endl;

  int choice;
  int number;
  scanf("%d", &choice);

  if (choice == 1) {
    std::cout << "Please input the number of markers:" << std::endl;
    scanf("%d", &number);
    for (int i = 0; i < number; i++) {
      InteractiveMarker int_marker;
      int_marker.header.frame_id = "velodyne";
      int_marker.header.stamp = ros::Time::now();
      int_marker.name = std::to_string(points.size());
      int_marker.description = "";

      
      int_marker.pose.position.x = i * 2;
      int_marker.pose.position.y = 0;
      int_marker.pose.position.z = 0;
      geometry_msgs::Pose temp;
      temp.position.x = int_marker.pose.position.x;
      temp.position.y = 0;
      temp.position.z = 0;
      points.push_back(temp);

      makeBoxControl(int_marker);
      _server->insert(int_marker, &processFeedback);
    }
  }
  else if (choice == 2) {
    std::cout<<"Please input the ID you want to erase:"<<std::endl;
    int erase_id;
    scanf("%d", &erase_id);
    _server->erase(std::to_string(erase_id));

    points[erase_id].position.x = -1000;
    points[erase_id].position.y = -1000;
    points[erase_id].position.z = -1000;
  }
  else if (choice == 3) {
    for(int i = 0; i < points.size(); i++) {
      if (points[i].position.x == -1000 && points[i].position.y == -1000 && points[i].position.z == -1000) {
        continue;
      }
      else {
        if (!time_stamp.empty())
          std::cout << time_stamp << ", ";
        std::cout << i << ", " << points[i].position.x << ", "
                               << points[i].position.y << ", "
                               << points[i].position.z << std::endl;
      }
    }
  }
  else {
    std::cout<<"Do nothing."<<std::endl;
  }
  _server->applyChanges();
}

void stampCallback(const std_msgs::String& stamp) {
  time_stamp = stamp.data;
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "simple_marker");
  ros::NodeHandle n;
  ros::Subscriber sub_nav = n.subscribe("/move_base_simple/goal", 1, navCallback);
  ros::Subscriber sub_tf = n.subscribe("/tf", 1, tfCallback);
  ros::Subscriber sub_stamp = n.subscribe("/visualizer_node/stamp", 1, stampCallback);

  interactive_markers::InteractiveMarkerServer server("simple_marker");
  _server = &server;

  ros::spin();
}