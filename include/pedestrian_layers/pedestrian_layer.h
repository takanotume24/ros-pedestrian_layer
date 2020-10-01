#ifndef PEDESTRIAN_LAYER_H_
#define PEDESTRIAN_LAYER_H_
#include <costmap_2d/GenericPluginConfig.h>
#include <costmap_2d/layer.h>
#include <costmap_2d/layered_costmap.h>
#include <dynamic_reconfigure/server.h>
#include <pedsim_msgs/AgentStates.h>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <pedsim_msgs/AgentStates.h>
#include <pluginlib/class_list_macros.h>
#include <sensor_msgs/Imu.h>

#include <map>
#include <mutex>
#include <deque>

#include "tf/transform_broadcaster.h"
#include "tf/transform_listener.h"

namespace pedestrian_layer_namespace {

class PedestrianLayer : public costmap_2d::Layer {
 public:
  PedestrianLayer();

  virtual void onInitialize();
  virtual void updateBounds(double robot_x, double robot_y, double robot_yaw,
                            double* min_x, double* min_y, double* max_x,
                            double* max_y);
  virtual void updateCosts(costmap_2d::Costmap2D& master_grid, int min_i,
                           int min_j, int max_i, int max_j);

 private:
  void reconfigureCB(costmap_2d::GenericPluginConfig& config, uint32_t level);
  void pedestrian_callback(const pedsim_msgs::AgentStatesConstPtr& msg);
  void add_cost(costmap_2d::Costmap2D& master_grid,
                const geometry_msgs::PoseStamped& pose_stamped);
  void delete_cost(costmap_2d::Costmap2D& master_grid,
                   const geometry_msgs::PoseStamped& pose_stamped);
  void change_cost(
      costmap_2d::Costmap2D& master_grid,
      const geometry_msgs::PoseStamped& pose_stamped, const unsigned char& cost);
  
  double mark_x_, mark_y_;
  pedsim_msgs::AgentStates states;
  std::deque<pedsim_msgs::AgentStates> states_history;
  std::deque<geometry_msgs::PoseStamped> pose_history;
  ros::Subscriber imu_sub_, pedestrian_sub_;
  std::mutex m;
  dynamic_reconfigure::Server<costmap_2d::GenericPluginConfig>* dsrv_;
  tf::TransformListener tf_listener;
  tf::TransformListener ln;
};
}  // namespace pedestrian_layer_namespace
#endif
