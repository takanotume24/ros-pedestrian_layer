#ifndef PEDESTRIAN_LAYER_H_
#define PEDESTRIAN_LAYER_H_
#include <costmap_2d/GenericPluginConfig.h>
#include <costmap_2d/layer.h>
#include <costmap_2d/layered_costmap.h>
#include <dynamic_reconfigure/server.h>
#include <pedsim_msgs/AgentStates.h>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <mutex>


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
  void imu_callback(const sensor_msgs::ImuConstPtr &msg);
  void pedestrian_callback(const pedsim_msgs::AgentStatesConstPtr &msg);

  double mark_x_, mark_y_;
  pedsim_msgs::AgentStates states;
  ros::Subscriber imu_sub_, pedestrian_sub_;
  std::mutex m;
  dynamic_reconfigure::Server<costmap_2d::GenericPluginConfig>* dsrv_;
};
}  // namespace pedestrian_layer_namespace
#endif
