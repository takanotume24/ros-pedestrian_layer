#include <pedestrian_layers/pedestrian_layer.h>
#include <pedsim_msgs/AgentStates.h>
#include <pluginlib/class_list_macros.h>
#include <sensor_msgs/Imu.h>

PLUGINLIB_EXPORT_CLASS(pedestrian_layer_namespace::PedestrianLayer,
                       costmap_2d::Layer)

using costmap_2d::LETHAL_OBSTACLE;

namespace pedestrian_layer_namespace {

PedestrianLayer::PedestrianLayer() {}

void PedestrianLayer::imu_callback(const sensor_msgs::ImuConstPtr &msg) {
  // ROS_INFO("called\t%f", msg->linear_acceleration.x);
}

void PedestrianLayer::pedestrian_callback(
    const pedsim_msgs::AgentStatesConstPtr &msg) {
      std::lock_guard<std::mutex> lock(m);
      states = *msg;
  ROS_INFO("agents:\t%d", msg->agent_states.size());
}

void PedestrianLayer::onInitialize() {
  ros::NodeHandle nh("~/" + name_), g_nh;
  current_ = true;

  // ros::Subscriber sub = nh.subscribe("/pedsim_simulator/simulated_agents",
  // 10,
  //                                     pedestrian_callback);
  imu_sub_ = g_nh.subscribe("/imu", 10, &PedestrianLayer::imu_callback, this);
  pedestrian_sub_ = g_nh.subscribe("/pedsim_simulator/simulated_agents", 10,
                                   &PedestrianLayer::pedestrian_callback, this);
  dsrv_ = new dynamic_reconfigure::Server<costmap_2d::GenericPluginConfig>(nh);
  dynamic_reconfigure::Server<costmap_2d::GenericPluginConfig>::CallbackType
      cb = boost::bind(&PedestrianLayer::reconfigureCB, this, _1, _2);
  dsrv_->setCallback(cb);
}

void PedestrianLayer::reconfigureCB(costmap_2d::GenericPluginConfig &config,
                                    uint32_t level) {
  enabled_ = config.enabled;
}

void PedestrianLayer::updateBounds(double robot_x, double robot_y,
                                   double robot_yaw, double *min_x,
                                   double *min_y, double *max_x,
                                   double *max_y) {
  if (!enabled_) return;

  mark_x_ = robot_x + cos(robot_yaw);
  mark_y_ = robot_y + sin(robot_yaw);

  *min_x = std::min(*min_x, mark_x_);
  *min_y = std::min(*min_y, mark_y_);
  *max_x = std::max(*max_x, mark_x_);
  *max_y = std::max(*max_y, mark_y_);
}

void PedestrianLayer::updateCosts(costmap_2d::Costmap2D &master_grid, int min_i,
                                  int min_j, int max_i, int max_j) {
  if (!enabled_) return;

  ros::spinOnce();
  unsigned int mx;
  unsigned int my;

  if (master_grid.worldToMap(mark_x_, mark_y_, mx, my)) {
    ROS_INFO("mx: %d\tmy: %d\tmark_x_: %f\tmark_y_: %f", mx, my, mark_x_,
             mark_y_);
    ROS_INFO("%d", states.agent_states.at(0).twist.linear.x);
    // master_grid.setCost(mx, my, LETHAL_OBSTACLE);
  }
}

}  // namespace pedestrian_layer_namespace