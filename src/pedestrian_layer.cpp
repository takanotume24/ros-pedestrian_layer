#include <pedestrian_layers/pedestrian_layer.h>
#include <pedsim_msgs/AgentStates.h>
#include <pluginlib/class_list_macros.h>
#include <sensor_msgs/Imu.h>

PLUGINLIB_EXPORT_CLASS(pedestrian_layer_namespace::PedestrianLayer,
                       costmap_2d::Layer)

using costmap_2d::FREE_SPACE;
using costmap_2d::LETHAL_OBSTACLE;

namespace pedestrian_layer_namespace {

PedestrianLayer::PedestrianLayer() {}

void PedestrianLayer::pedestrian_callback(
    const pedsim_msgs::AgentStatesConstPtr &msg) {
  std::lock_guard<std::mutex> lock(m);
  states = *msg;
  // ROS_INFO("agents:\t%d", msg->agent_states.size());
}

void PedestrianLayer::onInitialize() {
  ros::NodeHandle nh("~/" + name_), g_nh;
  current_ = true;

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
  if (states.agent_states.size() == 0) return;

  geometry_msgs::PoseStamped target_pose;
  geometry_msgs::PoseStamped source_pose;
  source_pose.header.frame_id = "odom";
  source_pose.header.stamp = states.agent_states.at(0).header.stamp;
  source_pose.header.seq = states.agent_states.at(0).header.seq;
  source_pose.pose = states.agent_states.at(0).pose;
  source_pose.pose.orientation.w = 1.0;

  auto target_frame = "map";

  try {
    tf_listener.waitForTransform(source_pose.header.frame_id, target_frame,
                                 source_pose.header.stamp, ros::Duration(1.0));
    tf_listener.transformPose(target_frame, source_pose, target_pose);
  } catch (std::exception e) {
    ROS_ERROR(e.what());
  }

  pose_history.push_back(target_pose);

  mark_x_ = target_pose.pose.position.x;
  mark_y_ = target_pose.pose.position.y;

  *min_x = std::min(*min_x, mark_x_);
  *min_y = std::min(*min_y, mark_y_);
  *max_x = std::max(*max_x, mark_x_);
  *max_y = std::max(*max_y, mark_y_);
}

void PedestrianLayer::change_cost(
    costmap_2d::Costmap2D &master_grid,
    const geometry_msgs::PoseStamped &pose_stamped, const unsigned char &cost) {
  unsigned int mx;
  unsigned int my;

  auto radius = 10;

  mark_x_ = pose_stamped.pose.position.x;
  mark_y_ = pose_stamped.pose.position.y;

  if (!master_grid.worldToMap(mark_x_, mark_y_, mx, my)) {
    ROS_ERROR("Skiped");
    return;
  }

  // ROS_INFO("mx = %d,\tmy = %d", mx, my);

  for (int i = 0; i < radius; i++) {
    for (int j = 0; j < radius; j++) {
      int x = mx + i - radius / 2;
      int y = my + j - radius / 2;

      if (x < 0 || master_grid.getSizeInCellsX() < x) continue;
      if (y < 0 || master_grid.getSizeInCellsY() < y) continue;

      ROS_INFO("x = %d, y = %d", x, y);

      master_grid.setCost(x, y, cost);
    }
  }
}

void PedestrianLayer::add_cost(costmap_2d::Costmap2D &master_grid,
                               const geometry_msgs::PoseStamped &pose_stamped) {
  change_cost(master_grid, pose_stamped, LETHAL_OBSTACLE);
}

void PedestrianLayer::delete_cost(
    costmap_2d::Costmap2D &master_grid,
    const geometry_msgs::PoseStamped &pose_stamped) {
  change_cost(master_grid, pose_stamped, FREE_SPACE);
  ROS_INFO("cleard -> x:%f, y:%f", pose_stamped.pose.position.x,
           pose_stamped.pose.position.y);
}

void PedestrianLayer::updateCosts(costmap_2d::Costmap2D &master_grid, int min_i,
                                  int min_j, int max_i, int max_j) {
  if (!enabled_) return;
  if (pose_history.size() == 0) return;

  ros::spinOnce();

  auto result = std::remove_if(pose_history.begin(), pose_history.end(),
                               [](geometry_msgs::PoseStamped x) {
                                 auto diff = ros::Time::now() - x.header.stamp;
                                 return diff.toSec() > 1.0;
                               });

  for (auto iter = result, last = pose_history.end(); iter != last; ++iter) {
    delete_cost(master_grid, *iter);
    ROS_INFO("deleted: %f", (ros::Time::now() - (*iter).header.stamp).toSec());
    ROS_INFO("deleted: %d", (*iter).header.seq);
  }

  pose_history.erase(result, pose_history.end());

  for (const auto &iter : pose_history) {
    // ROS_INFO("size: %d\t%f\t%f", pose_history.size(), iter.pose.position.x,
    //          iter.pose.position.y);
    add_cost(master_grid, iter);
  }
}

}  // namespace pedestrian_layer_namespace