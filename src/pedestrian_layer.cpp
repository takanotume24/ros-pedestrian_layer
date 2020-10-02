#include <pedestrian_layers/pedestrian_layer.h>

PLUGINLIB_EXPORT_CLASS(pedestrian_layer_namespace::PedestrianLayer,
                       costmap_2d::Layer)

using costmap_2d::FREE_SPACE;
using costmap_2d::LETHAL_OBSTACLE;

namespace pedestrian_layer_namespace {

PedestrianLayer::PedestrianLayer() {}

void PedestrianLayer::pedestrian_callback(
    const pedsim_msgs::AgentStatesConstPtr &msg) {
  std::lock_guard<std::mutex> lock(m_);
  states_ = *msg;
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
  min_max_initialized_ = false;
}

void PedestrianLayer::reconfigureCB(costmap_2d::GenericPluginConfig &config,
                                    uint32_t level) {
  enabled_ = config.enabled;
}

void PedestrianLayer::updateBounds(double robot_x, double robot_y,
                                   double robot_yaw, double *min_x,
                                   double *min_y, double *max_x,
                                   double *max_y) {
  std::lock_guard<std::mutex> lock(m_);

  if (!enabled_) return;
  if (states_.agent_states.size() == 0) return;

  if (!min_max_initialized_) {
    *min_x = 0.0;
    *min_y = 0.0;
    *max_x = 10000.0;
    *max_y = 10000.0;
    min_max_initialized_ = true;
    return;
  }

  for (const auto &agent_state : states_.agent_states) {
    geometry_msgs::PoseStamped target_pose;
    geometry_msgs::PoseStamped source_pose;
    source_pose.header = agent_state.header;
    source_pose.pose = agent_state.pose;
    source_pose.pose.orientation.w = 1.0;

    auto target_frame = "map";

    try {
      tf_listener_.waitForTransform(source_pose.header.frame_id, target_frame,
                                   source_pose.header.stamp,
                                   ros::Duration(1.0));
      tf_listener_.transformPose(target_frame, source_pose, target_pose);
    } catch (std::exception e) {
      ROS_ERROR("%s", e.what());
    }

    pose_histories_[agent_state.id].push_back(target_pose);

    *min_x = std::min(*min_x, target_pose.pose.position.x);
    *min_y = std::min(*min_y, target_pose.pose.position.y);
    *max_x = std::max(*max_x, target_pose.pose.position.x);  //最大に設定する。
    *max_y = std::max(*max_y, target_pose.pose.position.y);
    *min_x = 0.0;
    *min_y = 0.0;
    *max_x = 10000.0;
    *max_y = 10000.0;
  }
}

inline void PedestrianLayer::change_cost(
    costmap_2d::Costmap2D &master_grid,
    const geometry_msgs::PoseStamped &pose_stamped, const unsigned char &cost) {
  unsigned int mx;
  unsigned int my;

  const auto radius = 10;
  const auto grid_size_x = master_grid.getSizeInCellsX();
  const auto grid_size_y = master_grid.getSizeInCellsY();

  const auto mark_x = pose_stamped.pose.position.x;
  const auto mark_y = pose_stamped.pose.position.y;

  if (!master_grid.worldToMap(mark_x, mark_y, mx, my)) {
    // ROS_INFO("Skiped, mark_x = %lf, mark_y = %lf, mx = %d, my = %d",
    //                   mark_x, mark_y, mx, my);
    return;
  }

  // ROS_INFO("mx = %d,\tmy = %d", mx, my);

  for (int i = 0; i < radius; i++) {
    for (int j = 0; j < radius; j++) {
      int x = mx + i - radius / 2;
      int y = my + j - radius / 2;

      if (x < 0) continue;
      if (y < 0) continue;
      
      master_grid.setCost(x, y, cost);
    }
  }
}

inline void PedestrianLayer::add_cost(
    costmap_2d::Costmap2D &master_grid,
    const geometry_msgs::PoseStamped &pose_stamped) {
  change_cost(master_grid, pose_stamped, LETHAL_OBSTACLE);
}

inline void PedestrianLayer::delete_cost(
    costmap_2d::Costmap2D &master_grid,
    const geometry_msgs::PoseStamped &pose_stamped) {
  change_cost(master_grid, pose_stamped, costmap_2d::FREE_SPACE);
}

void PedestrianLayer::updateCosts(costmap_2d::Costmap2D &master_grid, int min_i,
                                  int min_j, int max_i, int max_j) {
  if (!enabled_) return;
  if (pose_histories_.size() == 0) return;

  for (auto &pose_history_map : pose_histories_) {
    auto &pose_history = pose_history_map.second;

    auto result = std::remove_if(pose_history.begin(), pose_history.end(),
                                 [](geometry_msgs::PoseStamped x) {
                                   auto diff =
                                       ros::Time::now() - x.header.stamp;
                                   auto capture_sec = 3.0;

                                   return diff.toSec() > capture_sec;
                                 });

    for (const auto &iter : pose_history) {
      delete_cost(master_grid, iter);
    }

    pose_history.erase(result, pose_history.end());

    auto diff_x = pose_history.end()->pose.position.x -
                  pose_history.begin()->pose.position.x;
    auto diff_y = pose_history.end()->pose.position.y -
                  pose_history.begin()->pose.position.y;

    std::deque<geometry_msgs::PoseStamped> predict_deque;

    for (const auto &iter : pose_history) {
      auto pose = iter;
      pose.pose.position.x += diff_x;
      pose.pose.position.y += diff_y;
      predict_deque.push_back(pose);
    }

    // ROS_INFO("%d", pose_history.size());

    for (const auto &iter : pose_history) {
      // add_cost(master_grid, iter);
    }
    for (const auto &iter : predict_deque) {
      add_cost(master_grid, iter);
    }
  }
}

}  // namespace pedestrian_layer_namespace