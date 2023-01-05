#ifndef NAV2_THETA_STAR_PLANNER__THETA_STAR_PLANNER_HPP_
#define NAV2_THETA_STAR_PLANNER__THETA_STAR_PLANNER_HPP_

#include <iostream>
#include <cmath>
#include <string>
#include <chrono>
#include <queue>
#include <algorithm>
#include <memory>
#include <vector>
#include "rclcpp/rclcpp.hpp"
#include "nav2_core/global_planner.hpp"
#include "nav2_core/planner_exceptions.hpp"
#include "nav_msgs/msg/path.hpp"
#include "nav2_util/robot_utils.hpp"
#include "nav2_util/lifecycle_node.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "nav2_costmap_2d/cost_values.hpp"
#include "nav2_util/node_utils.hpp"
#include "nav2_theta_star_planner/theta_star.hpp"
#include "nav2_util/geometry_utils.hpp"

using rcl_interfaces::msg::ParameterType;

namespace nav2_theta_star_planner
{

class ThetaStarPlanner : public nav2_core::GlobalPlanner
{
public:
  /**
    * @brief Configuring plugin
    * @param parent Lifecycle node pointer
    * @param name Name of plugin map
    * @param tf Shared ptr of TF2 buffer
    * @param costmap_ros Costmap2DROS object
    */  
  void configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
    std::string name, std::shared_ptr<tf2_ros::Buffer> tf,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override;

  /**
   * @brief Cleanup lifecycle node
   */
  void cleanup() override;

  /**
   * @brief Activate lifecycle node
   */
  void activate() override;

  /**
   * @brief Deactivate lifecycle node
   */
  void deactivate() override;

  /**
   * @brief Creating a plan from start and goal poses Èë¿Úº¯Êý
   * @param start Start pose
   * @param goal Goal pose
   * @return nav_msgs::Path of the generated path
   */
  nav_msgs::msg::Path createPlan(
    const geometry_msgs::msg::PoseStamped & start,
    const geometry_msgs::msg::PoseStamped & goal) override;

protected:
  std::shared_ptr<tf2_ros::Buffer> tf_;
  rclcpp::Clock::SharedPtr clock_;
  rclcpp::Logger logger_{rclcpp::get_logger("ThetaStarPlanner")};
  std::string global_frame_, name_;
  bool use_final_approach_orientation_;

  // parent node weak ptr
  rclcpp_lifecycle::LifecycleNode::WeakPtr parent_node_;

  std::unique_ptr<theta_star::ThetaStar> planner_;

  // Dynamic parameters handler
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr dyn_params_handler_;


  /**
   * @brief the function responsible for calling the algorithm and retrieving a path from it
   * @return global_path is the planned path to be taken
   */
  void getPlan(nav_msgs::msg::Path & global_path);

  /**
   * @brief interpolates points between the consecutive waypoints of the path
   * @param raw_path is used to send in the path received from the planner
   * @param dist_bw_points is used to send in the interpolation_resolution (which has been set as the costmap resolution)
   * @return the final path with waypoints at a distance of the value of interpolation_resolution of each other
   */
  static nav_msgs::msg::Path linearInterpolation(
    const std::vector<coordsW> & raw_path,
    const double & dist_bw_points);

  /**
   * @brief Callback executed when a paramter change is detected
   * @param parameters list of changed parameters
   */
  rcl_interfaces::msg::SetParametersResult
  dynamicParametersCallback(std::vector<rclcpp::Parameter> parameters);
};
}   //  namespace nav2_theta_star_planner

#endif  //  NAV2_THETA_STAR_PLANNER__THETA_STAR_PLANNER_HPP_
