// Copyright 2019 Intelligent Robotics Lab
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef PLANSYS2_EXECUTOR__EXECUTORNODE_HPP_
#define PLANSYS2_EXECUTOR__EXECUTORNODE_HPP_

#include <memory>
#include <vector>
#include <string>
#include <map>

#include <plansys2_domain_expert/DomainExpertClient.hpp>
#include <plansys2_problem_expert/ProblemExpertClient.hpp>
#include <plansys2_planner/PlannerClient.hpp>
#include <plansys2_executor/ActionExecutor.hpp>
#include <plansys2_executor/BTBuilder.hpp>

//#include "lifecycle_msgs/state.hpp"
//#include "lifecycle_msgs/transition.hpp"

#include <plansys2_msgs/ExecutePlan.h>
#include <plansys2_msgs/ActionExecutionInfo.h>
#include <plansys2_msgs/GetOrderedSubGoals.h>
#include <plansys2_msgs/Plan.h>
#include <std_msgs/String.h>

#include <ros/ros.h>
//#include "rclcpp_action/rclcpp_action.hpp"
#include <lifecycle/managed_node.h>
#include <lifecycle/lifecycle_publisher.h>

#include "pluginlib/class_loader.hpp"
#include "pluginlib/class_list_macros.hpp"

namespace plansys2
{

class ExecutorNode : public ros::lifecycle::ManagedNode
{
public:

  ExecutorNode();

  bool onConfigure();
  bool onActivate();
  bool onDeactivate();
  bool onCleanup();
  bool onShutdown();
  bool onError(std::exception &);

  void get_ordered_sub_goals_service_callback(
    plansys2_msgs::GetOrderedSubGoals::Request &request,
    plansys2_msgs::GetOrderedSubGoals::Response &response);

  void get_plan_service_callback(
    plansys2_msgs::GetPlan::Request &request,
    plansys2_msgs::GetPlan::Response &response);

protected:
  std::shared_ptr<ros::NodeHandle> node_;

  bool cancel_plan_requested_;
  std::optional<plansys2_msgs::Plan> current_plan_;
  std::optional<std::vector<plansys2_msgs::Tree>> ordered_sub_goals_;

  std::string action_bt_xml_;
  std::string start_action_bt_xml_;
  std::string end_action_bt_xml_;
  pluginlib::ClassLoader<plansys2::BTBuilder> bt_builder_loader_;

  std::shared_ptr<plansys2::DomainExpertClient> domain_client_;
  std::shared_ptr<plansys2::ProblemExpertClient> problem_client_;
  std::shared_ptr<plansys2::PlannerClient> planner_client_;

  rclcpp_lifecycle::LifecyclePublisher<plansys2_msgs::ActionExecutionInfo>::SharedPtr
    execution_info_pub_;
  rclcpp_lifecycle::LifecyclePublisher<plansys2_msgs::Plan>::SharedPtr executing_plan_pub_;

  std::shared_ptr<actionlib::SimpleActionServer<plansys2_msgs::ExecutePlanAction> > execute_plan_action_server_;
  rclcpp::Service<plansys2_msgs::srv::GetOrderedSubGoals>::SharedPtr
    get_ordered_sub_goals_service_;
  rclcpp_lifecycle::LifecyclePublisher<std_msgs::String>::SharedPtr dotgraph_pub_;

  std::optional<std::vector<plansys2_msgs::Tree>> getOrderedSubGoals();

  rclcpp::Service<plansys2_msgs::srv::GetPlan>::SharedPtr get_plan_service_;

  rclcpp_action::GoalResponse handle_goal(
    const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const ExecutePlan::Goal> goal);

  rclcpp_action::CancelResponse handle_cancel(
    const std::shared_ptr<GoalHandleExecutePlan> goal_handle);

  void execute(const std::shared_ptr<GoalHandleExecutePlan> goal_handle);

  void handle_accepted(const std::shared_ptr<GoalHandleExecutePlan> goal_handle);

  std::vector<plansys2_msgs::ActionExecutionInfo> get_feedback_info(
    std::shared_ptr<std::map<std::string, ActionExecutionInfo>> action_map);

  void print_execution_info(
    std::shared_ptr<std::map<std::string, ActionExecutionInfo>> exec_info);
};

}  // namespace plansys2

#endif  // PLANSYS2_EXECUTOR__EXECUTORNODE_HPP_
