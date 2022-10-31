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

#include <plansys2_domain_expert/DomainExpertClient.hpp>

#include <optional>
#include <algorithm>
#include <string>
#include <vector>
#include <memory>

namespace plansys2
{

DomainExpertClient::DomainExpertClient()
{
  node_ = ros::NodeHandle("domain_expert_client");

  get_domain_client_ = node_.serviceClient<plansys2_msgs::GetDomain>("domain_expert/get_domain");
  get_name_client_ = node_.serviceClient<plansys2_msgs::GetDomainName>("domain_expert/get_domain_name");
  get_types_client_ = node_.serviceClient<plansys2_msgs::GetDomainTypes>("domain_expert/get_domain_types");
  get_constants_client_ = node_.serviceClient<plansys2_msgs::GetDomainConstants>("domain_expert/get_domain_constants");
  get_predicates_client_ = node_.serviceClient<plansys2_msgs::GetStates>("domain_expert/get_domain_predicates");
  get_functions_client_ = node_.serviceClient<plansys2_msgs::GetStates>("domain_expert/get_domain_functions");
  get_actions_client_ = node_.serviceClient<plansys2_msgs::GetDomainActions>("domain_expert/get_domain_actions");
  get_durative_actions_client_ = node_.serviceClient<plansys2_msgs::GetDomainActions>("domain_expert/get_domain_durative_actions");
  get_predicate_details_client_ = node_.serviceClient<plansys2_msgs::GetNodeDetails>("domain_expert/get_domain_predicate_details");
  get_function_details_client_ = node_.serviceClient<plansys2_msgs::GetNodeDetails>("domain_expert/get_domain_function_details");
  get_action_details_client_ = node_.serviceClient<plansys2_msgs::GetDomainActionDetails>("domain_expert/get_domain_action_details");
  get_durative_action_details_client_ = node_.serviceClient<plansys2_msgs::GetDomainDurativeActionDetails>("domain_expert/get_domain_durative_action_details");
}

std::string DomainExpertClient::getNodeName()
{
  return std::string("domain_expert_client");
}

std::string DomainExpertClient::getName()
{
  while (!get_name_client_.waitForExistence(ros::Duration(1.0)))
    ROS_INFO_STREAM(getNodeName() << " Waiting for service " << get_name_client_.getService() << " to appear");    

  plansys2_msgs::GetDomainName srv;
  get_name_client_.call(srv);

  return srv.result.name;
}

std::vector<std::string> DomainExpertClient::getTypes()
{
  while (!get_types_client_.waitForExistence(ros::Duration(1.0)))
    ROS_INFO_STREAM(getNodeName() << " Waiting for service " << get_types_client_.getService() << " to appear");    

  
  plansys2_msgs::GetDomainTypes srv;
  get_types_client_.call(srv);

  return srv.result.types;
}

std::vector<std::string> DomainExpertClient::getConstants(const std::string & type)
{
  while (!get_constants_client_.waitForExistence(ros::Duration(1.0)))
    ROS_INFO_STREAM(getNodeName() << " Waiting for service " << get_constants_client_.getService() << " to appear");    

  plansys2_msgs::GetDomainConstants srv;
  get_constants_client_.call(srv);
  
  return srv.result.constants;

}

std::vector<plansys2::Predicate> DomainExpertClient::getPredicates()
{
  std::vector<plansys2::Predicate> ret;

  while (!get_predicates_client_.waitForExistence(ros::Duration(1.0)))
    ROS_INFO_STREAM(getNodeName() << " Waiting for service " << get_predicates_client_.getService() << " to appear");    

  plansys2_msgs::GetStates srv;
  get_predicates_client_.call(srv);

  ret = plansys2::convertVector<plansys2::Predicate, plansys2_msgs::msg::Node>(srv.result.states);

  return ret;
}

std::optional<plansys2::Predicate> DomainExpertClient::getPredicate(const std::string & predicate)
{
  while (!get_predicate_details_client_.waitForExistence(ros::Duration(1.0)))
    ROS_INFO_STREAM(getNodeName() << " Waiting for service " << get_predicate_details_client_.getService() << " to appear");    

  plansys2_msgs::GetNodeDetails srv;

  srv.request.expression = predicate;

  if(!get_predicate_details_client_.call(srv))
    return {};

  if (srv.result.success) {
    return srv.result.node;
  } else {
    ROS_ERROR_STREAM(getNodeName() << " " <<
		     get_predicate_details_client_.getService() << ": " <<
		     srv.result.error_info);
    return {};
  }
  
  return {};
}

std::vector<plansys2::Function> DomainExpertClient::getFunctions()
{  
  std::vector<plansys2::Function> ret;

  while (!get_functions_client_.waitForExistence(ros::Duration(1.0)))
    ROS_INFO_STREAM(getNodeName() << " Waiting for service " << get_functions_client_.getService() << " to appear");    
 
  plansys2_msgs::GetStates srv;
  get_functions_client_.call(srv);

  ret = plansys2::convertVector<plansys2::Function, plansys2_msgs::msg::Node>(
    srv.result.states);

  return ret;
}

std::optional<plansys2::Function> DomainExpertClient::getFunction(const std::string & function)
{
  while (!get_function_details_client_.waitForExistence(ros::Duration(1.0)))
    ROS_INFO_STREAM(getNodeName() << " Waiting for service " << get_function_details_client_.getService() << " to appear");    

  
  plansys2_msgs::GetNodeDetails srv;
  srv.request.expression = function;

  if(!get_function_details_client_.call(srv))
  {
    return {};
  }

  if (srv.result.success) {
    return srv.result.node;
  } else {
    ROS_ERROR_STREAM(getNodeName() << " " <<
		     get_function_details_client_.getService() << ": " <<
		     srv.result.error_info);
    return {};
  }
  return {};
}

std::vector<std::string> DomainExpertClient::getActions()
{
  std::vector<std::string> ret;

  while (!get_actions_client_->wait_for_service(std::chrono::seconds(1))) {
    if (!rclcpp::ok()) {
      return ret;
    }
    RCLCPP_ERROR_STREAM(
      node_->get_logger(),
      get_actions_client_->get_service_name() <<
        " service client: waiting for service to appear...");
  }

  auto request = std::make_shared<plansys2_msgs::GetDomainActions::Request>();

  auto future_result = get_actions_client_->async_send_request(request);

  if (rclcpp::spin_until_future_complete(node_, future_result, std::chrono::seconds(1)) !=
    rclcpp::FutureReturnCode::SUCCESS)
  {
    return ret;
  }

  auto result = *future_result.get();

  for (size_t i = 0; i < result.actions.size(); i++) {
    ret.push_back(result.actions[i]);
  }

  return ret;
}

plansys2_msgs::msg::Action::SharedPtr
DomainExpertClient::getAction(
  const std::string & action,
  const std::vector<std::string> & params)
{
  while (!get_action_details_client_->wait_for_service(std::chrono::seconds(1))) {
    if (!rclcpp::ok()) {
      return {};
    }
    RCLCPP_ERROR_STREAM(
      node_->get_logger(),
      get_action_details_client_->get_service_name() <<
        " service client: waiting for service to appear...");
  }

  auto request = std::make_shared<plansys2_msgs::GetDomainActionDetails::Request>();

  request->action = action;
  request->parameters = params;

  auto future_result = get_action_details_client_->async_send_request(request);

  if (rclcpp::spin_until_future_complete(node_, future_result, std::chrono::seconds(1)) !=
    rclcpp::FutureReturnCode::SUCCESS)
  {
    return {};
  }

  auto result = *future_result.get();


  if (result.success) {
    return std::make_shared<plansys2_msgs::msg::Action>(result.action);
  } else {
    RCLCPP_ERROR_STREAM(
      node_->get_logger(),
      get_action_details_client_->get_service_name() << ": " <<
        result.error_info);
    return {};
  }
}

std::vector<std::string>
DomainExpertClient::getDurativeActions()
{
  std::vector<std::string> ret;

  while (!get_durative_actions_client_->wait_for_service(std::chrono::seconds(1))) {
    if (!rclcpp::ok()) {
      return ret;
    }
    RCLCPP_ERROR_STREAM(
      node_->get_logger(),
      get_durative_actions_client_->get_service_name() <<
        " service client: waiting for service to appear...");
  }

  auto request = std::make_shared<plansys2_msgs::GetDomainActions::Request>();

  auto future_result = get_durative_actions_client_->async_send_request(request);

  if (rclcpp::spin_until_future_complete(node_, future_result, std::chrono::seconds(1)) !=
    rclcpp::FutureReturnCode::SUCCESS)
  {
    return ret;
  }

  auto result = *future_result.get();

  for (size_t i = 0; i < result.actions.size(); i++) {
    ret.push_back(result.actions[i]);
  }

  return ret;
}

plansys2_msgs::msg::DurativeAction::SharedPtr
DomainExpertClient::getDurativeAction(
  const std::string & action,
  const std::vector<std::string> & params)
{
  while (!get_durative_action_details_client_->wait_for_service(std::chrono::seconds(1))) {
    if (!rclcpp::ok()) {
      return nullptr;
    }
    RCLCPP_ERROR_STREAM(
      node_->get_logger(),
      get_durative_action_details_client_->get_service_name() <<
        " service client: waiting for service to appear...");
  }

  auto request = std::make_shared<plansys2_msgs::GetDomainDurativeActionDetails::Request>();

  request->durative_action = action;
  request->parameters = params;

  auto future_result = get_durative_action_details_client_->async_send_request(request);

  if (rclcpp::spin_until_future_complete(node_, future_result, std::chrono::seconds(1)) !=
    rclcpp::FutureReturnCode::SUCCESS)
  {
    return nullptr;
  }

  auto result = *future_result.get();

  if (result.success) {
    return std::make_shared<plansys2_msgs::msg::DurativeAction>(
      result.durative_action);
  } else {
    RCLCPP_ERROR_STREAM(
      node_->get_logger(),
      get_durative_action_details_client_->get_service_name() << ": " <<
        result.error_info);
    return nullptr;
  }
}

std::string
DomainExpertClient::getDomain()
{
  std::string ret;

  while (!get_domain_client_->wait_for_service(std::chrono::seconds(1))) {
    if (!rclcpp::ok()) {
      return ret;
    }
    RCLCPP_ERROR_STREAM(
      node_->get_logger(),
      get_domain_client_->get_service_name() <<
        " service client: waiting for service to appear...");
  }

  auto request = std::make_shared<plansys2_msgs::GetDomain::Request>();

  auto future_result = get_domain_client_->async_send_request(request);

  if (rclcpp::spin_until_future_complete(node_, future_result, std::chrono::seconds(1)) !=
    rclcpp::FutureReturnCode::SUCCESS)
  {
    return ret;
  }

  auto result = *future_result.get();

  ret = result.domain;

  return ret;
}
}  // namespace plansys2
