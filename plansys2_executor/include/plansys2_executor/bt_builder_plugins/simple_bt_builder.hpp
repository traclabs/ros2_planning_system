// Copyright 2020 Intelligent Robotics Lab
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

#ifndef PLANSYS2_EXECUTOR__BT_BUILDER_PLUGINS__SIMPLE_BT_BUILDER_HPP_
#define PLANSYS2_EXECUTOR__BT_BUILDER_PLUGINS__SIMPLE_BT_BUILDER_HPP_

#include <string>
#include <memory>
#include <vector>
#include <set>
#include <list>
#include <map>
#include <utility>
#include <tuple>

#include <std_msgs/Empty.h>

#include <plansys2_domain_expert/DomainExpertClient.hpp>
#include <plansys2_problem_expert/ProblemExpertClient.hpp>
#include <plansys2_executor/ActionExecutor.hpp>
#include <plansys2_executor/BTBuilder.hpp>
#include <plansys2_core/Types.hpp>
#include <plansys2_msgs/DurativeAction.h>
#include <plansys2_msgs/Plan.h>

#include <ros/ros.h>
//#include "rclcpp_action/rclcpp_action.hpp"

namespace plansys2
{

struct ActionNode
{
  using Ptr = std::shared_ptr<ActionNode>;
  static Ptr make_shared() {return std::make_shared<ActionNode>();}

  ActionStamped action;
  int node_num;
  int level_num;

  std::vector<plansys2::Predicate> predicates;
  std::vector<plansys2::Function> functions;

  std::list<ActionNode::Ptr> in_arcs;
  std::list<ActionNode::Ptr> out_arcs;
};

struct ActionGraph
{
  using Ptr = std::shared_ptr<ActionGraph>;
  static Ptr make_shared() {return std::make_shared<ActionGraph>();}

  std::list<ActionNode::Ptr> roots;
  std::map<float, std::list<ActionNode::Ptr>> levels;
};

class SimpleBTBuilder : public BTBuilder
{
public:
  SimpleBTBuilder();
  void initialize(
    const std::string & bt_action_1 = "",
    const std::string & bt_action_2 = "",
    int precision = 3);

<<<<<<< HEAD
  std::string get_tree(const plansys2_msgs::Plan & current_plan);
=======
  std::string get_tree(const plansys2_msgs::msg::Plan & current_plan);
  Graph::Ptr get_graph() {return nullptr;}
  bool propagate(Graph::Ptr) {return true;}
>>>>>>> rolling
  std::string get_dotgraph(
    std::shared_ptr<std::map<std::string, ActionExecutionInfo>> action_map,
    bool enable_legend = false,
    bool enable_print_graph = false);

protected:
  std::shared_ptr<plansys2::DomainExpertClient> domain_client_;
  std::shared_ptr<plansys2::ProblemExpertClient> problem_client_;

  ActionGraph::Ptr graph_;
  std::string bt_;
  std::string bt_action_;

<<<<<<< HEAD
  Graph::Ptr get_graph(const plansys2_msgs::Plan & current_plan);

  std::vector<ActionStamped> get_plan_actions(const plansys2_msgs::Plan & plan);
  void prune_backwards(GraphNode::Ptr new_node, GraphNode::Ptr node_satisfy);
  void prune_forward(GraphNode::Ptr current, std::list<GraphNode::Ptr> & used_nodes);
=======
  ActionGraph::Ptr get_graph(const plansys2_msgs::msg::Plan & current_plan);

  std::vector<ActionStamped> get_plan_actions(const plansys2_msgs::msg::Plan & plan);
  void prune_backwards(ActionNode::Ptr new_node, ActionNode::Ptr node_satisfy);
  void prune_forward(ActionNode::Ptr current, std::list<ActionNode::Ptr> & used_nodes);
>>>>>>> rolling
  void get_state(
    const ActionNode::Ptr & node,
    std::list<ActionNode::Ptr> & used_nodes,
    std::vector<plansys2::Predicate> & predicates,
    std::vector<plansys2::Function> & functions) const;

  bool is_action_executable(
    const ActionStamped & action,
    std::vector<plansys2::Predicate> & predicates,
    std::vector<plansys2::Function> & functions) const;
  std::list<ActionNode::Ptr> get_roots(
    std::vector<plansys2::ActionStamped> & action_sequence,
    std::vector<plansys2::Predicate> & predicates,
    std::vector<plansys2::Function> & functions,
    int & node_counter);
<<<<<<< HEAD
  GraphNode::Ptr get_node_satisfy(
    const plansys2_msgs::Tree & requirement,
    const Graph::Ptr & graph,
    const GraphNode::Ptr & current);
  GraphNode::Ptr get_node_satisfy(
    const plansys2_msgs::Tree & requirement,
    const GraphNode::Ptr & node,
    const GraphNode::Ptr & current);
  std::list<GraphNode::Ptr> get_node_contradict(
    const Graph::Ptr & graph,
    const GraphNode::Ptr & current);
=======
  ActionNode::Ptr get_node_satisfy(
    const plansys2_msgs::msg::Tree & requirement,
    const ActionGraph::Ptr & graph,
    const ActionNode::Ptr & current);
  ActionNode::Ptr get_node_satisfy(
    const plansys2_msgs::msg::Tree & requirement,
    const ActionNode::Ptr & node,
    const ActionNode::Ptr & current);
  std::list<ActionNode::Ptr> get_node_contradict(
    const ActionGraph::Ptr & graph,
    const ActionNode::Ptr & current);
>>>>>>> rolling
  void get_node_contradict(
    const ActionNode::Ptr & node,
    const ActionNode::Ptr & current,
    std::list<ActionNode::Ptr> & parents);
  void remove_existing_requirements(
    std::vector<plansys2_msgs::Tree> & requirements,
    std::vector<plansys2::Predicate> & predicates,
    std::vector<plansys2::Function> & functions) const;
  bool is_parallelizable(
    const plansys2::ActionStamped & action,
    const std::vector<plansys2::Predicate> & predicates,
    const std::vector<plansys2::Function> & functions,
    const std::list<ActionNode::Ptr> & ret) const;

  std::string get_flow_tree(
    ActionNode::Ptr node,
    std::list<std::string> & used_nodes,
    int level = 0);
  void get_flow_dotgraph(ActionNode::Ptr node, std::set<std::string> & edges);
  std::string get_node_dotgraph(
    ActionNode::Ptr node, std::shared_ptr<std::map<std::string,
    ActionExecutionInfo>> action_map, int level = 0);
  ActionExecutor::Status get_action_status(
    ActionStamped action,
    std::shared_ptr<std::map<std::string, ActionExecutionInfo>> action_map);
  void addDotGraphLegend(
    std::stringstream & ss, int tab_level, int level_counter,
    int node_counter);

  std::string t(int level);

  std::string execution_block(const ActionNode::Ptr & node, int l);
  void print_node(
    const ActionNode::Ptr & node,
    int level,
    std::set<ActionNode::Ptr> & used_nodes) const;

  void print_graph(const plansys2::ActionGraph::Ptr & graph) const;

  void print_node_csv(const ActionNode::Ptr & node, uint32_t root_num) const;
  void print_graph_csv(const plansys2::ActionGraph::Ptr & graph) const;

  void get_node_tabular(
    const plansys2::ActionNode::Ptr & node,
    uint32_t root_num,
    std::vector<std::tuple<uint32_t, uint32_t, uint32_t, std::string>> & graph) const;
  std::vector<std::tuple<uint32_t, uint32_t, uint32_t, std::string>> get_graph_tabular(
    const plansys2::ActionGraph::Ptr & graph) const;
};

}  // namespace plansys2


#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(plansys2::SimpleBTBuilder, plansys2::BTBuilder)

#endif  // PLANSYS2_EXECUTOR__BT_BUILDER_PLUGINS__SIMPLE_BT_BUILDER_HPP_
