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

#include <memory>

#include <plansys2_domain_expert/DomainExpertNode.hpp>
#include <ros/ros.h>

int main(int argc, char ** argv)
{
  ros::init(argc, argv, "domain_expert_node");
  ros::NodeHandle nh("domain_expert");
  std::shared_ptr<plansys2::DomainExpertNode> den; 
  den.reset( new plansys2::DomainExpertNode(nh) );

  ros::MultiThreadedSpinner spinner(4);
  spinner.spin();

  return 0;
}
