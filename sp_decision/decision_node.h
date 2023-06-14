#ifndef DECISION_NODE_H
#define DECISION_NODE_H

#include <thread>

#include "behaviors/add_blood.hpp"
#include "behaviors/retreat.hpp"

namespace sp_decision {
class DecisionNode {
 public:
  DecisionNode();
  ~DecisionNode() {
    if (decision_thread_.joinable()) {
      decision_thread_running_ = false;
      decision_thread_.join();
    }
    delete root_node_;
  }
  void ExecuteLoop();

 private:
  ros::NodeHandle nh_;
  SequenceNode* root_node_;
  Blackboard::Ptr blackboard_;
  ChassisExecutor::Ptr chassis_exe_;
  LogExecutor::Ptr log_exe_;
  std::thread decision_thread_;
  bool decision_thread_running_;
  int loop_rate_;
};
}  // namespace sp_decision

#endif