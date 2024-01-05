#ifndef BEHAVIOR_H
#define BEHAVIOR_H

#include "blackboard.hpp"
#include "executor/chassis_executor.hpp"
#include "executor/log_executor.hpp"

namespace sp_decision {
enum class BehaviorState { SUCCESS, FAILURE,RUNNING };
class TreeNode {
 public:
  TreeNode() {}
  TreeNode(std::string name, int level, const Blackboard::Ptr &blackboard_ptr,
           ChassisExecutor::Ptr &chassis_exe_ptr, LogExecutor::Ptr &log_exe_ptr)
      : name_(name),
        level_(level),
        blackboard_ptr_(blackboard_ptr),
        chassis_exe_ptr_(chassis_exe_ptr),
        log_exe_ptr_(log_exe_ptr) {}
  virtual ~TreeNode() = default;
  virtual BehaviorState Update() = 0;
  virtual void print() = 0;
  virtual void print_tree() = 0;
  BehaviorState Run() {
    behavior_state_ = Update();
    return behavior_state_;
  }
  Blackboard::Ptr blackboard_ptr_;
  ChassisExecutor::Ptr chassis_exe_ptr_;
  LogExecutor::Ptr log_exe_ptr_;
  int level_;
  std::string name_;
  BehaviorState behavior_state_;
};
class ActionNode : public TreeNode {
 public:
  ActionNode() : TreeNode::TreeNode() {}
  ActionNode(std::string name, int level, const Blackboard::Ptr &blackboard_ptr,
             ChassisExecutor::Ptr &chassis_exe_ptr,
             LogExecutor::Ptr &log_exe_ptr)
      : TreeNode::TreeNode(name, level, blackboard_ptr, chassis_exe_ptr,
                           log_exe_ptr) {}
  virtual ~ActionNode() = default;
  void print() {
    std::stringstream str;
    for (int i = 0; i < level_; i++) {
      str << "  ";
    }
    str << "- " << name_.data();
    log_exe_ptr_->info(str);
  }
  void print_tree() { print(); }
};
class SequenceNode : public TreeNode {
 public:
  SequenceNode() : TreeNode::TreeNode() {}
  SequenceNode(std::string name, int level,
               const Blackboard::Ptr &blackboard_ptr,
               ChassisExecutor::Ptr &chassis_exe_ptr,
               LogExecutor::Ptr &log_exe_ptr)
      : TreeNode::TreeNode(name, level, blackboard_ptr, chassis_exe_ptr,
                           log_exe_ptr) {}
  virtual ~SequenceNode() = default;
  void addChild(TreeNode *child_node_ptr) {
    child_node_ptr_list_.push_back(child_node_ptr);
  }
  virtual BehaviorState Update() {
    if (child_node_ptr_list_.size() == 0) {
      return BehaviorState::FAILURE;
    } else {
      for (int i = 0; i < child_node_ptr_list_.size(); i++) {
        if (child_node_ptr_list_[i]->Run() == BehaviorState::SUCCESS) {
          return BehaviorState::SUCCESS;
        }
        // else if(child_node_ptr_list_[i]->Run() == BehaviorState::RUNNING){
        //   return BehaviorState::SUCCESS;
        // }
      }
      return BehaviorState::FAILURE;
    }
  }
  void print() {
    std::stringstream str;
    for (int i = 0; i < level_; i++) {
      str << "  ";
    }
    str << "+ " << name_.data();
    log_exe_ptr_->info(str);
  }
  void print_tree() {
    print();
    for (int i = 0; i < child_node_ptr_list_.size(); i++) {
      child_node_ptr_list_[i]->print_tree();
    }
  }

 private:
  std::vector<TreeNode *> child_node_ptr_list_;
};

}  // namespace sp_decision

#endif