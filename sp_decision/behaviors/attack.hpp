

#ifndef ATTACK_H
#define ATTACK_H

#include "behavior_tree.h"

namespace sp_decision {
class AttackBehavior : public ActionNode {
 public:
  AttackBehavior(std::string name, int level,
                   const Blackboard::Ptr &blackboard_ptr,
                   ChassisExecutor::Ptr &chassis_exe_ptr,
                   LogExecutor::Ptr &log_exe_ptr)
      : ActionNode::ActionNode(name, level, blackboard_ptr, chassis_exe_ptr,log_exe_ptr) {}
  BehaviorState Update() {
      log_exe_ptr_->info("behavior: attack");
    return BehaviorState::SUCCESS;
  }

 private:

};
}  // namespace robot_decision

#endif