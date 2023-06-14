#ifndef ADD_BLOOG_H
#define ADD_BLOOD_H

#include "behavior_tree.h"
namespace sp_decision {
class AddBloodBehavior : public ActionNode {
 public:
  AddBloodBehavior(std::string name, int level,
                   const Blackboard::Ptr &blackboard_ptr,
                   ChassisExecutor::Ptr &chassis_exe_ptr,
                   LogExecutor::Ptr &log_exe_ptr)
      : ActionNode::ActionNode(name, level, blackboard_ptr, chassis_exe_ptr,log_exe_ptr) {}
  BehaviorState Update() {
    if (blackboard_ptr_->robot_hp_ <= blackboard_ptr_->min_hp_) {
      Go2Buff();
      log_exe_ptr_->info("behavior: add blood");
      return BehaviorState::SUCCESS;
    }
    return BehaviorState::FAILURE;
  }

 private:
  void Go2Buff() {
    chassis_exe_ptr_->FastMove(blackboard_ptr_->buff_pos_[1].x,
                               blackboard_ptr_->buff_pos_[1].y);
  }
};
}  // namespace robot_decision

#endif