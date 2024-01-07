

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
 void attack_outpost(){
    chassis_exe_ptr_->FastMove(blackboard_ptr_->attack_pos[0].x,
                               blackboard_ptr_->attack_pos[0].y);
    chassis_exe_ptr_->vel_msg_pub_.linear.x = blackboard_ptr_->vel_msg_sub_.linear.x;
    chassis_exe_ptr_->vel_msg_pub_.linear.y = blackboard_ptr_->vel_msg_sub_.linear.y;
    chassis_exe_ptr_->vel_msg_pub_.angular.z = 0.0;

 }

};
}  // namespace robot_decision

#endif