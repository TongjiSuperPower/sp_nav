#ifndef RETREAT_H
#define RETREAT_H

#include "behavior_tree.h"
namespace sp_decision {
class RetreatBehavior : public ActionNode {
 public:
  RetreatBehavior(std::string name, int level,
                  const Blackboard::Ptr &blackboard_ptr,
                  ChassisExecutor::Ptr &chassis_exe_ptr,
                  LogExecutor::Ptr &log_exe_ptr)
      : ActionNode::ActionNode(name, level, blackboard_ptr, chassis_exe_ptr,
                               log_exe_ptr) {}
  BehaviorState Update() {
    /**
     * @brief: 哨兵预装弹，无弹回原点自旋
     */
    if (blackboard_ptr_->robot_bullet_ < blackboard_ptr_->min_bullet_ ||
        blackboard_ptr_->outpost_hp_ < blackboard_ptr_->min_outpost_) {
      if (std::hypot(blackboard_ptr_->robot_pose_.pose.pose.position.x -
                         blackboard_ptr_->buff_pos_[0].x,
                     blackboard_ptr_->robot_pose_.pose.pose.position.y -
                         blackboard_ptr_->buff_pos_[0].y) >
          blackboard_ptr_->distance_tolerance_) {
        log_exe_ptr_->info("behavior[retreat]:", "fast move");
        Go2Init();
      } else {
        log_exe_ptr_->info("behavior[retreat]:", "idle");
        chassis_exe_ptr_->Idle();
      }
      return BehaviorState::SUCCESS;
    }
  }

 private:
  void Go2Init() {
    chassis_exe_ptr_->FastMove(blackboard_ptr_->buff_pos_[0].x,
                               blackboard_ptr_->buff_pos_[0].y);
  }
};
}  // namespace sp_decision

#endif