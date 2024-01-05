#include "decision_node.h"

namespace sp_decision {
DecisionNode::DecisionNode() {
  nh_.param("loop_rate", loop_rate_, 5);

  blackboard_ = std::make_shared<Blackboard>();
  log_exe_ = std::make_shared<LogExecutor>();
  chassis_exe_ = std::make_shared<ChassisExecutor>();
  root_node_ = new SequenceNode("robot_decision", 0, blackboard_, chassis_exe_,
                                log_exe_);
  AddBloodBehavior* add_blood_node_ =
      new AddBloodBehavior("add blood", 1, blackboard_, chassis_exe_, log_exe_);
  RetreatBehavior* retreat_node_ =
      new RetreatBehavior("add retreat", 2, blackboard_, chassis_exe_, log_exe_);

  root_node_->addChild(add_blood_node_);
  root_node_->addChild(retreat_node_);
  std::stringstream str;
  str << std::endl << "***********************" << std::endl;
  str << "*Super Power Decision*" << std::endl;
  str << "***********************";
  log_exe_->info(str);
  root_node_->print_tree();
  str.str("");
  str <<  "*********START*********";
  log_exe_->info(str);
  decision_thread_ = std::thread(&DecisionNode::ExecuteLoop, this);
  decision_thread_running_ = true;
}

void DecisionNode::ExecuteLoop() {
  ros::Rate loop_rate(loop_rate_);
  int cnt = 0;
  while (decision_thread_running_) {
    root_node_->Run();
  }
  cnt++;
  if (cnt >= loop_rate_) {
    cnt = 0;
    blackboard_->ResetFlag();
  }
  std::cout << "a" ;
  loop_rate.sleep();
}

}  // namespace sp_decision

int main(int argc, char** argv)
{
    ros::init(argc, argv, "sp_decision_node");
    sp_decision::DecisionNode decision;
    ros::spin();
    return 0;
}