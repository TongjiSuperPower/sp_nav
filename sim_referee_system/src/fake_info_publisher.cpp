#include <fake_info_publisher.h>

void FakeInfoPublisher::GameStart() { match_msg_.match_state = 1; }
void FakeInfoPublisher::Attacked() {
  match_msg_.match_state = 1;
  match_msg_.robot_hp = rng_.generateRandomInt(0, 200);
  match_msg_.robot_bullet = rng_.generateRandomInt(0, 750);
  match_msg_.outpost_hp = rng_.generateRandomInt(0, 600);
}
void FakeInfoPublisher::LackBullet() {
  match_msg_.match_state = 1;
  match_msg_.robot_hp = rng_.generateRandomInt(0, 400);
  match_msg_.robot_bullet = rng_.generateRandomInt(0, 100);
  match_msg_.outpost_hp = rng_.generateRandomInt(0, 600);
}

void FakeInfoPublisher::SwitchScenarios(int id) {
  switch (id) {
    case 1:
      GameStart();
      break;
    case 2:
      Attacked();
      break;
    case 3:
      LackBullet();
      break;
    default:
      ROS_WARN_STREAM("Scenario error:" << id);
      break;
  }
  // 场景选择错误，信息保留上一次
  FakeInfoPub();
}

void FakeInfoPublisher::FakeInfoPub() {
    fake_info_publihser_.publish(match_msg_);
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "sp_decision_node");
  FakeInfoPublisher fakeinfo;
  std::string input;
  while (ros::ok()) {
    std::cout << "Enter a scenario id (or 'quit' to exit): ";
    std::getline(std::cin, input);
    if (input == "quit") break;
    int id = std::stoi(input);
    fakeinfo.SwitchScenarios(id);
  }
  return 0;
}