#include <fake_info_publisher.h>
#include<sstream>
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
void FakeInfoPublisher::UserSetStatus(int match_state, int robot_hp, int robot_bullet, int outpost_hp){
  match_msg_.match_state = match_state;
  match_msg_.robot_hp = robot_hp;
  match_msg_.robot_bullet = robot_bullet;
  match_msg_.outpost_hp = outpost_hp;
}



std::string userinput;
int userorder[4];
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
    case 4:
    {
      std::cout<<"依次键入数据 : 比赛状态 血量 弹丸 前哨战"<<std::endl;

      std::getline(std::cin, userinput);
      std::istringstream userstr(userinput);
      // 从 std::istringstream 中提取整数，并存储在 vector 中
      for(int j=0;j<4;j++)
      {
        userstr >> userorder[j];
        std::cout<<userorder[j]<<std::endl;
      }
      // while (iss >> number) {
      //   numbers.push_back(number);
      // }

      UserSetStatus(userorder[0],userorder[1],userorder[2],userorder[3]);
      break;
    }
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