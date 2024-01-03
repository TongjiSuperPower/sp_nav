#ifndef FAKE_INFO_H
#define FAKE_INFO_H
#include <robot_msg/MatchMsg.h>
#include <ros/ros.h>
#include <geometry_msgs/Point.h>

#include <iostream>
#include <random>
#include <string>

class FakeInfoPublisher {
 public:
  FakeInfoPublisher() {
    fake_info_publihser_ =
        nh_.advertise<robot_msg::MatchMsg>("match_status", 1);
    fake_referee_data_publisher_ = 
        nh_.advertise<geometry_msgs::Point>("referee_data", 1);
    match_msg_.match_state = 0;
    match_msg_.outpost_hp = 500;
    match_msg_.robot_bullet = 750;
    match_msg_.robot_hp = 600;

    lower_referee_data_.x = 0.0;
    lower_referee_data_.y = 0.0;
    lower_referee_data_.z = 0.0;
  }
  void GameStart();
  void Attacked();
  void LackBullet();
  void UserSetStatus(int match_state, int robot_hp, int robot_bullet, int outpost_hp);
  void SwitchScenarios(int id);

 private:
  ros::NodeHandle nh_;
  ros::Publisher fake_info_publihser_;
  ros::Publisher fake_referee_data_publisher_;
  robot_msg::MatchMsg match_msg_;
  geometry_msgs::Point lower_referee_data_;
  void FakeInfoPub();
  void FakeRefereeDataPub();
  struct RandomNumberGenerator {
   public:
    RandomNumberGenerator() : generator_(std::random_device()()) {}

    int generateRandomInt(int min, int max) {
      std::uniform_int_distribution<int> distribution(min, max);
      return distribution(generator_);
    }
   private:
    std::mt19937 generator_;
  };
  RandomNumberGenerator rng_;
};

#endif