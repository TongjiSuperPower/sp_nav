#ifndef LOG_EXECUTOR_H
#define LOG_EXECUTOR_H

#include <ros/ros.h>
#include <string.h>
#include <sstream>
#ifdef DEBUG_MODE
#define DEBUG_LOG(msg) ROS_INFO_STREAM(msg)
#else
#define DEBUG_LOG(msg)
#endif

class LogExecutor {
 public:
  typedef std::shared_ptr<LogExecutor> Ptr;
  LogExecutor(){};
  ~LogExecutor(){};
  void info(std::string commit) { DEBUG_LOG(commit); }
  void info(std::string commit, auto text) { DEBUG_LOG(commit << text); }
  void info(std::stringstream& commit) { DEBUG_LOG(commit.str()); }
};

#endif
