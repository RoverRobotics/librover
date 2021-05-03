#pragma once

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <fstream>
#include <iostream>
#include <string>
#include <vector>
namespace Utilities {
/* classes */
class ParamsUtil;
}  // namespace Utilities

class Utilities::ParamsUtil {
 private:
  std::string robot_config_path_;
  std::string temp_robot_config_path_;
  std::ifstream file_reader_;
  std::ofstream file_writer_;
  std::vector<std::string> split(std::string str, std::string token);

 public:
  ParamsUtil(std::string robot_config_path);
  void write_params(std::string replacing_key, std::string replacing_value);
  std::vector<std::vector<std::string>> get_params();
};