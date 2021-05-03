#include "utilities.hpp"
namespace Utilities {

ParamsUtil::ParamsUtil(std::string robot_config_path) {
  robot_config_path_ = robot_config_path;
  temp_robot_config_path_ = robot_config_path + ".tmp";
}

void ParamsUtil::write_params(std::string replacing_key,
                              std::string replacing_value) {
  // open previous config file;
  std::string const HOME = std::getenv("HOME") ? std::getenv("HOME") : ".";

  file_reader_.open(robot_config_path_);
  file_writer_.open(temp_robot_config_path_);
  if (file_reader_.is_open()) {
    std::string line;
    while (std::getline(file_reader_, line)) {
      std::vector<std::string> key =
          split(line, ":"); /* get key-value pair into vector */
      std::vector<std::string> values =
          split(key[1], " ");        /* get all the values from this key */
      if (key[0] == replacing_key) { /* check for key */
        file_writer_ << replacing_key << ":" << replacing_value
                     << std::endl; /* output modified value instead */
      } else
        file_writer_ << line; /* output original value */
    }
  }
  file_reader_.close();
  file_writer_.close();
  int n = robot_config_path_.length();
  char robotconfig_path[n + 1];
  strcpy(robotconfig_path, robot_config_path_.c_str());
  n = temp_robot_config_path_.length();
  char newrobotconfig_path[n + 1];
  strcpy(newrobotconfig_path, temp_robot_config_path_.c_str());
  if (rename(newrobotconfig_path, robotconfig_path) == 0)
    puts("Config File Updated");
  else
    perror("Error Saving config file");
}
std::vector<std::string> ParamsUtil::split(std::string str, std::string token) {
  std::vector<std::string> result;
  while (str.size()) {
    int index = str.find(token);
    if (index != std::string::npos) {
      result.push_back(str.substr(0, index));
      str = str.substr(index + token.size());
      if (str.size() == 0) result.push_back(str);
    } else {
      result.push_back(str);
      str = "";
    }
  }
  return result;
}

std::vector<std::vector<std::string>> ParamsUtil::get_params() {
  // open yaml
  std::vector<std::vector<std::string>> params;
  file_reader_.open(robot_config_path_);
  if (file_reader_.is_open()) {
    std::string line;
    int line_position = -1;
    while (std::getline(file_reader_, line)) {
      line_position++;
      std::vector<std::string> key =
          split(line, ":");  // get key-value pair into vector
      std::vector<std::string> values =
          split(key[1], " ");            // get all the values from this key
      params[line_position][0] = key[0]; /* get key name */
      params[line_position].insert(params[line_position].end(), values.begin(),
                                   values.end());
    }
    file_reader_.close();
    return params;
  } else {
    std::cerr << "Failed to load config from " + robot_config_path_
              << std::endl;
    std::cerr << "Making a default config at " + robot_config_path_
              << std::endl;
    std::ofstream file_writer_;
    file_writer_.open(robot_config_path_);
    file_writer_ << "trim:0" << std::endl;
    file_writer_.close();
    params[0][0] = "trim";
    params[0][1] = "0";
  }
  return params;
}
}  // namespace Utilities