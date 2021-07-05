<p align="center">
  <a href="" rel="noopener">
 <img width=206px height=68px src="https://cdn.shopify.com/s/files/1/0055/0433/5925/files/mark_and_word_black_68_206.png?v=1582592595" alt="Rover Logo"></a>
</p>

<h3 align="center">librover</h3>

<div align="center">


</div>

---

<p align="center"> C++ Library for communicating with all Rover Robotics Products including Rover Pro 1, Rover Zero v2, Rover Pro 2, and Rover Mini.
    <br> 
</p>

## 📝 Table of Contents

- [About](#about)
- [Installation](#installation)
- [Usage](#usage)
- [Authors](#authors)

<!-- - [Deployment](#deployment) -->

## 🧐 About <a name = "about"></a>

C++ Library for communicating with all Rover Robotics Products including Rover Pro 1, Rover Zero v2, Rover Pro 2, and Rover Mini.

## 🏁 Installation <a name = "installation"></a>

These instructions will get you a copy of the project up and running on your local machine for development and testing purposes. 

```
git clone https://github.com/RoverRobotics/librover.git
cd librover
cmake .
make
sudo make install
```

## 🎈 Usage <a name="usage"></a>

We are currently support both Ros1 and Ros2 using this library. 
You can also use it as a part of your other Frameworks by including this library to your CMAKE files for C++ projects.

This is an example on how to use this library in a c++ project
```
#include <librover/protocol_pro> //include robot specific library

int main(int argc, char *argv[]){
  //Initialize robot parameters
  Control::pid_gains testgains_ = {0, 0, 0};

  Control::robot_motion_mode_t robot_mode = Control::INDEPENDENT_WHEEL;
  Control::angular_scaling_params angular_scaling_params_ = {0, 1, 0, 1, 1};
  //Create a robot object with set parameters
  std::unique_ptr<BaseProtocolObject> robot_ =
      std::make_unique<Zero2ProtocolObject>("/dev/rover-zero-v2", "serial",
                                            robot_mode, testgains_,
                                            angular_scaling_params_);
  //Robot Loop  
  while (true) {
    //request robot status
    auto status = robot_->status_request();
    print_status(status);
    //set robot velocities
    robot_->set_robot_velocity({1,0});
    auto status = robot_->status_request();
    // robot_->cycle_robot_mode();

    // robot_->send_estop(true);

    // robot_->update_drivetrim(0.01);
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
  }
```





<!-- ## 🚀 Deployment <a name = "deployment"></a>

Add additional notes about how to deploy this on a live system. -->

## ✍️ Authors <a name = "authors"></a>

- [@william_rook](https://github.com/drhieu) - Main Maintenaner
- [@roverrobotics](https://github.com/roverrobotics)
