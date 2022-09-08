<p align="center">
  <a href="" rel="noopener">
 <img width=478px height=164px src="https://cdn.shopify.com/s/files/1/0055/0433/5925/files/rover_logo_1.png?v=1625525167" alt="Rover Logo"></a>
</p>

<h3 align="center">librover</h3>

<div align="center">
<p>The following diagram shows how librover is used in the broader Rover Robotics Software stack. Most users won't need to work directly with this library and can instead use the ROS1 and ROS2 drivers which expose its functionalities. <b>DISCLAIMER: This is a dev fork for adding ros2 functionality to the Rover Mini and Rover Mega. </p>
<img width=478px height=200px src="https://raw.githubusercontent.com/RoverRobotics/librover/release/docs/librover_stack_diagram.PNG" alt="librover Stack Diagram"></a>

</div>

---

## 📝 Table of Contents

- [About](#about)
- [Installation](#installation)
- [Usage](#usage)
- [Authors](#authors)

<!-- - [Deployment](#deployment) -->

## 🧐 About <a name = "about"></a>

C++ Library for communicating with a Rover Robotics Rover Pro or Rover Zero 2.

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

The main purpose of this library is to be a dependancy for our ROS1 and ROS2 drivers, but it can also be used if you are a hardcore C++ programmer, want to use ISAAC SDK, or if you want to create a C++ based GUI.

Below is an example on how to use this library in a C++ project
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
