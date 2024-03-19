# uav_offboard Package Usage
## Brief Introduction
This package is responsible for generating trajectories based on messages from "/mavros/rc/in" topic. In other words, if you have a remote controller at hand, which in our case is Futaba 14SG, you can generate 4DOF trajectories as you move the joysticks. And the signals from switches can also be used for mode, swtiching, arm, kill or other functions.
## File Explanations

## How to use
### Start Mavros connect
Connect the Pixhawk hardware(with the rc receiver) to the computer, open the remote controller, and start the mavros connection.

If it is for generating the trajectory only, navigate to `uav_offboard/resource/rc_configuration.yaml`, and set the parameter `need_offboard` to `false`.
Navigate to the workspace, open a terminal, and enter the commands below:
```bash
source ./devel/setup.bash
roslaunch uav_offboard rc_command_processing.launch
```
The trajectory points will be published to the topic "/command/trajectory".

If it is for generating the trajectory in offboard control mode, you should connect the battery and go outdoors, be careful about the rotors. Navigate to `uav_offboard/resource/rc_configuration.yaml`, and set the parameter `need_offboard` to `true`.
```bash
source ./devel/setup.bash
roslaunch uav_offboard rc_command_processing.launch
```
