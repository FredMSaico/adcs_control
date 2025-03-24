# ADCS Control Package for CubeSat Attitude Control

This ROS 2 package provides attitude control algorithms for a CubeSat prototype. The package includes multiple control strategies such as PID, Boskovic, and Feedback control. It also provides utility nodes for setpoint and PWM signal publishing, as well as a graphical user interface (GUI) for visualization and interaction.

## Installation

1. Clone the repository:
   ```bash
   git clone https://github.com/FredMSaico/adcs_control.git
   ```
2. Navigate to your ROS 2 workspace and place the package inside the `src` folder:
   ```bash
   cd ~/ros2_ws/src
   mv ~/Downloads/adcs_control .
   ```
3. Build the package:
   ```bash
   cd ~/ros2_ws
   colcon build --packages-select adcs_control
   ```
4. Source the workspace:
   ```bash
   source install/setup.bash
   ```

## Nodes

The package provides the following ROS 2 nodes:

### Controllers
- **PID Controller**: Implements a PID control strategy.
  ```bash
  ros2 run adcs_control pid_controller
  ```
- **Boskovic Controller**: Implements the Boskovic adaptive control method.
  ```bash
  ros2 run adcs_control bosk_controller
  ```
- **Feedback Controller**: Implements a state-feedback control approach.
  ```bash
  ros2 run adcs_control feedback_controller
  ```

## Author
Alfredo Mamani Saico
