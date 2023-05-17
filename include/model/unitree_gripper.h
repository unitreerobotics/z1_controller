#ifndef _UNITREE_GRIPPER_H
#define _UNITREE_GRIPPER_H

#include <algorithm>
#include <cmath>
#include <chrono>
#include "common/math/mathTools.h"

typedef struct
{
  double angle;                      	// Current gripper opening angle. Unit: rad
  double speed;												// Current measured gripper speed. Unit: rad/s
  double tau;													// Current measured gripper output torque. Unit: Nm
  bool   is_grasped;									// Indicates whether an object is currently grasped.
  int8_t temperature;                	// current temperature (temperature conduction is slow that leads to lag)
}GripperState;

typedef struct
{
  double angle;                       // Size of object to grasp in radian               
  double speed;                       // Closing speed in rad/s          
  double tau;                         // Grasping tau in Nm	
  double epsilon_inner;               // Maximum tolerated deviation when the actual grasped angle is smaller than the commanded grasp angle.
  double epsilon_outer;               // Maximum tolerated deviation when the actual grasped angle is larger than the commanded grasp angle.
}GripperCmd;

class Unitree_Gripper
{
public:
  Unitree_Gripper();
  Unitree_Gripper(double max_toque);
  ~Unitree_Gripper() = default;

  /**
   * @brief Move the gripper to target angle
   * 
   * @param angle Target opening angle in rad
   * @param speed Opening speed in rad/s
   */
  void move(double angle, double speed);

  /**
   * @brief Grasps an object
   * 
   * @param angle Size of object to grasp in radian.
   * @param speed Closing speed in rad/s.
   * @param tau Grasping torque in Nm
   * @param epsilon_inner Maximum tolerated deviation when the actual grasped angle 
   * is smaller than the commanded grasp angle.
   * @param epsilon_outer Maximum tolerated deviation when the actual grasped angle 
   * is larger than the commanded grasp angle.
   * @return True if an object has been grasped.
   */
  void grasp( double angle,
              double speed,
              double tau,
              double epsilon_inner = 0.01,
              double epsilon_outer = 0.01);
  
  /**
   * @brief Get current output torque due to current state
   * 
   * @param angle Measured gripper angle
   * @param speed Measured gripper speed
   * @param torque Measured gripper torque
   * @param temperature Measured gripper temperature
   * @return double Output torque
   */
  double update(double angle, double speed, double torque, int8_t temperature = 0);

  GripperCmd cmd{};
  GripperState state{};
private:
  enum class GripperCtrlMode{
    Move,// Position
    Grasp// Torque
  } mode = GripperCtrlMode::Move;

  struct
  {
    double kp_p;
    double kd_p;
    double kp_v;
    double ki_v;
    double kd_v;
  } coe{};
  
  typedef struct
  {
    double angle;
    double speed;
  } Error_state;

  Error_state error{}, error_last{};
  double error_speed_sum_{};

  static constexpr double MAX_POSITION = -M_PI/2;
  static constexpr double MAX_SPEED = M_PI;
  double MAX_TORQUE = 20.0;

  std::chrono::steady_clock::time_point first_grasped_time_{};
  bool is_stopped_{};
  double tau_output{};

  void modify_cmd(GripperCmd& gripper_cmd);
  bool is_grasped();
};

#endif // _UNITREE_GRIPPER_H