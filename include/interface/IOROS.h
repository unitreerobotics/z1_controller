#ifdef COMPILE_WITH_SIMULATION

#ifndef IOROS_H
#define IOROS_H

#include <ros/ros.h>
#include "interface/IOInterface.h"
#include "unitree_arm_sdk/keyboard.h"
#include "message/unitree_legged_msgs/MotorCmd.h"
#include "message/unitree_legged_msgs/MotorState.h"

class IOROS : public IOInterface{
public:
    IOROS(CmdPanel *cmdPanel);
    ~IOROS();
    bool sendRecv(const LowlevelCmd *cmd, LowlevelState *state);
    // void setGripper(double position, double force){};
    // void getGripper(double &position, double &force){};
private:
    ros::NodeHandle _nm;
#ifndef UNITREE_GRIPPER
    ros::Subscriber _servo_sub[6];
    ros::Publisher _servo_pub[6];
    unitree_legged_msgs::MotorState _joint_state[6];
    unitree_legged_msgs::MotorCmd _joint_cmd[6];
#endif
#ifdef UNITREE_GRIPPER
    ros::Subscriber _servo_sub[7];
    ros::Publisher _servo_pub[7];
    unitree_legged_msgs::MotorState _joint_state[7];
    unitree_legged_msgs::MotorCmd _joint_cmd[7];
#endif
    void _sendCmd(const LowlevelCmd *cmd);
    void _recvState(LowlevelState *state);
    void _initRecv();
    void _initSend();

    void _joint00Callback(const unitree_legged_msgs::MotorState& msg);
    void _joint01Callback(const unitree_legged_msgs::MotorState& msg);
    void _joint02Callback(const unitree_legged_msgs::MotorState& msg);
    void _joint03Callback(const unitree_legged_msgs::MotorState& msg);
    void _joint04Callback(const unitree_legged_msgs::MotorState& msg);
    void _joint05Callback(const unitree_legged_msgs::MotorState& msg);
    void _gripperCallback(const unitree_legged_msgs::MotorState& msg);

};

#endif  // IOROS_H

#endif  // COMPILE_WITH_SIMULATION