#include "IOROS.h"
#include <iostream>
#include <unistd.h>
#include <csignal>

void RosShutDown(int sig){
	ROS_INFO("ROS interface shutting down!");
	ros::shutdown();
}

IOROS::IOROS(){
    std::cout << "The control interface for ROS Gazebo simulation" << std::endl;
    hasGripper = false;
    /* start subscriber */
    _initRecv();
    ros::AsyncSpinner subSpinner(1); // one threads
    subSpinner.start();
    usleep(300000);     //wait for subscribers start
    /* initialize publisher */
    _initSend();   

    signal(SIGINT, RosShutDown);

    //check if there is UnitreeGripper
    unitree_legged_msgs::MotorCmd grippercmd;
    grippercmd.mode = 10;
    grippercmd.q = 0;
    grippercmd.dq = 0;
    grippercmd.tau = 0;
    grippercmd.Kp = 0;
    grippercmd.Kd = 0;
    _servo_pub[6].publish(grippercmd);
    ros::spinOnce();
}

IOROS::~IOROS(){
}

bool IOROS::sendRecv(const LowlevelCmd *cmd, LowlevelState *state){
    _sendCmd(cmd);
    _recvState(state);
    return true;
}

void IOROS::_sendCmd(const LowlevelCmd *lowCmd){
    for(int i(0); i < lowCmd->q.size(); ++i){
        _joint_cmd[i].mode = 10;
        _joint_cmd[i].q    = lowCmd->q[i];
        _joint_cmd[i].dq   = lowCmd->dq[i];
        _joint_cmd[i].tau  = lowCmd->tau[i];
        _joint_cmd[i].Kd   = lowCmd->kd[i]*0.0128;
        _joint_cmd[i].Kp   = lowCmd->kp[i]*25.6;
        _servo_pub[i].publish(_joint_cmd[i]);
    }
    ros::spinOnce();
}

void IOROS::_recvState(LowlevelState *state){
    for(int i(0); i < state->q.size(); ++i){
        state->q[i]   = _joint_state[i].q;
        state->dq[i]  = _joint_state[i].dq;
        state->ddq[i] = _joint_state[i].ddq;
        state->tau[i] = _joint_state[i].tauEst;
    }
}

void IOROS::_initSend(){
    _servo_pub[0] = _nm.advertise<unitree_legged_msgs::MotorCmd>("/z1_gazebo/Joint01_controller/command", 1);
    _servo_pub[1] = _nm.advertise<unitree_legged_msgs::MotorCmd>("/z1_gazebo/Joint02_controller/command", 1);
    _servo_pub[2] = _nm.advertise<unitree_legged_msgs::MotorCmd>("/z1_gazebo/Joint03_controller/command", 1);
    _servo_pub[3] = _nm.advertise<unitree_legged_msgs::MotorCmd>("/z1_gazebo/Joint04_controller/command", 1);
    _servo_pub[4] = _nm.advertise<unitree_legged_msgs::MotorCmd>("/z1_gazebo/Joint05_controller/command", 1);
    _servo_pub[5] = _nm.advertise<unitree_legged_msgs::MotorCmd>("/z1_gazebo/Joint06_controller/command", 1);
    _servo_pub[6] = _nm.advertise<unitree_legged_msgs::MotorCmd>("/z1_gazebo/gripper_controller/command", 1);
}

void IOROS::_initRecv(){
    _servo_sub[0] = _nm.subscribe("/z1_gazebo/Joint01_controller/state", 1, &IOROS::_joint00Callback, this);
    _servo_sub[1] = _nm.subscribe("/z1_gazebo/Joint02_controller/state", 1, &IOROS::_joint01Callback, this);
    _servo_sub[2] = _nm.subscribe("/z1_gazebo/Joint03_controller/state", 1, &IOROS::_joint02Callback, this);
    _servo_sub[3] = _nm.subscribe("/z1_gazebo/Joint04_controller/state", 1, &IOROS::_joint03Callback, this);
    _servo_sub[4] = _nm.subscribe("/z1_gazebo/Joint05_controller/state", 1, &IOROS::_joint04Callback, this);
    _servo_sub[5] = _nm.subscribe("/z1_gazebo/Joint06_controller/state", 1, &IOROS::_joint05Callback, this);
    _servo_sub[6] = _nm.subscribe("/z1_gazebo/gripper_controller/state", 1, &IOROS::_gripperCallback, this);
}

void IOROS::_joint00Callback(const unitree_legged_msgs::MotorState& msg){
    _joint_state[0].mode = msg.mode;
    _joint_state[0].q = msg.q;
    _joint_state[0].dq = msg.dq;
    _joint_state[0].ddq = msg.ddq;
    _joint_state[0].tauEst = msg.tauEst;
}

void IOROS::_joint01Callback(const unitree_legged_msgs::MotorState& msg){
    _joint_state[1].mode = msg.mode;
    _joint_state[1].q = msg.q;
    _joint_state[1].dq = msg.dq;
    _joint_state[1].ddq = msg.ddq;
    _joint_state[1].tauEst = msg.tauEst;
}

void IOROS::_joint02Callback(const unitree_legged_msgs::MotorState& msg){
    _joint_state[2].mode = msg.mode;
    _joint_state[2].q = msg.q;
    _joint_state[2].dq = msg.dq;
    _joint_state[2].ddq = msg.ddq;
    _joint_state[2].tauEst = msg.tauEst;
}

void IOROS::_joint03Callback(const unitree_legged_msgs::MotorState& msg){
    _joint_state[3].mode = msg.mode;
    _joint_state[3].q = msg.q;
    _joint_state[3].dq = msg.dq;
    _joint_state[3].ddq = msg.ddq;
    _joint_state[3].tauEst = msg.tauEst;
}

void IOROS::_joint04Callback(const unitree_legged_msgs::MotorState& msg){
    _joint_state[4].mode = msg.mode;
    _joint_state[4].q = msg.q;
    _joint_state[4].dq = msg.dq;
    _joint_state[4].ddq = msg.ddq;
    _joint_state[4].tauEst = msg.tauEst;
}

void IOROS::_joint05Callback(const unitree_legged_msgs::MotorState& msg){
    _joint_state[5].mode = msg.mode;
    _joint_state[5].q = msg.q;
    _joint_state[5].dq = msg.dq;
    _joint_state[5].ddq = msg.ddq;
    _joint_state[5].tauEst = msg.tauEst;
}

void IOROS::_gripperCallback(const unitree_legged_msgs::MotorState& msg){
    hasGripper = true;
    _joint_state[6].mode = msg.mode;
    _joint_state[6].q = msg.q;
    _joint_state[6].dq = msg.dq;
    _joint_state[6].ddq = msg.ddq;
    _joint_state[6].tauEst = msg.tauEst;
}