#ifndef STATE_SETTRAJ_H
#define STATE_SETTRAJ_H

#include "FSM/State_Trajectory.h"

class State_SetTraj: public State_Trajectory{
public:
    State_SetTraj(CtrlComponents *ctrlComp, 
        ArmFSMStateName stateEnum = ArmFSMStateName::SETTRAJ, 
        std::string stateString = "setTraj");
    ~State_SetTraj();
    void enter();
    void run();
    void exit();
    int checkChange(int cmd);
private:
    Vec6 _posture[2];
};

#endif