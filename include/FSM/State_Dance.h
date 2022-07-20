#ifndef STATE_DANCETRAJ_H
#define STATE_DANCETRAJ_H

#include "FSM/State_Trajectory.h"

class State_Dance : public State_Trajectory{
public:
    State_Dance(CtrlComponents *ctrlComp, 
        TrajectoryManager *traj, 
        ArmFSMStateName stateEnum, 
        ArmFSMStateName nextState, 
        std::string stateString);
    void exit();
    int checkChange(int cmd);
private:
    bool _freeBottom = false;
    // ArmFSMStateName _nextState;
    void _setTraj(){}
};

#endif