#ifndef STATE_BACKTOSTART_H
#define STATE_BACKTOSTART_H

#include "FSM/State_Trajectory.h"

class State_BackToStart : public State_Trajectory{
public:
    State_BackToStart(CtrlComponents *ctrlComp);
private:
    void _setTraj();
    void _setTrajSDK(){}
};

#endif  // STATE_BACKTOSTART_H