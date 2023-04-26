#ifndef STATE_BACKTOSTART_H
#define STATE_BACKTOSTART_H


#include "FSM/FSMState.h"
#include "trajectory/JointSpaceTraj.h"

class State_BackToStart : public FSMState{
public:
    State_BackToStart(CtrlComponents *ctrlComp);
    ~State_BackToStart();
    void enter();
    void run();
    void exit();
    int checkChange(int cmd);
private:
    bool _reach, _pastReach;
    JointSpaceTraj *_jointTraj;
    Vec6 _pos_startFlat;
};

#endif  // STATE_BACKTOSTART_H