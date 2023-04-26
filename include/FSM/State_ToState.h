#ifndef TOSTATE_H
#define TOSTATE_H

#include "FSM/FSMState.h"
#include "trajectory/JointSpaceTraj.h"

class State_ToState : public FSMState{
public:
    State_ToState(CtrlComponents *ctrlComp);
    ~State_ToState();
    void enter();
    void run();
    void exit();
    int checkChange(int cmd);
private:
    bool _setCorrectly;
    double _costTime;
    HomoMat _goalHomo;
    JointSpaceTraj *_jointTraj;
    bool _reach, _pastReach;
    std::string _goalName;
};

#endif  // TOSTATE_H