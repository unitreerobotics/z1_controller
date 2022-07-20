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
    double _costTime;
    HomoMat _goalHomo;
    JointSpaceTraj *_jointTraj;
    bool _reach, _pastReach;
    std::string _goalName;
    // CSVTool *_csv;
};

#endif  // TOSTATE_H