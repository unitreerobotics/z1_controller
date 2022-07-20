#ifndef MOVEJ_H
#define MOVEJ_H

#include "FSM/FSMState.h"
#include "trajectory/JointSpaceTraj.h"

class State_MoveJ : public FSMState{
public:
    State_MoveJ(CtrlComponents *ctrlComp);
    ~State_MoveJ();
    void enter();
    void run();
    void exit();
    int checkChange(int cmd);
private:
    double _costTime;
    HomoMat _goalHomo;
    JointSpaceTraj *_jointTraj;
    bool _reached, _pastReached, _finalReached;
    std::vector<Vec6> _result;
};

#endif  // CARTESIAN_H