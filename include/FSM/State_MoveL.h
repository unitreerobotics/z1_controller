#ifndef MOVEL_H
#define MOVEL_H

#include "FSM/FSMState.h"
#include "trajectory/EndLineTraj.h"

class State_MoveL : public FSMState{
public:
    State_MoveL(CtrlComponents *ctrlComp);
    ~State_MoveL();
    void enter();
    void run();
    void exit();
    int checkChange(int cmd);
private:
    double _speed;
    std::vector<Vec6>  _postures;
    EndLineTraj *_lineTraj;
    bool _timeReached, _taskReached, _pastTaskReached, _finalReached;
    uint _taskCnt;
};
#endif  // CARTESIAN_H