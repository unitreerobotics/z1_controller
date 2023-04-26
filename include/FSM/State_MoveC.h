#ifndef MOVEC_H
#define MOVEC_H

#include "FSM/FSMState.h"
#include "trajectory/EndCircleTraj.h"

class State_MoveC : public FSMState{
public:
    State_MoveC(CtrlComponents *ctrlComp);
    ~State_MoveC();
    void enter();
    void run();
    void exit();
    int checkChange(int cmd);
private:
    double _speed;
    std::vector<Vec6>  _postures;
    EndCircleTraj *_circleTraj;
    bool _timeReached, _taskReached, _pastTaskReached, _finalReached;
    uint _taskCnt;
};

#endif  // CARTESIAN_H