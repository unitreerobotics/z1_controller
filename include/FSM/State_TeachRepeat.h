#ifndef STATE_TEACHREPEAT_H
#define STATE_TEACHREPEAT_H

#include "FSM/FSMState.h"
#include "trajectory/TrajectoryManager.h"

class State_TeachRepeat : public FSMState{
public:
    State_TeachRepeat(CtrlComponents *ctrlComp);
    ~State_TeachRepeat();
    void enter();
    void run();
    void exit();
    int checkChange(int cmd);
private:
    bool _setCorrectly;
    JointSpaceTraj *_toStartTraj;
    bool _reachedStart = false;
    bool _finishedRepeat = false;
    size_t _index = 0;
    Vec6 _trajStartQ, _trajStartQd;
    double _trajStartGripperQ, _trajStartGripperQd;

    CSVTool *_csvFile;
};

#endif  // STATE_TEACHREPEAT_H