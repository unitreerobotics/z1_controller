#ifndef STATE_TEACHREPEAT_H
#define STATE_TEACHREPEAT_H

#include "FSM/FSMState.h"
#include "trajectory/TrajectoryManager.h"
#include "common/utilities/CSVTool.h"

class State_TeachRepeat : public FSMState{
public:
    State_TeachRepeat(CtrlComponents *ctrlComp);
    ~State_TeachRepeat();
    void enter();
    void run();
    void exit();
    int checkChange(int cmd);
private:
    JointSpaceTraj *_toStartTraj;
    bool _reachedStart = false;
    bool _finishedRepeat = false;
    size_t _index = 0;
    size_t _indexPast;
    Vec6 _trajStartQ, _trajStartQd;

    CSVTool *_csvFile;
};

#endif  // STATE_TEACHREPEAT_H