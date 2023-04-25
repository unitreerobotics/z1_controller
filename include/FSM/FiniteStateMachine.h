#ifndef FSM_H
#define FSM_H

#include <vector>
#include "FSM/FSMState.h"
#include "common/utilities/loop.h"
#include "control/CtrlComponents.h"

class FiniteStateMachine{
public:
    FiniteStateMachine(std::vector<FSMState*> states, CtrlComponents *ctrlComp);
    virtual ~FiniteStateMachine();

private:
    void _run();
    std::vector<FSMState*> _states;

    FSMMode _mode;
    bool _running;
    FSMState* _currentState;
    FSMState* _nextState;
    int _nextStateEnum;

    CtrlComponents *_ctrlComp;
    LoopFunc *_runThread;
};

#endif  // FSM_H