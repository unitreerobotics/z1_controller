#ifndef FSM_H
#define FSM_H

#include <vector>
#include "FSM/BaseState.h"
#include "unitree_arm_sdk/cmdPanel.h"
#include "unitree_arm_sdk/loop.h"

enum class FSMRunMode{
    NORMAL,
    CHANGE
};

/*
    状态机初始状态为states的第一个元素
    为了支持多个状态机从同一个CmdPanel获取触发信号，
    加入了cmdChannel，不同状态机之间的cmdChannel必须不同，
    否则可能会漏过状态触发
*/
class FiniteStateMachine{
public:
    FiniteStateMachine(std::vector<BaseState*> states,
        CmdPanel *cmdPanel, size_t cmdChannel = 0, double dt=0.002);
    virtual ~FiniteStateMachine();

private:
    void _run();
    static void* _staticRun(void* obj);
    std::vector<BaseState*> _states;

    FSMRunMode _mode;
    bool _running;
    BaseState* _currentState;
    BaseState* _nextState;
    int _nextStateEnum;

    size_t _cmdChannel;
    CmdPanel *_cmdPanel;
    LoopFunc *_runThread;
};

#endif  // FSM_H