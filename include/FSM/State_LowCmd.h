#ifndef LOWCMD_H
#define LOWCMD_H

#include "FSMState.h"

class State_LowCmd : public FSMState{
public:
    State_LowCmd(CtrlComponents *ctrlComp);
    void enter();
    void run();
    void exit();
    int checkChange(int cmd);
private:
    std::vector<float> _kp;
    std::vector<float> _kw;
};

#endif  // LOWCMD_H