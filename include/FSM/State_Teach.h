#ifndef STATETEACH_H
#define STATETEACH_H

#include "FSM/FSMState.h"

class State_Teach : public FSMState{
public:
    State_Teach(CtrlComponents *ctrlComp);
    ~State_Teach();
    void enter();
    void run();
    void exit();
    int checkChange(int cmd);
private:
    CSVTool *_trajCSV;
    size_t _stateID = 0;

    Vec6 _KdDiag;
    Vec6 _kpForStable;
    Mat6 _Kd;
    double _errorBias;
};

#endif  // STATETEACH_H