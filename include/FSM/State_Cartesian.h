#ifndef CARTESIAN_H
#define CARTESIAN_H

#include "FSM/FSMState.h"

class State_Cartesian : public FSMState{
public:
    State_Cartesian(CtrlComponents *ctrlComp);
    ~State_Cartesian(){}
    void enter();
    void run();
    void exit();
    int checkChange(int cmd);
private:
    double _oriSpeed = 0.3;// control by keyboard
    double _posSpeed = 0.2;
    double oriSpeedLimit = 0.5;// limits in SDK
    double posSpeedLimit = 0.5;
    VecX _changeDirectionsf;

    Vec6 _twist;
    HomoMat _endHomoGoal, _endHomoPast, _endHomoDelta;
    Vec6 _endPostureGoal, _endPosturePast, _endPostureDelta;
};

#endif  // CARTESIAN_H