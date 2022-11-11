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
    double _oriSpeed = 0.4;// control by keyboard or joystick
    double _posSpeed = 0.15;
    double oriSpeedLimit = 0.3;// limits in SDK
    double posSpeedLimit = 0.3;
    VecX _changeDirections;

    Vec6 _twist;
    HomoMat _endHomoGoal, _endHomoGoalPast;
    Vec6 _endPostureGoal, _endPosturePast, _endPostureDelta;
};

#endif  // CARTESIAN_H