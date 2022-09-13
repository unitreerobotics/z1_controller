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
    double _posSpeed;
    double _oriSpeed;
    Vec3 _omega;
    Vec3 _velocity;
    Vec6 _twist;
    HomoMat _endHomoGoal, _endHomoGoalPast;
    Vec6 _endPostureGoal, _endPosturePast, _endPostureDelta;
    HomoMat _endHomoFeedback;
    Vec6 _Pdes;
    Vec6 _Pfd;
    Vec6 _Pkp;
};

#endif  // CARTESIAN_H