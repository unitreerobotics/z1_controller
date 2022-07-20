#ifndef MOVEC_H
#define MOVEC_H

#include "FSM/FSMState.h"
#include "trajectory/CartesionSpaceTraj.h"

class State_MoveC : public FSMState{
public:
    State_MoveC(CtrlComponents *ctrlComp);
    ~State_MoveC();
    void enter();
    void run();
    void exit();
    int checkChange(int cmd);
private:
    void quadprogArea();
    double _posSpeed;
    double _oriSpeed;
    Vec3 _omega;
    Vec3 _velocity;
    Vec6 _twist;
    Vec6 _pastPosture, _endPosture, _middlePostureGoal, _endPostureGoal, _endTwist;
    HomoMat _endHomoFeedback;
    Vec6 _Pdes;
    Vec6 _Pfd;
    Vec6 _Pkp;
    Vec6 _endPostureError;

    // 轨迹相关变量
    CartesionSpaceTraj *_cartesionTraj;
    bool _reached, _timeReached, _taskReached, _pastTaskReached;
};

#endif  // CARTESIAN_H