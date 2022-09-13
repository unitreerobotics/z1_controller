#ifndef MOVEL_H
#define MOVEL_H

#include "FSM/FSMState.h"
#include "trajectory/CartesionSpaceTraj.h"

class State_MoveL : public FSMState{
public:
    State_MoveL(CtrlComponents *ctrlComp);
    ~State_MoveL();
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
    Vec6 _pastPosture, _endPosture, _endTwist;
    HomoMat _endHomoFeedback;
    Vec6 _Pdes;
    Vec6 _Pfd;
    Vec6 _Pkp;
    Vec6 _endPostureError;

    // 轨迹相关变量
    std::vector<std::vector<double> >  _posture;
    CartesionSpaceTraj *_cartesionTraj;
    bool _timeReached, _taskReached, _pastTaskReached, _finalReached;
};
#endif  // CARTESIAN_H