#ifndef ENDHOMOTRAJ_H
#define ENDHOMOTRAJ_H

#include "model/ArmModel.h"
#include "trajectory/Trajectory.h"
#include "trajectory/SCurve.h"

class EndHomoTraj : public Trajectory{
public:
    EndHomoTraj(CtrlComponents *ctrlComp);
    virtual ~EndHomoTraj();
    bool getJointCmd(Vec6 &q, Vec6 &qd);
    bool getJointCmd(Vec6 &q, Vec6 &qd, double &gripperQ, double &gripperQd);

protected:
    virtual bool _getEndTraj(HomoMat &homo, Vec6 &twist) = 0;

    HomoMat _cmdHomo;
    Vec6 _cmdTwist;

    SCurve _sCurve;
};

#endif  // ENDHOMOTRAJ_H