#ifndef ENDHOMOTRAJ_H
#define ENDHOMOTRAJ_H

#include "model/ArmDynKineModel.h"
#include "trajectory/Trajectory.h"
#include "trajectory/SCurve.h"

class EndHomoTraj : public Trajectory{
public:
    EndHomoTraj(CtrlComponents *ctrlComp);
    EndHomoTraj(ArmDynKineModel *armModel);
    virtual ~EndHomoTraj();
    bool getJointCmd(Vec6 &q, Vec6 &qd);
    bool getJointCmd(Vec6 &q, Vec6 &qd, double &gripperQ, double &gripperQd);

protected:
    virtual bool _getEndTraj(HomoMat &homo, Vec6 &twist) = 0;

    HomoMat _cmdHomo;
    Vec6 _cmdTwist;
    Vec6 _qPast;
    // double _currentTime;

    SCurve *_sCurve;

};

#endif  // ENDHOMOTRAJ_H
