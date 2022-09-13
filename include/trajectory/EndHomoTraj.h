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

    Vec6 _deltaPosture;
    Vec3 _omgtheta;
    double _theta;
    Mat3 _startR, _endR, _delatR, _currentR;
    Vec3 _startp, _endp, _currentp;
    SCurve *_sCurve;
private:
    bool checkInSingularity();
    void _checkAngleValid(const Vec6 &q, int pointOrder);
    bool _checkJointAngleValid(const double &q, int jointOrder);
};

#endif  // ENDHOMOTRAJ_H