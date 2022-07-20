#ifndef ENDLINETRAJ_H
#define ENDLINETRAJ_H

#include "trajectory/EndHomoTraj.h"
#include "trajectory/SCurve.h"

class EndLineTraj : public EndHomoTraj{
public:
    EndLineTraj(CtrlComponents *ctrlComp);
    EndLineTraj(ArmDynKineModel *armModel);
    ~EndLineTraj(){}

    void setEndLineTraj(HomoMat startHomo, Vec3 deltaPos, Vec3 deltaOri, double maxMovingSpeed, double maxTurningSpeed);
    void setEndLineTraj(std::string stateName, Vec3 deltaPos, Vec3 deltaOri, double maxMovingSpeed, double maxTurningSpeed);

private:
    bool _getEndTraj(HomoMat &homo, Vec6 &twist);

    // SCurve _sCurves[2];    // 0: position, 1: orientation
    Vec3 _movingAxis, _turningAxis;
    double _movingDistance, _turningAngle;
    Vec3 _currentDistance, _currentAngle;
    Vec3 _currentVelocity, _currentOmega;
       

    SCurve _posCurve;
    SCurve _oriCurve;
};

#endif  // ENDLINETRAJ_H