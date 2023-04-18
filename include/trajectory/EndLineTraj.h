#ifndef ENDLINETRAJ_H
#define ENDLINETRAJ_H

#include "trajectory/EndHomoTraj.h"
#include "trajectory/SCurve.h"

class EndLineTraj : public EndHomoTraj{
public:
    EndLineTraj(CtrlComponents *ctrlComp);
    ~EndLineTraj(){}

    bool setEndLineTraj(Vec6 startP, Vec6 endP, double speed);
    bool setEndLineTraj(HomoMat startHomo, Vec3 deltaPos, Vec3 deltaOri, double speed);
    bool setEndLineTraj(std::string stateName, Vec3 deltaPos, Vec3 deltaOri, double speed);
    bool setEndLineTraj(std::string startName, std::string endName, double speed);
private:
    bool _getEndTraj(HomoMat &homo, Vec6 &twist);

    Vec3 _currentDistance, _currentAngle, _omgtheta;
    Mat3 _startR, _endR, _delatR, _currentR;
    Vec3 _startp, _endp, _deltap, _currentp;
};

#endif