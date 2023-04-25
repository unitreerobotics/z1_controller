#ifndef ENDCIRCLETRAJ_H
#define ENDCIRCLETRAJ_H

#include "trajectory/EndHomoTraj.h"

class EndCircleTraj: public EndHomoTraj{
public:
    EndCircleTraj(CtrlComponents *ctrlComp);
    ~EndCircleTraj(){}

    void setEndRoundTraj(HomoMat startHomo, Vec3 axisPointFromInit, 
            Vec3 axisDirection, double maxSpeed, double angle,
            bool keepOrientation = true);
    void setEndRoundTraj(std::string stateName, Vec3 axisPointFromInit, 
            Vec3 axisDirection, double maxSpeed, double angle,
            bool keepOrientation = true);
    void setEndRoundTraj(Vec6 startP, Vec6 middleP, Vec6 endP, double speed);
private:
    void _centerCircle(Vec3 p1, Vec3 p2, Vec3 p3);
    bool _getEndTraj(HomoMat &homo, Vec6 &twist);
    Vec3 _center, _omegaAxis;
    double _radius, _theta;

    Vec6 _middlePosture;
    Vec6 _middleQ;
    HomoMat _initHomoToCenter, _middleHomo, _centerHomo;
    RotMat _initOri;

    bool _keepOrientation;
};

#endif  // ENDCIRCLETRAJ_H