#ifndef JOINTSPACETRAJ_H
#define JOINTSPACETRAJ_H

#include <vector>
#include <trajectory/Trajectory.h>
#include <string>
#include "control/CtrlComponents.h"
#include "trajectory/SCurve.h"

class JointSpaceTraj : public Trajectory{
public:
    JointSpaceTraj(CtrlComponents *ctrlComp);
    ~JointSpaceTraj(){}

    bool getJointCmd(Vec6 &q, Vec6 &qd);
    bool getJointCmd(Vec6 &q, Vec6 &qd, double &gripperQ, double &gripperQd);
    
    void setGripper(double startQ, double endQ, double speed = M_PI);
    void setJointTraj(Vec6 startQ, Vec6 endQ, double speed);
    bool setJointTraj(Vec6 startQ, std::string endName, double speed);
    bool setJointTraj(std::string startName, std::string endName, double speed);
private:
    SCurve _jointCurve;
    double ddQMax;
    double dddQMax;
};

#endif  // JOINTSPACETRAJ_H