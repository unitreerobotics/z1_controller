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
    JointSpaceTraj(ArmDynKineModel *armModel);
    JointSpaceTraj(ArmDynKineModel *armModel, CSVTool *csvState);
    ~JointSpaceTraj(){}

    void setGripper(double startQ, double endQ);
    bool getJointCmd(Vec6 &q, Vec6 &qd);
    bool getJointCmd(Vec6 &q, Vec6 &qd, double &gripperQ, double &gripperQd);
    void setJointTraj(Vec6 startQ, Vec6 endQ, double speed);
    bool setJointTraj(Vec6 startQ, std::string endName, double speed);
    void setJointTraj(std::string startName, std::string endName, double speed);

    void setJointTraj(Vec6 startQ, Vec6 middleQ, double middleS, Vec6 endQ, double speed);
    void setJointTraj(std::string startName, std::string middleName, double middleS, std::string endName, double speed);
    void setJointTraj(std::vector<Vec6> stateQ, std::vector<double> stateS, double pathTime);
    void setJointTraj(std::vector<std::string> stateName, std::vector<double> stateS, double pathTime);

private:
    void _checkAngleValid(const Vec6 &q, int pointOrder);
    bool _checkJointAngleValid(const double &q, int jointOrder);
    void _generateA345();

    SCurve _jointCurve;
    int _jointNum;
    double _pathTimeTemp;       // path total time
    double _s, _sDot;       // [0, 1]
    double _a3, _a4, _a5;   // parameters of path

    int _trajOrder;     // The order of trajectory curve
    std::vector<Vec6> _curveParam;

};

#endif  // JOINTSPACETRAJ_H