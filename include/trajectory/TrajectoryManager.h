#ifndef TRAJECTORY_MANAGER_H
#define TRAJECTORY_MANAGER_H

#include <vector>
#include "control/CtrlComponents.h"
#include "trajectory/JointSpaceTraj.h"
#include "trajectory/EndLineTraj.h"
#include "trajectory/EndCircleTraj.h"
#include "trajectory/StopForTime.h"

class TrajectoryManager{
public:
    TrajectoryManager(CtrlComponents *ctrlComp);
    ~TrajectoryManager(){}
    bool getJointCmd(Vec6 &q, Vec6 &qd);
    bool getJointCmd(Vec6 &q, Vec6 &qd, double &gripperQ, double &gripperQd);
    void addTrajectory(Trajectory* traj);
    void setLoop(double backSpeed = 0.7);
    void restartTraj();
    void emptyTraj();
    Vec6 getStartQ();
    Vec6 getEndQ();
    Vec6 getEndPosture();
    double getStartGripperQ();
    double getEndGripperQ();
    HomoMat getEndHomo();
    size_t size() {return _trajVec.size();} ;
private:
    CtrlComponents *_ctrlComp;
    JointSpaceTraj *_trajBack;
    std::vector<Trajectory*> _trajVec;
    int _trajID = 0;
    double _jointErr = 0.05;
    bool _trajCorrect = true;
    bool _loop = false;
};

#endif  // TRAJECTORY_MANAGER_H