#ifndef CARTESIANPATH_H
#define CARTESIANPATH_H

#include "FSM/FSMState.h"
#include "trajectory/EndHomoTraj.h"
#include "trajectory/TrajectoryManager.h"

class State_Trajectory : public FSMState{
public:
    State_Trajectory(CtrlComponents *ctrlComp, 
        ArmFSMStateName stateEnum = ArmFSMStateName::TRAJECTORY, 
        std::string stateString = "trajectory");
    ~State_Trajectory();
    void enter();
    void run();
    void exit();
    int checkChange(int cmd);
protected:
    // EndHomoTraj *_traj;
    virtual void _setTraj();
    virtual void _setTrajSdk();
    TrajectoryManager *_traj;
    HomoMat _goalHomo;
    Vec6 _goalTwist;

    JointSpaceTraj *_toStartTraj;
    bool _reachedStart = false;
    bool _finishedTraj = false;
    std::vector<JointSpaceTraj*> _jointTraj;
    std::vector<EndCircleTraj*> _circleTraj;
    std::vector<StopForTime*> _stopTraj;
    std::vector<EndLineTraj*> _lineTraj;

    long long startTime;
    static std::vector<TrajCmd> _trajCmd;
    Vec6 _posture[2];

    static bool _isTrajFSM;
};

#endif  // CARTESIANPATH_H