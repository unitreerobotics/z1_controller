#ifndef TRAJECTORY_H
#define TRAJECTORY_H

#include "common/math/mathTools.h"
#include "control/CtrlComponents.h"
#include "common/utilities/timer.h"

class Trajectory{
public:
    Trajectory(CtrlComponents *ctrlComp);
    virtual ~Trajectory(){}
    virtual bool getJointCmd(Vec6 &q, Vec6 &qd){return false;};
    virtual bool getJointCmd(Vec6 &q, Vec6 &qd, double &gripperQ, double &gripperQd){return false;};
    virtual bool getCartesionCmd(Vec6 pastPosture, Vec6 &endPosture, Vec6 &endTwist){return false;};

    void restart();
    virtual void setGripper(double startQ, double endQ, double speed = M_PI);

    bool correctYN(){return _settingCorrect;}
    Vec6 getStartQ(){return _startQ;}
    Vec6 getEndQ(){return _endQ;}
    double getEndGripperQ(){return _endGripperQ;};
    double getStartGripperQ(){return _startGripperQ;};
    HomoMat getStartHomo(){return _startHomo;};
    HomoMat getEndHomo(){return _endHomo;};
    Vec6 getEndPosture(){return _endPosture;};
    double getPathTime(){return _pathTime;}
protected:
    void _runTime();

    CtrlComponents *_ctrlComp;
    ArmModel *_armModel;
    CSVTool *_csvState;
    bool _pathStarted = false;
    bool _reached = false;
    bool _settingCorrect = true;
    double _startTime, _currentTime, _pathTime, _tCost;

    Vec6 _qPast;
    Vec6 _startQ, _endQ, _deltaQ;
    HomoMat _startHomo, _endHomo;
    Vec6 _startPosture, _endPosture, _deltaPosture;
    double _startGripperQ, _endGripperQ;
};

#endif  // TRAJECTORY_H