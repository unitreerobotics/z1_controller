#ifndef STOPFORTIME_H
#define STOPFORTIME_H

#include "trajectory/Trajectory.h"

class StopForTime : public Trajectory{
public:
    StopForTime(CtrlComponents *ctrlComp);
    ~StopForTime(){}

    bool getJointCmd(Vec6 &q, Vec6 &qd);
    bool getJointCmd(Vec6 &q, Vec6 &qd, double &gripperQ, double &gripperQd);
    void setStop(Vec6 stopQ, double stopTime);
private:
};

#endif  // STOPFORTIME_H