#ifndef LOWLEVELCMD_H
#define LOWLEVELCMD_H

#include "common/math/mathTypes.h"
#include "common/math/mathTools.h"
#include <vector>
#include <iostream>

struct LowlevelCmd{
private:
    size_t _dof = 6;
public:
    std::vector<double> q;
    std::vector<double> dq;
    std::vector<double> tau;
    std::vector<double> kp;
    std::vector<double> kd;


    LowlevelCmd(){
#ifndef UNITREE_GRIPPER
        q.resize(_dof);
        dq.resize(_dof);
        tau.resize(_dof);
        kp.resize(_dof);
        kd.resize(_dof);
#endif

#ifdef UNITREE_GRIPPER
        q.resize(_dof+1);
        dq.resize(_dof+1);
        tau.resize(_dof+1);
        kp.resize(_dof+1);
        kd.resize(_dof+1);
#endif
    }
    ~LowlevelCmd(){}

    void setZeroDq();
    void setZeroTau();
    void setZeroKp();
    void setZeroKd();
    void setControlGain();
    void setControlGain(std::vector<float> KP, std::vector<float> KW);
    void setQ(std::vector<double> qInput);
    void setQ(VecX qInput);
    void setQd(VecX qDInput);
    void setTau(VecX tauInput);
    void setPassive();
    void setGripperGain();
    void setGripperGain(float KP, float KW);
    void setGripperZeroGain();
    void setGripperQ(double qInput);
    void setGripperQd(double qdInput);
    void setGripperTau(double tauInput);
    Vec6 getQ();
    Vec6 getQd();
};


#endif  //LOWLEVELCMD_H
